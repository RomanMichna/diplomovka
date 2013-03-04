/**
* @file ExpCameraProvider.cpp
* This file declares a module that provides camera images.
* @author Colin Graf
*/

#include <cstdio>

#include "ExpCameraProvider.h"
#include "Representations/Perception/JPEGImage.h"
#include "Platform/Camera.h"
#include "Platform/SystemCall.h"
#include "Platform/SoundPlayer.h"

PROCESS_WIDE_STORAGE(ExpCameraProvider) ExpCameraProvider::theInstance = 0;

ExpCameraProvider::ExpCameraProvider() : currentImageCamera(0), imageTimeStamp(0), otherImageTimeStamp(0), lastImageTimeStamp(0), lastImageTimeStampLL(0)
{
#ifdef CAMERA_INCLUDED
  camera = new NaoCamera("/dev/video0", ImageInfo::upperCamera, true);
  lowerCamera = new NaoCamera("/dev/video1", ImageInfo::lowerCamera, false);
  cycleTime = camera->getFrameRate();
  ASSERT(camera->getFrameRate() == lowerCamera->getFrameRate());
#else
  camera = lowerCamera = NULL;
  cycleTime = 1.f / 30.f;
#endif
  theInstance = this;
}

ExpCameraProvider::~ExpCameraProvider()
{
#ifdef CAMERA_INCLUDED
  if(camera)
    delete camera;
  if(lowerCamera)
    delete lowerCamera;
#endif
  theInstance = 0;
}

void ExpCameraProvider::update(Image& image)
{
#ifdef CAMERA_INCLUDED
  ASSERT(!currentImageCamera);
  if(camera->hasImage() && (!lowerCamera->hasImage() || camera->getTimeStamp() < lowerCamera->getTimeStamp()))
  {
    image.setImage(const_cast<unsigned char*>(camera->getImage()));
    lastImageTimeStampLL = camera->getTimeStamp();
    imageTimeStamp = image.timeStamp = std::max(lastImageTimeStamp + 1, (unsigned) (camera->getTimeStamp() / 1000) - SystemCall::getSystemTimeBase());
    camera->setSettings(theCameraSettings);
    camera->writeCameraSettings();
    currentImageCamera = camera;

    imageInfo.prevCamera = imageInfo.camera;
    imageInfo.camera = camera->getCurrentCamera();
  }
  else if(lowerCamera->hasImage())
  {
    image.setImage(const_cast<unsigned char*>(lowerCamera->getSSEImage()));
    lastImageTimeStampLL = lowerCamera->getTimeStamp();
    otherImageTimeStamp = image.timeStamp = std::max(lastImageTimeStamp + 1, (unsigned) (lowerCamera->getTimeStamp() / 1000) - SystemCall::getSystemTimeBase());
    lowerCamera->setSettings(theCameraSettings);
    lowerCamera->writeCameraSettings();
    currentImageCamera = lowerCamera;

    imageInfo.prevCamera = imageInfo.camera;
    imageInfo.camera = lowerCamera->getCurrentCamera();
  }
  imageInfo.whiteBalanced = camera->whiteBalanced && lowerCamera->whiteBalanced;
  imageInfo.toggling = true;
  ASSERT(image.timeStamp >= lastImageTimeStamp);
  lastImageTimeStamp = image.timeStamp;
#else
  imageInfo.whiteBalanced = true;
#endif // CAMERA_INCLUDED
  DEBUG_RESPONSE("representation:JPEGImage", OUTPUT(idJPEGImage, bin, JPEGImage(image)););
}

void ExpCameraProvider::update(ImageInfo& imageInfo)
{
#ifdef CAMERA_INCLUDED
  imageInfo = this->imageInfo;
#endif
}

void ExpCameraProvider::update(FrameInfo& frameInfo)
{
  frameInfo.time = theImage.timeStamp;
  frameInfo.cycleTime = cycleTime * 0.5f;
}

void ExpCameraProvider::update(CognitionFrameInfo& cognitionFrameInfo)
{
  cognitionFrameInfo.time = theImage.timeStamp;
  cognitionFrameInfo.cycleTime = cycleTime * 0.5f;
}

bool ExpCameraProvider::isFrameDataComplete()
{
#ifdef CAMERA_INCLUDED
  if(theInstance)
    return theInstance->camera->hasImage() || theInstance->lowerCamera->hasImage();
  else
#endif
    return true;
}

void ExpCameraProvider::waitForFrameData2()
{
#ifdef CAMERA_INCLUDED

  const unsigned int timeout = 1000 * 10;

  if(currentImageCamera)
  {
    currentImageCamera->releaseImage();
    currentImageCamera = 0;
  }

  for(;;)
  {
    if(camera->hasImage() || lowerCamera->hasImage())
      return;

    bool reset = false, resetOther = false;
    if(!NaoCamera::captureNew(*camera, *lowerCamera, timeout, reset, resetOther))
    {
      OUTPUT_WARNING("ExpCameraProvider: Poll failed. Resetting both cameras");
      reset = resetOther = true;
    }
    else
    {
      if(reset)
        OUTPUT_WARNING("ExpCameraProvider: Capturing image failed. Resetting " << ImageInfo::getName(camera->getCurrentCamera()) << ".");
      if(resetOther)
        OUTPUT_WARNING("ExpCameraProvider: Capturing image failed. Resetting " << ImageInfo::getName(lowerCamera->getCurrentCamera()) << ".");
    }
  
    unsigned int now = SystemCall::getRealSystemTime();
    if(!reset && imageTimeStamp && now - imageTimeStamp >= timeout)
    {
      OUTPUT_WARNING("ExpCameraProvider: Capturing image timed out. Resetting " << ImageInfo::getName(camera->getCurrentCamera()) << ".");
      reset = true;
    }
    if(!resetOther && otherImageTimeStamp && now - otherImageTimeStamp >= timeout)
    {
      OUTPUT_WARNING("ExpCameraProvider: Capturing image timed out. Resetting " << ImageInfo::getName(lowerCamera->getCurrentCamera()) << ".");
      resetOther = true;
    }

    if(reset)
    {
      ImageInfo::Camera c = camera->getCurrentCamera();
      delete camera;
      camera = new NaoCamera(c == ImageInfo::upperCamera ? "/dev/video0" : "/dev/video1", c, c == ImageInfo::upperCamera);
      imageTimeStamp = 0;
      SoundPlayer::play("cameraReset.wav");
    }
  
    if(resetOther)
    {
      ImageInfo::Camera c = lowerCamera->getCurrentCamera();
      delete lowerCamera;
      lowerCamera = new NaoCamera(c == ImageInfo::upperCamera ? "/dev/video0" : "/dev/video1", c, c == ImageInfo::upperCamera);
      otherImageTimeStamp = 0;
      SoundPlayer::play("cameraReset.wav");
    }

    if(camera->hasImage() && camera->getTimeStamp() < lastImageTimeStampLL)
      camera->releaseImage();
    if(lowerCamera->hasImage() && lowerCamera->getTimeStamp() < lastImageTimeStampLL)
      lowerCamera->releaseImage();
  }

#endif
}

void ExpCameraProvider::waitForFrameData()
{
#ifdef CAMERA_INCLUDED
  if(theInstance)
    theInstance->waitForFrameData2();
#endif
}

MAKE_MODULE(ExpCameraProvider, Infrastructure)
