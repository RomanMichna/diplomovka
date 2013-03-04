/**
* @file CameraProvider.cpp
* This file declares a module that provides camera images.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#include <cstdio>

#include "CameraProvider.h"
#include "Representations/Perception/JPEGImage.h"
#include "Platform/Camera.h"
#include "Platform/SystemCall.h"
#include "Platform/SoundPlayer.h"
#include "Tools/Debugging/DebugDrawings.h"

PROCESS_WIDE_STORAGE(CameraProvider) CameraProvider::theInstance = 0;

CameraProvider::CameraProvider() : lastSwitchTime(0), bothCameras(false)
{
#ifdef CAMERA_INCLUDED
  camera = new NaoCamera("/dev/video0", ImageInfo::upperCamera, true);
  otherCamera = new NaoCamera("/dev/video1", ImageInfo::lowerCamera, false);
  cycleTime = camera->getFrameRate();
#else
  camera = otherCamera = NULL;
  cycleTime = 1.f / 30.f;
#endif
  theInstance = this;
}

CameraProvider::~CameraProvider()
{
#ifdef CAMERA_INCLUDED
  if(camera)
    delete camera;
  if(otherCamera)
    delete otherCamera;
#endif
  theInstance = 0;
}

void CameraProvider::update(Image& image)
{
#ifdef CAMERA_INCLUDED
  DECLARE_PLOT("module:CameraProvider:timeCameraSettings");
  DECLARE_PLOT("module:CameraProvider:timeFetchImage");
  ASSERT(camera->hasImage());
  image.setImage(const_cast<unsigned char*>(camera->getImage()));
  image.timeStamp = (unsigned) (camera->getTimeStamp() / 1000) - SystemCall::getSystemTimeBase();
  camera->setSettings(theCameraSettings);
  const unsigned fetchTime = camera->writeCameraSettings();
  PLOT("module:CameraProvider:timeCameraSettings", fetchTime);
  PLOT("module:CameraProvider:timeFetchImage", camera->timeWaitedForLastImage);
  DEBUG_RESPONSE("module:CameraProvider:assertCameraSettings", camera->assertCameraSettings(););
  DEBUG_RESPONSE("module:CameraProvider:writeCameraSettings", camera->writeCameraSettings(););
#endif // CAMERA_INCLUDED
  DEBUG_RESPONSE("representation:JPEGImage", OUTPUT(idJPEGImage, bin, JPEGImage(image)););
}

void CameraProvider::update(ImageOther& image)
{
#ifdef CAMERA_INCLUDED
  if(bothCameras)
  {
    ASSERT(otherCamera->hasImage());
    image.setImage(const_cast<unsigned char*>(otherCamera->getImage()));
    image.timeStamp = (unsigned) (otherCamera->getTimeStamp() / 1000) - SystemCall::getSystemTimeBase();
    otherCamera->setSettings(theCameraSettings);
    DEBUG_RESPONSE("module:CameraProvider:assertCameraSettings", otherCamera->assertCameraSettings(););
    DEBUG_RESPONSE("module:CameraProvider:writeCameraSettings", otherCamera->writeCameraSettings(););
  }
#endif // CAMERA_INCLUDED
  DEBUG_RESPONSE("representation:JPEGImageOther", OUTPUT(idJPEGImageOther, bin, JPEGImage(image)););
}

void CameraProvider::update(ImageInfo& imageInfo)
{
  imageInfo = theImageInfo;
  theImageInfo.toggling = theImageRequest.requestedCamera == ImageInfo::toggleCameras;
  
#ifdef CAMERA_INCLUDED
  DEBUG_RESPONSE("module:CameraProvider:timeCameraSwitch",
  {
    if(theImageInfo.cameraChanged())
    {
      unsigned diff = SystemCall::getRealSystemTime() - lastSwitchTime;
      OUTPUT(idText, text, "Switching cameras took: " << diff);
    }
    lastSwitchTime = SystemCall::getRealSystemTime();
  });
  theImageInfo.prevCamera = theImageInfo.camera;
  if(theImageInfo.toggling)
  {
    NaoCamera* temp = camera;
    camera = otherCamera;
    otherCamera = temp;
    theImageInfo.camera = camera->getCurrentCamera();
  }
  else
  {  
  ImageInfo::Camera currentCamera = camera->getCurrentCamera();
  bothCameras = theImageRequest.requestedCamera == ImageInfo::bothCameras;
  if((bothCameras && currentCamera != ImageInfo::upperCamera) ||
     (!bothCameras && currentCamera != theImageRequest.requestedCamera))
  {
    NaoCamera* temp = camera;
    camera = otherCamera;
    otherCamera = temp;
    if(theImageInfo.prevCamera != ImageInfo::bothCameras)
      for(int i = 0; i < 3; ++i)
        camera->captureNew(); // currentCamera was inactive until now -> skip old images
  }
  else if(bothCameras && theImageInfo.prevCamera != ImageInfo::bothCameras)
    for(int i = 0; i < 3; ++i)
      otherCamera->captureNew(); // otherCamera was inactive until now -> skip old images
  theImageInfo.camera = theImageRequest.requestedCamera;
  }
  imageInfo.whiteBalanced = camera->whiteBalanced;
#endif
}

void CameraProvider::update(FrameInfo& frameInfo)
{
  frameInfo.time = theImage.timeStamp;
  frameInfo.cycleTime = theImageRequest.requestedCamera == ImageInfo::toggleCameras ? cycleTime / 2.f : cycleTime;
}

void CameraProvider::update(CognitionFrameInfo& cognitionFrameInfo)
{
  cognitionFrameInfo.time = theImage.timeStamp;
  cognitionFrameInfo.cycleTime = theImageRequest.requestedCamera == ImageInfo::toggleCameras ? cycleTime / 2.f : cycleTime;
}

bool CameraProvider::isFrameDataComplete()
{
#ifdef CAMERA_INCLUDED
  if(theInstance)
    return theInstance->camera->hasImage() && (!theInstance->bothCameras || theInstance->otherCamera->hasImage());
  else
#endif
    return true;
}

void CameraProvider::waitForFrameData()
{
#ifdef CAMERA_INCLUDED
  if(theInstance)
    do
    {
      bool reset = false;
      bool resetOther = false;
      DEBUG_RESPONSE_ONCE("module:CameraProvider:resetCamera", if(theInstance) reset = true;);
      
      if(!theInstance->camera->captureNew())
      {
        OUTPUT_WARNING("CameraProvider: Capturing image failed. Resetting camera.");
        reset = true;
      }
      
      if(theInstance->bothCameras && !theInstance->otherCamera->captureNew())
      {
        OUTPUT_WARNING("CameraProvider: Capturing image failed. Resetting other camera.");
        resetOther = true;
      }
      
      if(reset)
      {
        SoundPlayer::play("cameraReset.wav");
        ImageInfo::Camera c = theInstance->camera->getCurrentCamera();
        delete theInstance->camera;
        theInstance->camera = new NaoCamera(c == ImageInfo::upperCamera ? "/dev/video0" : "/dev/video1", c, c == ImageInfo::upperCamera);
      }
      
      if(resetOther)
      {
        SoundPlayer::play("cameraReset.wav");
        ImageInfo::Camera c = theInstance->otherCamera->getCurrentCamera();
        delete theInstance->otherCamera;
        theInstance->otherCamera = new NaoCamera(c == ImageInfo::upperCamera ? "/dev/video0" : "/dev/video1", c, c == ImageInfo::upperCamera);
      }
    }
    while(theInstance->camera->getTimeStamp() < theInstance->otherCamera->getTimeStamp());
#endif
}

MAKE_MODULE(CameraProvider, Infrastructure)
