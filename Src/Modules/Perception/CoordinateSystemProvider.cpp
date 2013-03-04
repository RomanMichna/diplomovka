/**
* @file CoordinateSystemProvider.cpp
* This file implements a module that provides a coordinate system in image coordinates
* that is parallel to the ground and compensates for distortions resulting from the
* rolling shutter.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#include "CoordinateSystemProvider.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Geometry.h"

void CoordinateSystemProvider::update(ImageCoordinateSystem& imageCoordinateSystem)
{
  imageCoordinateSystem.setCameraInfo(theCameraInfo);

  Geometry::Line horizon = Geometry::calculateHorizon(theCameraMatrix, theCameraInfo);
  imageCoordinateSystem.origin = horizon.base;
  imageCoordinateSystem.rotation.c[0] = horizon.direction;
  imageCoordinateSystem.rotation.c[1] = Vector2<>(-horizon.direction.y, horizon.direction.x);
  imageCoordinateSystem.invRotation = imageCoordinateSystem.rotation.transpose();

  const CameraMatrix& cmPrev = cameraMatrixPrev[theImageInfo.camera];
  RotationMatrix r(theCameraMatrix.rotation.transpose() * cmPrev.rotation);
  imageCoordinateSystem.offset = Vector2<>(r.getZAngle(), r.getYAngle());

  calcScaleFactors(imageCoordinateSystem.a, imageCoordinateSystem.b, theFilteredJointData.timeStamp - cameraMatrixPrevTimeStamp[theImageInfo.camera]);
  imageCoordinateSystem.offsetInt = Vector2<int>(int(imageCoordinateSystem.offset.x * 1024 + 0.5f), 
                                                 int(imageCoordinateSystem.offset.y * 1024 + 0.5f));
  imageCoordinateSystem.aInt = int(imageCoordinateSystem.a * 1024 + 0.5f);
  imageCoordinateSystem.bInt = int(imageCoordinateSystem.b * 1024 + 0.5f);
  
  cameraMatrixPrev[theImageInfo.camera] = theCameraMatrix;
  cameraMatrixPrevTimeStamp[theImageInfo.camera] = theFilteredJointData.timeStamp;
  if(!theImageInfo.toggling)
  {
    ImageInfo::Camera otherCamera = theImageInfo.camera == ImageInfo::upperCamera ? ImageInfo::lowerCamera : ImageInfo::upperCamera;
    cameraMatrixPrev[otherCamera] = theCameraMatrixOther;
    cameraMatrixPrevTimeStamp[otherCamera] = theFilteredJointData.timeStamp;
  }

  DECLARE_DEBUG_DRAWING("horizon", "drawingOnImage"); // displays the horizon
  ARROW("horizon",
        imageCoordinateSystem.origin.x,
        imageCoordinateSystem.origin.y,
        imageCoordinateSystem.origin.x + imageCoordinateSystem.rotation.c[0].x * 50,
        imageCoordinateSystem.origin.y + imageCoordinateSystem.rotation.c[0].y * 50,
        0, Drawings::ps_solid, ColorRGBA(255, 0, 0));
  ARROW("horizon",
        imageCoordinateSystem.origin.x,
        imageCoordinateSystem.origin.y,
        imageCoordinateSystem.origin.x + imageCoordinateSystem.rotation.c[1].x * 50,
        imageCoordinateSystem.origin.y + imageCoordinateSystem.rotation.c[1].y * 50,
        0, Drawings::ps_solid, ColorRGBA(255, 0, 0));
  COMPLEX_DEBUG_IMAGE(corrected,
  {
    INIT_DEBUG_IMAGE_BLACK(corrected);
    int yDest = -imageCoordinateSystem.toCorrectedCenteredNeg(0, 0).y;
    for(int ySrc = 0; ySrc < theImage.resolutionHeight; ++ySrc)
      for(int yDest2 = -imageCoordinateSystem.toCorrectedCenteredNeg(0, ySrc).y; yDest <= yDest2; ++yDest)
      {
        int xDest = -imageCoordinateSystem.toCorrectedCenteredNeg(0, ySrc).x;
        for(int xSrc = 0; xSrc < theImage.resolutionWidth; ++xSrc)
        {
          for(int xDest2 = -imageCoordinateSystem.toCorrectedCenteredNeg(xSrc, ySrc).x; xDest <= xDest2; ++xDest)
          {
            DEBUG_IMAGE_SET_PIXEL_YUV(corrected, xDest + int(theCameraInfo.opticalCenter.x + 0.5f), 
                                                 yDest + int(theCameraInfo.opticalCenter.y + 0.5f),
                                                 theImage.image[ySrc][xSrc].y, 
                                                 theImage.image[ySrc][xSrc].cb,
                                                 theImage.image[ySrc][xSrc].cr);
          }
        }
      }
    SEND_DEBUG_IMAGE(corrected);
  });

  COMPLEX_DEBUG_IMAGE(horizonAligned,
  {
    INIT_DEBUG_IMAGE_BLACK(horizonAligned);
    for(int ySrc = 0; ySrc < theCameraInfo.resolutionHeight; ++ySrc)
      for(int xSrc = 0; xSrc < theCameraInfo.resolutionWidth; ++xSrc)
      {
        Vector2<> corrected(imageCoordinateSystem.toCorrected(Vector2<int>(xSrc, ySrc)));
        corrected.x -= theCameraInfo.opticalCenter.x;
        corrected.y -= theCameraInfo.opticalCenter.y;
        const Vector2<>& horizonAligned(imageCoordinateSystem.toHorizonAligned(corrected));

        DEBUG_IMAGE_SET_PIXEL_YUV(horizonAligned, int(horizonAligned.x + theCameraInfo.opticalCenter.x + 0.5f),
                                                  int(horizonAligned.y + theCameraInfo.opticalCenter.y + 0.5f),
                                                  theImage.image[ySrc][xSrc].y,
                                                  theImage.image[ySrc][xSrc].cb,
                                                  theImage.image[ySrc][xSrc].cr);
      }
    SEND_DEBUG_IMAGE(horizonAligned);
  });
}

void CoordinateSystemProvider::calcScaleFactors(float& a, float& b, unsigned int abTimeDiff) const
{
  float imageRecordingTime = theRobotDimensions.imageRecordingTime,
        imageRecordingDelay = theRobotDimensions.imageRecordingDelay;
  if(abTimeDiff)
  {
    float timeDiff = (float) int(abTimeDiff) * 0.001f; // in seconds
    float timeDiff2 = (float) int(theFrameInfo.time - theFilteredJointData.timeStamp) * 0.001f; // in seconds
    a = (timeDiff2 - imageRecordingTime - imageRecordingDelay) / timeDiff;
    b = imageRecordingTime / theImage.resolutionHeight / timeDiff;
  }
  else
    a = b = 0;
}

MAKE_MODULE(CoordinateSystemProvider, Perception)
