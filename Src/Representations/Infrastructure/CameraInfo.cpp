/**
* @file CameraInfo.cpp
* Implementation of class CameraInfo
*/

#include "CameraInfo.h"
#include "Image.h"

CameraInfo::CameraInfo() : resolutionWidth(0), resolutionHeight(0), openingAngleWidth(0), openingAngleHeight(0), focalLength(0.f) {}

void CameraInfo::calcAdditionalConstants()
{
  float scale = (float) cameraResolutionWidth / (float) resolutionWidth;
  resolutionWidth = cameraResolutionWidth;
  resolutionHeight = cameraResolutionHeight;
  opticalCenter *= scale; 
  focalLength *= scale;
  focalLenPow2 = focalLength * focalLength;
  focalLenPow4 = focalLenPow2 * focalLenPow2;
  focalLengthInv = 1.0f / focalLength;
}
