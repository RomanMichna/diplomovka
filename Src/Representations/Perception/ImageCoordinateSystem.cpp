/**
* @file ImageCoordinateSystem.cpp
* Implementation of a class that provides transformations on image coordinates.
* Parts of this class were copied from class ImageInfo.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
* @author <a href="mailto:oberlies@sim.tu-darmstadt.de">Tobias Oberlies</a>
*/

#include "ImageCoordinateSystem.h"

ImageCoordinateSystem::ImageCoordinateSystem() : xTable(0), yTable(0) {}

ImageCoordinateSystem::~ImageCoordinateSystem()
{
  if(xTable)
  {
    delete[] xTable;
    delete[] yTable;
  }
}

void ImageCoordinateSystem::setCameraInfo(const CameraInfo& cameraInfo)
{
  if(xTable)
    return;
  this->cameraInfo = cameraInfo;
  xTable = new int[cameraInfo.resolutionWidth];
  yTable = new int[cameraInfo.resolutionHeight];
  for(int i = 0; i < cameraInfo.resolutionWidth; ++i)
    xTable[i] = int(::atan((cameraInfo.opticalCenter.x - i) / cameraInfo.focalLength) * 1024 + 0.5f);
  for(int i = 0; i < cameraInfo.resolutionHeight; ++i)
    yTable[i] = int(::atan((i - cameraInfo.opticalCenter.y) / cameraInfo.focalLength) * 1024 + 0.5f);
  for(int i = -3072; i < 3072; ++i)
    table[i + 3072] = int(::tan(i / 1024.0f) * cameraInfo.focalLength + 0.5f);
}
