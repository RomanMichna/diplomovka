/** 
* @file ImageGridProvider3.cpp
* Implementation of a module that provides information 
* about how detailed different parts of the image are to be scanned
* @author <a href="allli@informatik.uni-bremen.de">Alexander Härtl</a> 
*/

#include "ImageGridProvider3.h"
#include "Tools/Range.h"
#include "Tools/Debugging/DebugDrawings.h"

#define LARGEST_INTERVAL 5

ImageGridProvider3::ImageGridProvider3()
{
}

void ImageGridProvider3::init()
{
  DECLARE_PLOT("module:ImageGridProvider3:residuum");
  DECLARE_PLOT("module:ImageGridProvider3:resolution");
  DECLARE_PLOT("module:ImageGridProvider3:coverage");
}

void ImageGridProvider3::update(ImageGrid& imageGrid)
{
  MODIFY("module:ImageGridProvider3:parameters", parameters);

  const Vector3<>& xAxis3D = theCameraMatrix.rotation.c0;
  const Vector2<> xAxis(Vector2<>(xAxis3D.x, xAxis3D.y).abs(), xAxis3D.z);

  a = xAxis.x / theCameraMatrix.translation.z;
  b = -(theCameraInfo.opticalCenter.y * xAxis.x + theCameraInfo.focalLength * xAxis.y) / theCameraMatrix.translation.z;
  // the width of one mm in pixels at (x,y) equals distortionA * y + distortionB

  // first compute limiting y-coordinate of the distance of the field diagonal
  {
    const float fieldDiagonal = 2.0f * Vector2<>((float)theFieldDimensions.xPosOpponentGroundline, (float)theFieldDimensions.yPosLeftSideline).abs();
    const Vector2<> transformedXAxis(xAxis.x, -xAxis.y);
    Vector2<> temp(Vector2<>(transformedXAxis).rotateRight() * theCameraMatrix.translation.z + transformedXAxis * fieldDiagonal);
    temp *= theCameraInfo.focalLength / temp.x;
    yStartCoord = theCameraInfo.opticalCenter.y - temp.y;
    yStartCoord = Range<>(0, (float)cameraResolutionHeight).limit(yStartCoord);
  }
  const int numOfGoalAreaScanLines = ((int)yStartCoord - 1) / parameters.resolutionGoal.y;
  yStart = parameters.resolutionGoal.y * numOfGoalAreaScanLines;

  const float timeForGoalArea = numOfGoalAreaScanLines * timePerLine(parameters.resolutionGoal.x);
  const float remainingTime = parameters.totalTime - timeForGoalArea;

  // use secant-method to get best matching resolution
  // as initial values use resolution of 2 and 4 pixels at remaining area's center
  const float yRef = (float)((yStart + cameraResolutionHeight) / 2);
  const float temp = a * yRef + b;
  float x = 2.0f / temp, lastX = 4.0f / temp, lastFX = approximateTime(lastX) - remainingTime;
  for(int i = 0; i < parameters.maxIterations; ++i)
  {
    const float fX = approximateTime(x) - remainingTime;
    const float divisor = fX - lastFX;
    if(abs(divisor) < 1e-3f)
      break;
    float temp = x;
    x = x - (x - lastX) / divisor * fX;
    if(x < 1.0f)
      x = 1.0f;
    lastX = temp;
    lastFX = fX;
  }

  // create actual imageGrid
  // first put in "goal area"
  int y = 0;
  const ImageGrid::LineInformation goalRes(parameters.resolutionGoal.y, parameters.resolutionGoal.x);
  for(int i = 0; i < numOfGoalAreaScanLines; ++i)
  {
    imageGrid.lineInformation[i] = imageGrid.lineInformationLUT[y] = goalRes;
    y += goalRes.rowInterval;
  }
  imageGrid.lastYCoord = y - goalRes.rowInterval;
  ImageGrid::LineInformation* p = imageGrid.lineInformation + numOfGoalAreaScanLines;
  vector<ResolutionChange> resolutionChanges;
  ImageGrid::LineInformation currentRes;
  computeResolutionChanges(resolutionChanges, currentRes, x);
  for(vector<ResolutionChange>::const_iterator i = resolutionChanges.begin(); i != resolutionChanges.end(); ++i)
  {
    if(y < i->y)
    {
      while(y < i->y)
      {
        *p = imageGrid.lineInformationLUT[y] = currentRes;
        ++p;
        y += currentRes.rowInterval;
      }
      imageGrid.lastYCoord = y - currentRes.rowInterval;
    }
    ++(i->horizontal ? currentRes.columnInterval : currentRes.rowInterval);
  }

  p[-1].rowInterval -= y - cameraResolutionHeight;
  imageGrid.lineInformationLUT[cameraResolutionHeight - p[-1].rowInterval].rowInterval = p[-1].rowInterval;

  imageGrid.numberOfLines = p - imageGrid.lineInformation;

  PLOT("module:ImageGridProvider3:residuum", lastFX);
  PLOT("module:ImageGridProvider3:resolution", x);

#ifndef RELEASE
  imageGrid.draw(theImage);
  PLOT("module:ImageGridProvider3:coverage", (float)imageGrid.getNumOfSamples() / ((float)cameraResolutionWidth * (float)cameraResolutionHeight));
#endif
}

void ImageGridProvider3::computeResolutionChanges(vector<ResolutionChange>& resolutionChanges, ImageGrid::LineInformation& initialRes, float resolution) const
{
  initialRes = ImageGrid::LineInformation(1, 1);
  for(int i = 0; i < 2; ++i)
  {
    const bool horizontal = i == 0;
    const float res = horizontal ? resolution : resolution * parameters.horVerRatio;
    const int maxInterval = horizontal ? parameters.maxInterval.x : parameters.maxInterval.y;
    unsigned char& initialInterval = horizontal ? initialRes.columnInterval : initialRes.rowInterval;
    for(int j = 1; j < maxInterval; ++j)
    {
      ResolutionChange resChange;
      resChange.horizontal = horizontal;
      resChange.y = (int)((((float)j + 0.5f) / res - b) / a);
      if(resChange.y <= yStart)
      {
        ++initialInterval;
      }
      else if(resChange.y < cameraResolutionHeight)
        resolutionChanges.push_back(resChange);
      else
        break;
    }
  }
  ResolutionChange lastResChange;
  lastResChange.y = cameraResolutionHeight;
  resolutionChanges.push_back(lastResChange);

  sort(resolutionChanges.begin(), resolutionChanges.end());
}

float ImageGridProvider3::approximateTime(float resolution) const
{
  vector<ResolutionChange> resolutionChanges;
  ImageGrid::LineInformation currentLineInf;
  computeResolutionChanges(resolutionChanges, currentLineInf, resolution);

  int y = yStart;
  float time = 0;
  for(vector<ResolutionChange>::const_iterator i = resolutionChanges.begin(); i != resolutionChanges.end(); ++i)
  {
    if(i->y > y)
    {
      const int numOfLines = ((i->y - y) - 1) / currentLineInf.rowInterval + 1;
      time += timePerLine(currentLineInf.columnInterval) * (float)numOfLines;
      y += numOfLines * currentLineInf.rowInterval;
    }
    ++(i->horizontal ? currentLineInf.columnInterval : currentLineInf.rowInterval);
  }
  return time;
}

float ImageGridProvider3::timePerLine(int columnInterval) const
{
  if(columnInterval == 1)
    return parameters.timeFullResLine;
  else
    return (float)((cameraResolutionWidth - 1) / columnInterval + 1) * parameters.timePerPixel + parameters.addTimePerLine;
}

MAKE_MODULE(ImageGridProvider3, Perception)
