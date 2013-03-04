/**
* @file GoalPerceptor.h
* @author jeff
* @author moe
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Perception/GoalPercept.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/RegionPercept.h"
#include "Representations/Configuration/ColorTable64.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Tools/Storage.h"
#include "Tools/Debugging/DebugImages.h"

#ifndef MAX_GOAL_SPOTS
#define MAX_GOAL_SPOTS 8
#endif

MODULE(GoalPerceptor)
  REQUIRES(CameraMatrix)
  REQUIRES(ImageCoordinateSystem)
  REQUIRES(CameraInfo)
  REQUIRES(Image)
  REQUIRES(ColorTable64)
  REQUIRES(FieldDimensions)
  REQUIRES(FrameInfo)
  REQUIRES(OwnTeamInfo)
  REQUIRES(RegionPercept)
  PROVIDES_WITH_DRAW(GoalPercept)
END_MODULE

/**
 * @class GoalPerceptor
 */
class GoalPerceptor: public GoalPerceptorBase
{
private:
  class Parameters : public Streamable
  {
  private:
    void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN;
      STREAM(stepSize);
      STREAM(maxSkip);
      STREAM(minSegmentLength);
      STREAM(minYRunDiff);
      STREAM(minPostPixelHeight);
      STREAM(widthStepSize);
      STREAM(maxWidthErrorRatio);
      STREAM(minWidthErrorRatio);
      STREAM(maxWrongSizeRatio);
      STREAM(maxDistanceError);
      STREAM(maxHeadCloserThanFoot);
      STREAM(minCenterDistance);
      STREAM(maxGreenScan);
      STREAM(minGreenBelow);
      STREAM(greenScanOffset);
      STREAM_REGISTER_FINISH;
    }

  public:
    Parameters() :
      stepSize(2),
      maxSkip(4),
      minSegmentLength(2),
      minYRunDiff(4),
      minPostPixelHeight(40),
      minHeadVisibleOffset(5),
      widthStepSize(10),
      maxGreenScan(10),
      minGreenBelow(5),
      greenScanOffset(2),
      maxWidthErrorRatio(1.5f),
      minWidthErrorRatio(0.4f),
      maxWrongSizeRatio(0.25f),
      maxDistanceError(0.5f),
      maxHeadCloserThanFoot(500),
      minCenterDistance(20)
    {}
    int stepSize,
        maxSkip,
        minSegmentLength,
        minYRunDiff,
        minPostPixelHeight,
        minHeadVisibleOffset,
        widthStepSize,
        maxGreenScan,
        minGreenBelow,
        greenScanOffset;
    float maxWidthErrorRatio,
          minWidthErrorRatio,
          maxWrongSizeRatio,
          maxDistanceError,
          maxHeadCloserThanFoot,
          minCenterDistance;
  };

  class Spot
  {
  public:
    ColorClasses::Color color;
    Vector2<> head, foot;
    Vector2<> center;
    int avrgWidth;
    bool headVisible,
         footVisible,
         dead;
    Vector2<> onField;
    enum
    {
      unknown,
      left,
      right
    } side;
  };

  Parameters parameters; /**< The parameters of this module. */
  GoalPercept* percept;
  Storage<Spot, MAX_GOAL_SPOTS> spots;

  void update(GoalPercept& percept);
  void findSpots();
  void checkSpotsWidth();
  void checkGreenBelow();
  void checkFieldCoordinates();
  void checkMinimumDistance();
  void resetPercept();
  void createPercept();
  bool topYellow(int minWidth);
  ColorClasses::Color getColor(int color);

  int findBlueOrYellowRight(int x, int y, int xEnd);
  int findGreenDown(int x, int y, int yEnd);
  int runRight(int x, int y, ColorClasses::Color col, int yEnd, int maxSkip);
  int runLeft(int x, int y, ColorClasses::Color col, int yEnd, int maxSkip);
  int runDown(int x, int y, ColorClasses::Color col, int yEnd, int maxSkip);
  int runUp(int x, int y, ColorClasses::Color col, int yEnd, int maxSkip);
  inline ColorClasses::Color imageColor(int x, int y);
  int calculateExpectedPixelWidth(Vector2<int> posImg);
  Vector2<> getPointOnField(const Vector2<int> &p);
};
