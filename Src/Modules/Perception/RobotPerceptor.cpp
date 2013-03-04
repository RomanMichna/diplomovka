#include "RobotPerceptor.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Debugging/Asserts.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DrawingHelpers.h"
#include "Tools/ImageProcessing/BresenhamLineScan.h"
#include "Tools/Boundary.h"

MAKE_MODULE(RobotPerceptor, Perception)

#ifndef POINT_IS_WITHIN_IMAGE
#define POINT_IS_WITHIN_IMAGE(p, image) \
  ((p).x >= 0 && (p).x < (image).resolutionWidth \
   && (p).y >= 0 && (p).y < (image).resolutionHeight)
#endif

RobotPerceptor::RobotPerceptor()
{
  InConfigMap stream("robotPerceptor.cfg");
  ASSERT(stream.exists());
  stream >> params;
  pointExplorer.initFrame(&theImage, &theColorTable64, params.exploreStepSize,
                          params.gridStepSize, params.skipOffset, params.minSegSize);
  scanlineFeatures.reserve(5);
}

void RobotPerceptor::update(RobotPercept& robotPercept)
{
  DECLARE_DEBUG_DRAWING("module:RobotPerceptor:scanlines", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:RobotPerceptor:densitometricFeatures", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:RobotPerceptor:deviation", "drawingOnImage");
  MODIFY("parameters:RobotPerceptor", params);

  robotPercept.robots.clear();
  robotPercept.unlocalizableRobots.clear();
  scanlineFeatures.clear();

  Vector2<int> lowestPx;
  for(std::vector<TeamMarkerSpots::TeamMarkerSpot>::const_iterator i = theTeamMarkerSpots.teamMarkers.begin(), end = theTeamMarkerSpots.teamMarkers.end(); i != end; ++i)
  {
    const TeamMarkerSpots::TeamMarkerSpot& teamMarkerSpot = *i;
    if(checkTeamMarkerEnvironment(teamMarkerSpot, lowestPx))
    {
      if(lowestPx.y < theImage.resolutionHeight)
        robotPercept.robots.push_back(RobotPercept::Robot(teamMarkerSpot.centerOfGravity, lowestPx, teamMarkerSpot.standing, teamMarkerSpot.color));
      else
        robotPercept.unlocalizableRobots.push_back(RobotPercept::Robot(teamMarkerSpot.centerOfGravity, lowestPx, true, teamMarkerSpot.color));
    }
  }
}

bool RobotPerceptor::checkTeamMarkerEnvironment(const TeamMarkerSpots::TeamMarkerSpot& teamMarkerSpot, Vector2<int>& lowestPx)
{
  Vector2<int> diff = teamMarkerSpot.right - teamMarkerSpot.left;
  Vector2<int> left = Vector2<int>(teamMarkerSpot.right.x - int(.9f * float(diff.x)),
                                   teamMarkerSpot.right.y - int(.9f * float(diff.y)));
  Vector2<int> right = Vector2<int>(teamMarkerSpot.left.x + int(.9f * float(diff.x)),
                                    teamMarkerSpot.left.y + int(.9f * float(diff.y)));
  Vector2<int> perpendicular = right - left;
  perpendicular.rotateLeft();

  // scan above team marker
  Vector2<int> slLeftUp = left - perpendicular * 2;
  Vector2<int> slMiddleUp = teamMarkerSpot.centerOfGravity - perpendicular * 2;
  Vector2<int> slRightUp = right - perpendicular * 2;
  scanRobot(left, slLeftUp);
  scanRobot(teamMarkerSpot.centerOfGravity, slMiddleUp);
  scanRobot(right, slRightUp);

  // scan below team marker
  Vector2<int> slLeftDown = left + perpendicular * 3;
  Vector2<int> slRightDown = right + perpendicular * 3;
  scanRobot(left, slLeftDown);
  scanRobot(right, slRightDown);

  // sanity checks
  for(std::vector<DensitometricFeatures>::iterator d = scanlineFeatures.begin(); d != scanlineFeatures.end(); d++)
  {
    if(d->pixels < params.minPixels)
      return false;
#ifdef TARGET_ROBOT
    else if(d->variance.x < params.minYVariance)
      return false;
#else // special treatment for simulated robots
    if(SystemCall::getMode() != SystemCall::simulatedRobot && d->variance.x < params.minYVariance)
      return false;
#endif
    else if(d->variance.y > params.maxCbVariance)
      return false;
    else if(d->variance.z > params.maxCrVariance)
      return false;
  }

  // determine point to locate the robot
  const int width(perpendicular.abs());
  lowestPx = teamMarkerSpot.centerOfGravity;
  for(;;)
  {
    lowestPx.y += 2;

    if(lowestPx.y < theImage.resolutionHeight)
    {
      if(ColorClasses::green != pointExplorer.getColor(lowestPx.x, lowestPx.y))
      {
        continue;
      }
    }
    else
    {
      lowestPx.y = theImage.resolutionHeight;
      break;
    }

    const Boundary<int> areaBelowRobot(Range<int>(lowestPx.x - width, lowestPx.x + width),
                                       Range<int>(lowestPx.y, lowestPx.y + 2));
    if(pointExplorer.colorRatio(ColorClasses::green, areaBelowRobot, 2) < params.greenGroundRatio)
    {
      continue;
    }

    break;
  }

  const int pixelHeight = lowestPx.y - teamMarkerSpot.centerOfGravity.y;
  // this is too high to be a horizontal robot
  if(!teamMarkerSpot.standing && pixelHeight > 100)
    return false; // TODO params
  // small high team markers are not relevant
  if(teamMarkerSpot.area < 100 && pixelHeight > 100)
    return false; // TODO params

  // TODO compare area / height and scanlines to the ground / height

  COMPLEX_DRAWING("module:RobotPerceptor:scanlines",
  {
    LINE("module:RobotPerceptor:scanlines", teamMarkerSpot.left.x, teamMarkerSpot.left.y, right.x, right.y,
    2, Drawings::ps_solid, teamMarkerSpot.color);
    LINE("module:RobotPerceptor:scanlines", slLeftDown.x, slLeftDown.y, slLeftUp.x, slLeftUp.y,
    2, Drawings::ps_solid, ColorClasses::white);
    LINE("module:RobotPerceptor:scanlines", teamMarkerSpot.centerOfGravity.x, teamMarkerSpot.centerOfGravity.y, slMiddleUp.x, slMiddleUp.y,
    2, Drawings::ps_solid, ColorClasses::white);
    LINE("module:RobotPerceptor:scanlines", slRightDown.x, slRightDown.y, slRightUp.x, slRightUp.y,
    2, Drawings::ps_solid, ColorClasses::white);
  });

  return true;
}

bool RobotPerceptor::scanRobot(const Vector2<int>& start, const Vector2<int>& end)
{
  if(!POINT_IS_WITHIN_IMAGE(start, theImage))
    return false;
  ASSERT_POINT_WITHIN_IMAGE(start, theImage);

  BresenhamLineScan scan(start, end);
  scan.init();
  Vector2<int> current = start;
  int i = 0;

  for(; i < scan.numberOfPixels; i += 2)
  {
    scan.getNext(current);
    scan.getNext(current);
    if(!POINT_IS_WITHIN_IMAGE(current, theImage))
      return false;

    if(ColorClasses::white == pointExplorer.getColor(current.x, current.y))
    {
      CROSS("module:RobotPerceptor:detailedScanlines", current.x, current.y,
            1, 1, Drawings::ps_solid, ColorClasses::black);
      break;
    }
  }

  Vector2<int> whiteStart = current;
  DensitometricFeatures df;

  for(; i < scan.numberOfPixels; i++)
  {
    scan.getNext(current);
    if(!POINT_IS_WITHIN_IMAGE(current, theImage))
      return true;
    ASSERT_POINT_WITHIN_IMAGE(current, theImage);

    const Image::Pixel& p = theImage.image[current.y][current.x];
    Vector3<int> color((int) p.y, (int) p.cb, (int) p.cr);
    df.accumulate(color);
  }

  Vector2<int> whiteEnd = current;
  df.finish();
  scanlineFeatures.push_back(df);

  COMPLEX_DRAWING("module:RobotPerceptor:deviation",
  {
    const float stdDeviation2 = 2.0f * sqrt((float)(df.variance.y + df.variance.z));
    Vector2<int> current = start;
    BresenhamLineScan scan(whiteStart, whiteEnd);
    scan.init();
    for(int i = 0; i < scan.numberOfPixels; i++)
    {
      const Image::Pixel& p = theImage.image[current.y][current.x];
      const float deviation = sqrt((float)((p.cb - df.mean.y) * (p.cb - df.mean.y) + (p.cr - df.mean.z) * (p.cr - df.mean.z)));
      ColorRGBA color = heat(deviation, stdDeviation2);
      DOT("module:RobotPerceptor:deviation", current.x, current.y, color, color);
      scan.getNext(current);
    }
  });

  COMPLEX_DRAWING("module:RobotPerceptor:densitometricFeatures",
  {
    static unsigned char drawingCounter = 0;
    int x = 0, y = 0;
    int fontSize = 8;
    ColorClasses::Color color = ColorClasses::white;

    switch(drawingCounter)
    {
    case 0:
    {
      x = start.x - 100;
      y = start.y - 75;
    }
    break;
    case 1:
    {
      x = start.x;
      y = start.y - 125;
    }
    break;
    case 2:
    {
      x = start.x + 100;
      y = start.y - 75;
    }
    break;
    case 3:
    {
      x = start.x - 100;
      y = start.y + 75;
    }
    break;
    default:
    {
      x = start.x + 100;
      y = start.y + 75;
    }
    break;
    }

    if(++drawingCounter == 5)
      drawingCounter = 0;

    DRAWTEXT("module:RobotPerceptor:densitometricFeatures",
             x, y + 10, fontSize, color, "  y : " << df.mean.x << ", " << df.variance.x);

    DRAWTEXT("module:RobotPerceptor:densitometricFeatures",
             x, y + 20, fontSize, color, " cb : " << df.mean.y << ", " << df.variance.y);

    DRAWTEXT("module:RobotPerceptor:densitometricFeatures",
             x, y + 30, fontSize, color, " cr : " << df.mean.z << ", " << df.variance.z);
  });

  return true;
}

