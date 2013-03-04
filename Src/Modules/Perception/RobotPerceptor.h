/**
* @file RobotPerceptor.h
* @author <a href="mailto:afabisch@tzi.de">Alexander Fabisch</a>
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Configuration/ColorTable64.h"
#include "Representations/Perception/TeamMarkerSpots.h"
#include "Representations/Perception/RobotPercept.h"
#include "PointExplorer.h"
#include "Tools/Math/Vector3.h"
#include <vector>

MODULE(RobotPerceptor)
  REQUIRES(Image)
  REQUIRES(ColorTable64)
  REQUIRES(TeamMarkerSpots)
  PROVIDES_WITH_MODIFY_AND_DRAW(RobotPercept)
END_MODULE

/**
 * @class RobotPerceptor
 * This class finds indicators for robots on the image.
 */
class RobotPerceptor : public RobotPerceptorBase
{
  /**
   * @class Parameters
   * The parameters of the RobotPerceptor.
   */
  class Parameters : public Streamable
  {
  private:
    /** Streams the parameters. */
    void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN;
      STREAM(minPixels);
      STREAM(minYVariance);
      STREAM(maxCbVariance);
      STREAM(maxCrVariance);
      STREAM(greenGroundRatio);
      STREAM(exploreStepSize);
      STREAM(gridStepSize);
      STREAM(skipOffset);
      STREAM(minSegSize);
      STREAM_REGISTER_FINISH;
    }
  public:
    /** Thresholds for the robot's body area. */
    Vector2<int> threshold;
    /** The ratio of white pixels to total pixels above the team marker. */
    float aboveWhiteRatio;
    /** The ratio of white pixels to total pixels below the team marker. */
    float belowWhiteRatio;
    /** Minimal number of pixels on each scanline. */
    int minPixels;
    /** Minimal variance of y values on each scanline. */
    int minYVariance;
    /** Maximal variance of cb values on each scanline. */
    int maxCbVariance;
    /** Maximal variance of cr values on each scanline. */
    int maxCrVariance;
    /** The ratio of green pixels in the area below the robot. */
    float greenGroundRatio;
    /** The distance in pixels between exploring scanlines. */
    int exploreStepSize;
    /** The distance in pixels between neighboring scan lines. */
    int gridStepSize;
    /** The maximum number of pixels to skip when grouping pixels to segments. */
    int skipOffset;
    /** The minimal size/length for a segment in pixels for each color. */
    int minSegSize[ColorClasses::numOfColors];
  };

  class DensitometricFeatures
  {
    Vector3<int> accumulator2;
  public:
    DensitometricFeatures() : pixels(0) {}
    unsigned char pixels;
    Vector3<int> mean;
    Vector3<int> variance;

    void accumulate(const Vector3<int>& value)
    {
      pixels++;
      mean += value;
      accumulator2.x += value.x * value.x;
      accumulator2.y += value.y * value.y;
      accumulator2.z += value.z * value.z;
    }

    void finish()
    {
      if(pixels > 0)
      {
        const int besselsCorrection(pixels - 1);
        const int normalization(pixels * besselsCorrection);
        variance = accumulator2 / besselsCorrection - Vector3<int>(mean.x * mean.x, mean.y * mean.y, mean.z * mean.z) / normalization;
        mean /= (int) pixels;
      }
    }
  };

  /** The parameters of the RobotPerceptor. */
  Parameters params;
  /** The PointExplorer used in this module. */
  PointExplorer pointExplorer; // TODO remove pointExplorer

  /** Updates the RobotPercept. */
  void update(RobotPercept& robotPercept);

  /**
   * Checks several assumptions about the Environment of the team marker.
   * @param teamMarkerSpot The team marker to be checked.
   * @param lowestPx The lowest pixel of the robot will be set if the check is successful.
   * @return Is this a robot?
   */
  bool checkTeamMarkerEnvironment(const TeamMarkerSpots::TeamMarkerSpot& teamMarkerSpot, Vector2<int>& lowestPx);

  /**
   * Scans the robot's body and generates densitometric features.
   * @param start The start of the scanline.
   * @param end The end of the scanline.
   * @return Pixels scanned?
   */
  bool scanRobot(const Vector2<int>& start, const Vector2<int>& end);

  std::vector<DensitometricFeatures> scanlineFeatures; /**< Statistical information about the pixels on the scanlines. */

public:
  /** Initializes the parameters. */
  RobotPerceptor();
};
