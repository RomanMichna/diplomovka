/**
* @file TeamMarkerPerceptor.h
* @author <a href="mailto:afabisch@tzi.de">Alexander Fabisch</a>
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/ColorTable64.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Perception/RegionPercept.h"
#include "Representations/Perception/TeamMarkerSpots.h"
#include "Tools/Math/Vector2.h"

MODULE(TeamMarkerPerceptor)
  REQUIRES(ColorTable64)
  REQUIRES(Image)
  REQUIRES(RegionPercept)
  PROVIDES_WITH_MODIFY_AND_DRAW(TeamMarkerSpots)
END_MODULE

class TeamMarkerPerceptor : public TeamMarkerPerceptorBase
{
  typedef std::pair<Vector2<int>, Vector2<int> > ScanlineSegment;
  typedef std::vector<Vector2<int> > Polygon;
  typedef Polygon::iterator Vertex;
  typedef Polygon::reverse_iterator VertexR;

  /**
   * @class Parameters
   * The parameters for the TeamMarkerPerceptor.
   */
  class Parameters : public Streamable
  {
  private:
    void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN;
      STREAM(minRegionSize);
      STREAM(maxHeight);
      STREAM(teamMarkerCrDiff);
      STREAM(teamMarkerCbDiff);
      STREAM(clippingWidthScale);
      STREAM(clippingHeightScale);
      STREAM(minTeamMarkerArea);
      STREAM(sideLengthRatio);
      STREAM(maxOutsideImageRatio);
      STREAM(maxYOnScanline);
      STREAM(maxDuration);
      STREAM_REGISTER_FINISH;
    }
  public:
    /** Minimal size of the regions provided by the Regionizer. */
    int minRegionSize;
    /** Maximal hight of the regions provided by the Regionizer. */
    int maxHeight;
    /** Maximal cr difference between the start pixel and another pixel on the scanline for region growing. */
    int teamMarkerCrDiff;
    /** Maximal cb difference between the start pixel and another pixel on the scanline for region growing. */
    int teamMarkerCbDiff;
    /** The scanlines must not exceed a certain width. */
    float clippingWidthScale;
    /** The scanlines must not exceed a certain height. */
    float clippingHeightScale;
    /** The minimal team marker area on the image. */
    int minTeamMarkerArea;
    /** The boundaries of the ratio width / height. */
    Vector2<> sideLengthRatio;
    /** Maximal ratio of pixels on scanlines outside of the image. */
    float maxOutsideImageRatio;
    /** Maximal value of the y channel on scanline pixels. */
    int maxYOnScanline;
    /** Maximal duration in milliseconds for processing the team markers. */
    unsigned maxDuration;
  };

  /** Parameters that can be changed during execution via simulator. */
  Parameters params;
  /** Horizontal distance between vertical scanlines. */
  static const int scanlineDiff = 4;

  void init();

  void update(TeamMarkerSpots& teamMarkerSpots);

  void extractTeamMarkers(TeamMarkerSpots& teamMarkerSpots);

  /** Performs some sanity checks on the base regions extracted from the RegionPercept. */
  bool regionCheck(const RegionPercept::Region* region);
  float regionLength;     /**< The length of the base region. */
  float regionHeight;     /**< The height of the base region. */
  Vector2<int> approxCog; /**< The center of gravity of the base region. */

  /** Grows the base region based on cb and cr distances. */
  bool regionGrowing(TeamMarkerSpots::TeamMarkerSpot& teamMarker);
  Polygon above, below;   /**< The end points of the grown team marker region. */
  int cbStart;            /**< The cb value used for distance measurements. */
  int crStart;            /**< The cr value used for distance measurements. */
  float width;            /**< The width of the grown region. */
  void scanDistance(Vector2<int> dir, const Vector2<int>& start, Vector2<int>& current, Polygon* p = 0);

  /** Checks whether the size of the region is within the allowed boundaries. */
  bool scanlineRegionArea(TeamMarkerSpots::TeamMarkerSpot& teamMarker);
  std::vector<ScanlineSegment> scanlineSegments;

  /** Calculates the orientation of the team marker based on the moments of the region. */
  bool scanlineRegionPrincipalAxis(TeamMarkerSpots::TeamMarkerSpot& teamMarker);
  float orientation;      /**< The orientation of the team marker. */

  /** This was previously used for more sanity checks but now it just provides debug drawings. */
  void polygonDebugDrawing(TeamMarkerSpots::TeamMarkerSpot& teamMarker);
  Polygon polygon;        /**< The polygon that describes the team marker region. */
};
