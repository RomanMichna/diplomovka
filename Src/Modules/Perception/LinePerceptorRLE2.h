/**
* @file LinePerceptorRLE2.h
* This file declares a module that provides the line percept only using the RunLengthImage.
* @author Alexander Härtl
*/

#ifndef LinePerceptorRLE2_H
#define LinePerceptorRLE2_H

#include "Tools/Module/Module.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Perception/RunLengthImage.h"
#include "Representations/Perception/ImageGrid.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Perception/LinePercept.h"
#include "Representations/Configuration/ColorConfiguration.h"
#include "Tools/MMX.h"

MODULE(LinePerceptorRLE2)
  REQUIRES(FieldDimensions)
  REQUIRES(RunLengthImage)
  REQUIRES(ImageGrid)
  REQUIRES(CameraMatrix)
  REQUIRES(ImageCoordinateSystem)
  REQUIRES(CameraInfo)
  REQUIRES(ColorConfiguration)
  REQUIRES(FrameInfo)
  PROVIDES_WITH_MODIFY_AND_OUTPUT(LinePercept)
END_MODULE


class LinePerceptorRLE2 : public LinePerceptorRLE2Base
{
private:
  /**
  * A collection of parameters for the ball perceptor.
  */
  class Parameters : public Streamable
  {
  public:
    int thresholdY, thresholdCb, thresholdCr; /**< color threshold for merging runs. */
    float maxWidthRatioDifference; /**< Maximum deviation of the ratio of two adjacent runs from 1.0 to be merged. */
    int minRunsInRegion; /**< Minimum number of adjacent runs to be considered as a region. */
    int searchDepthLeftRight; /**< Search depth for green next to runs. */
    float toleranceWidth, toleranceWidthFactor; /**< Absolute and relative tolerance for the width (intercept in linear regression) */
    float toleranceWidthSlope, toleranceWidthSlopeFactor; /**< Absolute and relative tolerance for the width slope (in linear regression) */
    float thresholdVerticalLine; /**< Threshold for the slope of a region in the image to be considered as possibly belonging to a robot. */
    float thresholdMergingD, thresholdMergingAlpha; /**< Thresholds in hesse normal form for merging runs. */
    float thresholdMergingY; /**< Threshold for maximum y-difference of end points of runs to be merged. */
    float minMergingCoverage; /**< Minimum fraction of single runs to be merged compared to the combined line. */
    float thresholdIntersectionAngle; /**< Maximum difference from right angle between to runs to be considered as an intersection. */
    float minOverlapIntersection; /**< Minimum overlap of one line beyond another for detecting intersections. */
    float maxDistanceIntersection; /**< Maximum distance of one line to another for detecting intersections. */
    float thresholdLengthCircleSegment; /**< Maximum length of a segment to be considered belonging to a circle, in field coordinates. */
    float thresholdLengthInImage; /**< minimum length of short segments (in field coordinates) in the image to be considered valid. */
    float minAngleDiffCircleSegment; /**< Minimum difference of segments to be considered as belonging to the center circle*/
    float maxDistCircleCenters; /**< Maximum distance of estimated circle centers from segments to be clustered to a circle. */
    float circleRadiusThreshold; /**< Maximum difference of the estimated circle radius from the expected radius (in mm). */
    float maxCircleVariance; /**< Maximum allowed variance in circle fitting. */
    int minCircleSegments; /**< Minimum number of segments that (possibly) form a circle. */
    float straightPenaltyFactor; /**< Factor the tolerances are multiplied by if a segment is possibly a robot. */
    int horizontalSegmentLength; /**< Minimum length of a run to be considered horizontal for special treatment. */
    bool buildRegionsWithColorSplitting; /**< *Whether the color is considered for splitting regions during region building */
    bool circleRefinement; /**< Whether the center circle is refined using least squares fitting. */

    /** Default constructor. */
    Parameters() :
      thresholdY(24),
      thresholdCb(16),
      thresholdCr(16),
      maxWidthRatioDifference(0.4f),
      minRunsInRegion(3),
      searchDepthLeftRight(6),
      toleranceWidth(2.0f),
      toleranceWidthFactor(0.6f),
      toleranceWidthSlope(0.1f),
      toleranceWidthSlopeFactor(0.08f),
      thresholdVerticalLine(0.7f),
      thresholdMergingD(100.0f),
      thresholdMergingAlpha(0.15f),
      thresholdMergingY(15.0f),
      minMergingCoverage(0.6f),
      thresholdIntersectionAngle(0.15f),
      minOverlapIntersection(5.0f),
      maxDistanceIntersection(20.0f),
      thresholdLengthCircleSegment(500.0f),
      thresholdLengthInImage(40.0f),
      minAngleDiffCircleSegment(0.2f),
      maxDistCircleCenters(150.0f),
      circleRadiusThreshold(150.0f),
      maxCircleVariance(900.0f),
      minCircleSegments(3),
      straightPenaltyFactor(0.7f),
      horizontalSegmentLength(120),
      buildRegionsWithColorSplitting(true),
      circleRefinement(true)
      {}

  private:
    /**
    * The method makes the object streamable.
    * @param in The stream from which the object is read.
    * @param out The stream to which the object is written.
    */
    virtual void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN;
      STREAM(thresholdY);
      STREAM(thresholdCb);
      STREAM(thresholdCr);
      STREAM(maxWidthRatioDifference);
      STREAM(minRunsInRegion);
      STREAM(searchDepthLeftRight);
      STREAM(toleranceWidth);
      STREAM(toleranceWidthFactor);
      STREAM(toleranceWidthSlope);
      STREAM(toleranceWidthSlopeFactor);
      STREAM(thresholdVerticalLine);
      STREAM(thresholdMergingD);
      STREAM(thresholdMergingAlpha);
      STREAM(thresholdMergingY);
      STREAM(minMergingCoverage);
      STREAM(thresholdIntersectionAngle);
      STREAM(minOverlapIntersection);
      STREAM(maxDistanceIntersection);
      STREAM(thresholdLengthCircleSegment);
      STREAM(thresholdLengthInImage);
      STREAM(minAngleDiffCircleSegment);
      STREAM(maxDistCircleCenters);
      STREAM(circleRadiusThreshold);
      STREAM(maxCircleVariance);
      STREAM(minCircleSegments);
      STREAM(straightPenaltyFactor);
      STREAM(horizontalSegmentLength);
      STREAM(buildRegionsWithColorSplitting);
      STREAM(circleRefinement);
      STREAM_REGISTER_FINISH;
    }
  } parameters; /**< Parameters for the module. */

  /**
  * A structure to represent adjacencies of runs.
  */
  struct Run
  {
    const RunLengthImage::Run* run; /**< The run this run is adjacent with, or NULL if there is none. */
    bool visited; /**< Whether this run has already been processed. */
    Run(const RunLengthImage::Run* run) : run(run), visited(false) {}
  };

  /**
  * A class representing a region / segment of a line.
  */
  struct Region
  {
    const RunLengthImage::Run* run; /**< The first run of this region. */
    short numberOfRuns; /**< Number of runs forming this region. */
    bool valid; /**< Whether this region is still valid. */
    bool shortSegment; /**< Whether this region is too short to be a regular line. */
    enum Type
    {
      Unknown,
      PossibleRobot,
      MatchingWidth,
      Circle,
      Straight,
      StraightPossibleRobot,
    } type; /**< The assumed type of this region. The higher the value, the more certain we are that this is a regular line. */
    Vector2<> startInImage, endInImage, startOnField, endOnField; /**< As the name indicates. */
    Vector2<> circleCenters[2]; /**< The two hypothetical circle center points in field coordinates if this region is considered as a circle segment. */
    float alpha, d; /**< Hesse normal form in field coordinates. */
    Vector2<> dir0; /**< normalized direction vector. */
    Region() : valid(true), type(Unknown) {}
  };

  static const ColorClasses::Color type2color[]; /**< Assignment of colors to types (only used for debug drawings). */

  /**
  * The main method of the module. Tries to find lines and the center circle in the image.
  * @param linePercept The LinePercept where the found lines are written to.
  */
  void update(LinePercept& linePercept);

  /**
  * Initializes the module.
  */
  void init();

  /**
  * Does some stuff that has to be done prior to the actual processing.
  */
  void preExecution();

  /**
  * Draws some debug drawings.
  */
  void draw() const;

  /**
  * Searches the run length image for adjacent runs that have a similar color and
  * whose ratio of run lengths does not exceed a ceratin threshold, which is the pre-
  * requisite for belonging to a line. The result is written to the member adjacency.

  * @param pStart The first run in the run length image to be processed.
  */
  void computeAdjacency(const RunLengthImage::Run* const pStart);

  /**
  * Processed the adjacent regions found prior and builds regions from them. The
  * shape of the region as well as the color gradient is ignored here. Only regions
  * whose average color is white enough are added to the result.
  *
  * @param regions The result vector the found regions are added to.
  */
  void buildRegions(vector<Region>& regions);

  /**
  * Processed the adjacent regions found prior and builds regions from them. The
  * shape of the region is ignored, whereas too hard color changes cause a splitting
  * of the region. Only regions whose average color is white enough are added to the result.
  *
  * @param regions The result vector the found regions are added to.
  */
  void buildRegionsWithColorSplitting(vector<Region>& regions);

  /**
  * Checks the shape of the regions and splits and invalidates them accordingly.
  * A region is supposed to be shaped like a trapezoid, the expected form of which can be
  * determined via the known camera pose.
  *
  * @param regions The regions to be processed.
  */
  void checkShape(vector<Region>& regions);

  /**
  * Checks whether regions are surrounded by sufficient green and
  * invalidates them accordingly.
  *
  * @param regions The regions to be processed.
  */
  void checkGreen(vector<Region>& regions);

  /**
  * Scans the run length image for long white runs potentially forming a horizontal line, 
  * which are naturally exlucded from the region building done prior.
  *
  * @param regions The result vector the found regions are added to.
  */
  void findHorizontalSegments(vector<Region>& regions);

  /**
  * Tries to find the center circle by clustering single segments. The
  * clustering is done using the Union Find algorithm. As a first
  * stage of processing, the points normal to the segments in a distance
  * of center circle radius are clustered, based on this approximation all
  * segments potentially belonging to a circle are found and the circle is
  * approcimated using least squares fitting (in image coordinates).
  *
  * @param regions The regions to be processed.
  * @param linePercept The LinePercept, the result is written to.
  */
  void findCircle(vector<Region>& regions, LinePercept& linePercept) const;

  /**
  * Projects the found segments onto the field applying motion compensation.
  * Simultaneously the hesse normal form, the normalized direction vector and
  * the potential circle centers are computed.
  *
  * @param regions The regions to be processed.
  */
  void projectOntoField(vector<Region>& regions) const;

  /**
  * Merges regions that belong to a single line. Candidates are identified
  * by comparing the hesse normal form and clustered with the Union Find
  * algorithm. Merged segments that do not cover a sufficiently large amount
  * of the resulting lines, as well as segments that are too short to be a
  * line are discarded.
  *
  * @param regions The regions to be processed.
  */
  void mergeRegions(vector<Region>& regions) const;

  /**
  * Finds intersections within the lines as well as the center circle with lines
  * and writes them to the LinePercept.
  *
  * @param regions The regions to be processed.
  * @param linePercept The LinePercept, the result is written to.
  */
  void findIntersections(vector<Region>& regions, LinePercept& linePercept) const;

  /**
  * Transforms the internal format of regions / segments into the format accepted 
  * by the LinePercept and writes the result into the LinePercept.
  *
  * @param regions The regions to be processed.
  * @param linePercept The LinePercept, the result is written to.
  */
  void createLines(vector<Region>& regions, LinePercept& linePercept) const;

  /** 
   * The method computes the vanishing point of a certain
   * axis in image coordinates.
   *
   * @param The axis in world coordinates the vanishing point of which to be computed
   * @return The vanishing point in the image
   */
  Vector2<> computeVanishingPoint(int axis) const;

  __m64 mThreshold; /**< The color merging threshold as packed unsigned shorts. */
  __m64 divisionLUT[cameraResolutionHeight]; /**< LUT containing packed reciprocals shifted by 16 bits used to compute the average colors. */

  /**
  * A class implementing a linear regression model that can be successively fed with input data.
  * It approximates the trend of the centers as well as the widths of runs with a linear model.
  * To save computation time, the parameters are not updated after every input, but in exponentially
  * growing intervals.
  */
  class SuccessiveLinearRegressor
  {
  private:
    float sumX, sumXX, sumYWidth, sumYCenter, sumXYWidth, sumXYCenter; /**< Internal accumulators. */
    int count; /**< Number of values in the accumulators. */
    int nextParameterUpdate; /**< The count when the next parameter update is triggered. */

    inline void computeModelParameters(); /**< Computes the model parameters from the internal accumulators (the actual regression). */

    SuccessiveLinearRegressor(); /**< No default constructor. */

  public:
    float aWidth, aCenter, bWidth, bCenter; /**< The model parameters. */

    /**
    * Constructor taking the initial run to initialize the accumulators.
    *
    * @param run The first run to be analyzed.
    */
    SuccessiveLinearRegressor(const RunLengthImage::Run& run);

    /**
    * Integrates a single run ("measurement") into the model. The
    * model parameters are not necessarily updated (see above).
    *
    * @param run The run to be integrated into the model.
    */
    inline void addMeasurement(const RunLengthImage::Run& run);

    /**
    * Applies the approximated model to a certain value (a y-coordinate).
    *
    * @param x The y-coordinate to which the model is applied.
    * @param width The predicted width.
    * @param center The predicted center.
    */
    inline void predictValues(const int x, int& width, int& center) const;
    inline float getWidthSlope() const { return bWidth; }
    inline float getWidthIntercept() const { return aWidth; }
    inline float getCenterSlope() const { return bCenter * 0.5f; }
    inline float getCenterIntercept() const { return aCenter * 0.5f; }
    inline float getAvgX() const { return sumX / (float)count; }
  };

  /**
  * A class used to cluster potential circle regions.
  */
  struct CircleRegion
  {
    Region* region; /**< The region the potential circle center originates from. */
    CircleRegion* parent; /**< The parent region, or (by convention) this if there is no parent. */
    Vector2<>* circleCenter; /**< Pointer to the circle center represented by this wrapper. */
    short regionNumber; /**< Unique number of the cluster this region belongs to. */
    bool match; /**< After preprocessing whether the region matches the approximated circle center. */

    CircleRegion() : match(false) {}
  };

  /**
  * A class used to merge regions using Union Find.
  */
  struct MergingRegion
  {
    Region* region; /**< The region wrapped. */
    MergingRegion* parent;  /**< The parent region, or (by convention) this if there is no parent. */
    int regionNumber; /**< Unique number of the cluster this region belongs to. */
  };

  /**
  * Function template to compress the path p is part of, i.e.
  * lets all elements of the path point to the root.
  * Assumes that there is a member 'parent', and that the
  * root region points to itself.
  *
  * @param p The element of th path to be compressed
  */
  template <typename T> static void pathCompress(T* p)
  {
    T* root = p;
    while(root->parent != root) root = root->parent;
    while(p != root)
    {
      T* temp = p->parent;
      p->parent = root;
      p = temp;
    }
  }

  /**
  * Implements the Union Find algorithm to efficiently unite two elements.
  */
  template <typename T> static void unite(T* p1, T* p2)
  {
    // first compress paths for faster access in subsequent calls
    pathCompress(p1);
    pathCompress(p2);
    // always point to earlier root
    if(p1->parent < p2->parent) p2->parent->parent = p1->parent;
    else p1->parent->parent = p2->parent;
  }

  Run* adjacency; /**< Contains information on adjacent runs of similar length and color. */
  const RunLengthImage::Run* pStart; /**< The first run to be processed. */
  int numberOfRuns; /**< The number of runs processed. */

public:
  LinePerceptorRLE2();
  ~LinePerceptorRLE2();
};

#endif// __LinePerceptorRLE2_h_
