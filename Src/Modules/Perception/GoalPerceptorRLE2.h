/**
* @file GoalPerceptorRLE2.h
* This file declares a module that provides the goal percept only using the RunLengthImage.
* @author Alexander Härtl
*/

#ifndef GoalPerceptorRLE2_H
#define GoalPerceptorRLE2_H

#include "Tools/Module/Module.h"
#include "Tools/Debugging/DebugImages.h"
#include "Tools/Range.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Perception/RunLengthImage.h"
#include "Representations/Perception/ImageGrid.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Perception/GoalPercept.h"
#include "Representations/Configuration/ColorConfiguration.h"
#include <vector>
#include "Tools/MMX.h"

MODULE(GoalPerceptorRLE2)
  REQUIRES(FieldDimensions)
  REQUIRES(RunLengthImage)
  REQUIRES(ImageGrid)
  REQUIRES(CameraMatrix)
  REQUIRES(ImageCoordinateSystem)
  REQUIRES(CameraInfo)
  REQUIRES(ColorConfiguration)
  REQUIRES(FrameInfo)
  REQUIRES(OwnTeamInfo)
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(GoalPercept)
END_MODULE


class GoalPerceptorRLE2 : public GoalPerceptorRLE2Base
{
private:
  /**
  * A collection of parameters for the ball perceptor.
  */
  class Parameters : public Streamable
  {
  public:
    int thresholdY, thresholdCb, thresholdCr; /**< Threshold for the single channels for merging similarly colored runs. */
    int houghWidth; /**< Width of the one-dimensional hough space. The higher this value, the higher the accuracy and the computation time. */
    int houghSpaceBeginAboveVanishingPoint; /**< "Dead zone" above the vanishing point. */
    int houghSpaceBaseValue; /**< The value the color difference is subtracted from to be added to the hough accumulator. */
    int houghSpaceMinValue; /**< The minimum value to increase the hough accumulator. */
    vector<int> houghSpaceConvolutionKernel; /**< Convolution kernel used for smoothing the hough space. */
    int houghSpaceMaxima; /**< Number of hough space maxima to be searched for goal posts. */
    int minHeight; /**< Minimum height of a region to be considered as a goal post. */
    int searchDepthGreenBelow; /**< Search depth below a potential goal post for green runs. */
    int searchHeightGoalPost;
    int maxSkipRuns; /**< Maximum number of non matching runs to be skipped while region building. */
    float widthToleranceFactor; /**< Maximum ratio of expected and actual goal post width. */

    /** Default constructor. */
    Parameters() :
      thresholdY(32),
      thresholdCb(22),
      thresholdCr(22),
      houghWidth(120),
      houghSpaceBeginAboveVanishingPoint(20),
      houghSpaceBaseValue(120),
      houghSpaceMinValue(12),
      houghSpaceMaxima(5),
      minHeight(30),
      searchDepthGreenBelow(5),
      searchHeightGoalPost(5),
      maxSkipRuns(2),
      widthToleranceFactor(0.5)
    {
      houghSpaceConvolutionKernel.reserve(5);
      houghSpaceConvolutionKernel.push_back(1);
      houghSpaceConvolutionKernel.push_back(4);
      houghSpaceConvolutionKernel.push_back(6);
      houghSpaceConvolutionKernel.push_back(4);
      houghSpaceConvolutionKernel.push_back(1);
    }

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
      STREAM(houghWidth);
      STREAM(houghSpaceBeginAboveVanishingPoint);
      STREAM(houghSpaceBaseValue);
      STREAM(houghSpaceMinValue);
      STREAM(houghSpaceConvolutionKernel);
      STREAM(houghSpaceMaxima);
      STREAM(minHeight);
      STREAM(searchDepthGreenBelow);
      STREAM(searchHeightGoalPost);
      STREAM(maxSkipRuns);
      STREAM(widthToleranceFactor);
      STREAM_REGISTER_FINISH;
    }
  };

  Parameters parameters; /**< Parameters for the module. */
  
  /**
  * Structure representing a hough line in image coordinates.
  */
  struct HoughLine
  {
    float xStart, slope; /**< The x-coordinate can be computed by xStart + y * slope. */
    HoughLine() {}
    HoughLine(float xStart, float slope) : xStart(xStart), slope(slope) {}
  };

  /**
  * A class representing a single region, especially a goal post.
  */
  struct Region
  {
    Image::Pixel color; /**< average color of region. */
    int count; /**< number of runs that form the region. */
    int height; /**< height of this region in pixels. */
    const RunLengthImage::Run* const * pStart; /**< pointer to the first run. */
    const RunLengthImage::Run* const * pEnd; /**< pointer behind the last run. */
  };

  /**
  * A structure representing a potential goal post.
  */
  struct PotentialGoalPost
  {
    Vector2<> footPointInImage; /**< The (not yet motion corrected) coordinates of the foot point in the image. */
    Vector2<> relPos; /**< The position of the foot point in robot relative coordinates. */
    GoalPost::Position side; /**< Whether this goal post is known to be left or right, or unknown. */
    float averageWidth; /**< The average width of the runs forming this goal post.*/
    int height; /**< The height of this goal post in the image. */
    Range<int> exclusionRange; /**< The approcimate range in the hough space covered by this goal post. */
    int maximumIndex; /**< Number of the maximum this goal post corresponds to. */

    /**
    * Comparison operator to sort goal posts by color first, then by angle in robot relative coordinates.
    * @param other The goal post to be compared with this one.
    * @return Whether this goal post is before the other one according to the named sorting scheme.
    */
    bool operator<(const PotentialGoalPost& other) const
    {
      return relPos.angle() > other.relPos.angle();
    }
  };

  DECLARE_DEBUG_IMAGE(goalPostSegmentWeighting);
  DECLARE_DEBUG_IMAGE(houghSpace);
  DECLARE_DEBUG_IMAGE(goalSegments);

  /**
  * The main method of the module. Tries to find goal posts in the image.
  * @param goalPercept The GoalPercept where the found goal post(s) is (are) written to.
  */
  void update(GoalPercept& goalPercept);

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
  * Method to smoothen the hough space using a convolution filter. The first and last values are processed regularly, assuming
  * that there is some extra space allocated for the hough space.
  *
  * @param houghSpace The raw hough space to be smoothened.
  * @param averagedHoughSpace The resulting smoothened hough space (assumed to be already allocated).
  * @param houghSpaceWidth The width of the hough space.
  * @param coefficients The convolution kernel.
  */
  void averageHoughSpaceConvolution(const unsigned int* houghSpace, unsigned int* averagedHoughSpace, int houghSpaceWidth, vector<int>& coefficients) const;

  /**
  * Finds the maxima in the hough space. After a maximum has been found, the local neighborhood
  * is cleared to get a better coverage of the hough space and to avoid finding multiple maxima
  * belonging to a single goal post.
  *
  * @param averagedHoughSpace The (smoothened) hough space from which the maxima are extracted.
  * @param maxima The array of ints the maxima are written to (assumed to be already allocated).
  * @param numOfMaxima Number of maxima to be extracted.
  * @param averagedHoughSpaceWidth The width of the hough space.
  * @param clearingWidth The window around a maximum to be cleared after having been found.
  */
  void findMaxima(unsigned int* averagedHoughSpace, int* maxima, int numOfMaxima, int averagedHoughSpaceWidth, int clearingWidth) const;

  /**
  * Extracts the runs lying on a hough line in the image from the run length image.
  *
  * @param houghLine The hough line to be processed.
  * @param runs The runs lying on the hough line.
  */
  void getAdjacentRuns(HoughLine& houghLine, vector<const RunLengthImage::Run*>& runs) const;

  /**
  * Finds regions of simliar color within a set of runs lying on a hough line.
  *
  * @param runs The runs of a hough line (assumed to be sorted by y-coordinate).
  * @return The regions of similar color found within the runs of a hough line.
  */
  vector<Region> buildRegions(const vector<const RunLengthImage::Run*>& runs) const;

  /**
  * Finds regions of simliar color within a set of runs lying on a hough line.
  * Single non-matching runs are skipped according to the parameter maxSkipRuns.
  *
  * @param runs The runs of a hough line (assumed to be sorted by y-coordinate).
  * @return The regions of similar color found within the runs of a hough line.
  */
  vector<Region> buildRegionsWithSkipping(const vector<const RunLengthImage::Run*>& runs) const;

  /**
  * Tries to find the goal bar in a region that is assumed to be a goal post by
  * looking for substantially longer runs at the uppermost part of the region.
  *
  * @param region The region in which the goal bar is searched.
  * @param houghLine The hough line on which the region was found.
  * @param expectedWidth The expected width of the uppermost part of the goal post in the image.
  * @return The side of the goal post as indicated by a found goal bar, or UNKNOWN.
  */
  GoalPost::Position findGoalBar(const Region& region, const HoughLine& houghLine, int expectedWidth) const;

  /**
  * Applies some high level sanity checks on the found set of goal posts, i.e. number
  * of goal post, sane laterality and color, etc. Additionally writes the GoalPercept.
  *
  * @param goalPosts The set of goal posts found in the image.
  * @param goalPercept The GoalPercept where the result is written to.
  */
  void sanityChecks(vector<PotentialGoalPost>& goalPosts, GoalPercept& goalPercept) const;

  /**
  * Adapter to convert a potential goal post into the format needed by the GoalPercept.
  *
  * @param goalPost The goal post to be converted.
  * @return The converted goal post in GoalPercept-format.
  */
  inline GoalPost convertGoalPost(const PotentialGoalPost& goalPost) const;

  /**
  * Approximates the width of a goal post in the image given its robot relative position
  * at a certain y-coordinate.
  *
  * @param fieldCoord The robot relative field coordinates of the goal post.
  * @param y The y-coordinate in which the goal post is to be projected into the image.
  * @return The range in image x-coordinates of the projected goal post.
  */
  Range<> getWidthOfGoalPostInImage(const Vector2<>& fieldCoords, float y) const;

  float houghLineXStartSlope, houghLineXStartIntercept,
    houghLineSlopeSlope, houghLineSlopeIntercept; /**< Precomputed values to efficiently compute the hough line given the index in the hough space. */

  /**
  * Computes the hough line belonging to a certain hough index.
  *
  * @param index The index in the hough space for which the HoughLine is to be computed.
  * @return The hough line belonging to the hough index.
  */
  inline HoughLine getHoughLine(int index);

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

public:
  GoalPerceptorRLE2() {}
};

#endif// __GoalPerceptorRLE2_h_
