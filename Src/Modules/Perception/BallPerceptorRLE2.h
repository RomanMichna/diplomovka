/**
* @file BallPerceptorRLE2.h
* This file declares a module that provides the ball percept.
* @author Alexander Härtl
*/

#ifndef BallPerceptorRLE2_H
#define BallPerceptorRLE2_H

#include "Tools/Module/Module.h"
#include "Tools/BoolArray.h"
#include "Tools/MMX.h"
#include "Tools/Debugging/DebugImages.h"
#include "Tools/Math/Matrix.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Perception/RunLengthImage.h"
#include "Representations/Perception/ImageGrid.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Perception/BallPercept.h"
#include "Representations/Configuration/ColorConfiguration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/JointData.h"

MODULE(BallPerceptorRLE2)
  REQUIRES(FieldDimensions)
  REQUIRES(RunLengthImage)
  REQUIRES(ImageGrid)
  REQUIRES(CameraMatrix)
  REQUIRES(ImageCoordinateSystem)
  REQUIRES(CameraInfo)
  REQUIRES(ColorConfiguration)
  REQUIRES(RobotDimensions)
  REQUIRES(FrameInfo)
  REQUIRES(Image) // just for debug images
  REQUIRES(FilteredJointData) // for a plot
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(BallPercept)
END_MODULE

class BallPerceptorRLE2 : public BallPerceptorRLE2Base
{
private:
  /**
  * A collection of parameters for the ball perceptor.
  */
  class Parameters : public Streamable
  {
  public:
    float additionalWidth;
    int thresholdY, thresholdCb, thresholdCr, thresholdSaturation; /**< thresholds for comparing a run with the average color of a region. */
    int maxNumberOfRuns; /**< maximum number of runs to be processed, counted from bottom of the image. */
    int sortedRuns; /**< number of runs to be sorted by their rating. */
    int promisingRuns; /**< number of runs from which a region is tried to build, best rating first. */
    int widthPenalty; /**< penalty for one pixel difference from the expected diameter of the ball in image (for the rating of runs). */
    float radiusOffset; /**< value added to the measured radius to compensate for a constant measurement error */
    float thresholdRadius; /**< threshold for the difference of the measured and the expected radius. */
    float roundnessThreshold; /**< threshold for the average deviation of border points from the fitted circle. */
    unsigned minRunsPerRegion; /**< number of runs that are necessary to be considered as a region resp. a ball. */
    bool useRANSAC; /**< whether to use the RANSAC algorithm to find the best matching circle. */
    unsigned iterationsRANSAC; /**< number of iterations used for the RANSAC algorithm. */
    float thresholdRANSAC; /**< distance of points from the model to be considered as consensus. */
    bool useMMX; /**< Whether the MMX-accelerated implementations are used. */
    bool preprocessing; /**< Whether the computation is sped up by preprocessing. */

    /** Default constructor. */
    Parameters() :
      additionalWidth(3.0f),
      thresholdY(64),
      thresholdCb(32),
      thresholdCr(32),
      thresholdSaturation(40),
      maxNumberOfRuns(1000),
      sortedRuns(100),
      promisingRuns(10),
      widthPenalty(10),
      radiusOffset(0.5f),
      thresholdRadius(4.0f),
      roundnessThreshold(2.5f),
      minRunsPerRegion(2),
      useRANSAC(false),
      iterationsRANSAC(10),
      thresholdRANSAC(2.0f),
      useMMX(true),
      preprocessing(true)
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
      STREAM(additionalWidth);
      STREAM(thresholdY);
      STREAM(thresholdCb);
      STREAM(thresholdCr);
      STREAM(thresholdSaturation);
      STREAM(maxNumberOfRuns);
      STREAM(sortedRuns);
      STREAM(promisingRuns);
      STREAM(widthPenalty);
      STREAM(radiusOffset);
      STREAM(thresholdRadius);
      STREAM(roundnessThreshold);
      STREAM(minRunsPerRegion);
      STREAM(useRANSAC);
      STREAM(iterationsRANSAC);
      STREAM(thresholdRANSAC);
      STREAM(useMMX);
      STREAM(preprocessing);
      STREAM_REGISTER_FINISH;
    }
  };

  Parameters parameters; /**< Parameters for the module. */

  /** 
  * A class representing a potential ball found in the image.
  */
  class PotentialBall
  {
  public:
    vector<const RunLengthImage::Run*> runs; /**< the set of runs the region in the image consists of. */
    short xMin, xMax, yMin, yMax; /**< the bounding box of the region. */
    Vector2<> centerInImage;
    Vector2<> relPosOnField;
    float radius; /**< radius in pixels in the image. */
    float radiusBasedDistance;
    Image::Pixel avgColor;
    bool valid;
  };

  /**
  * A class used to sort the runs in the image by a rating function.
  */
  class ExtendedRun
  {
  public:
    unsigned short colorDistance; /**< the rating of the associated run, less is better. */
    unsigned short runIndex; /**< the index of the associated run relative to the starting run. */

    /** comparison operator to sort runs descending by their rating.
    * @param other The run this run is compared with.
    * @return Whether this run has a better weighting than the other run.
    */
    inline bool operator<(const ExtendedRun& other) const
    {
      return colorDistance < other.colorDistance;
    }
  };

  /**
  * A class representing a single border point of the ball in the image.
  */
  class BallPoint : public Vector2<int>
  {
  public:
    bool isCornerPoint; /**< Whether this point is a corner of the image, to exclude it from the convex hull */
    BallPoint(const Vector2<int>& point, bool isCornerPoint) : Vector2<int>(point), isCornerPoint(isCornerPoint) {}
    BallPoint() : Vector2<int>(), isCornerPoint(false) {}
  };

  /**
  * The main method of the module. Tries to find a ball in the image.
  * @param ballPercept The BallPercept where the found ball is written to.
  */
  void update(BallPercept& ballPercept);

  /**
  * Initializes the module.
  */
  void init();

  /**
  * Does some stuff that has to be done prior to the actual processing.
  */
  void preExecution();

  /**
  * Processes each run in the RunLengthImage and computes the sum of color differences
  * of the three color channels. The result is written into an array of ExtendedRuns.
  *
  * @param orange The color prototype for orange represented as a 32-bit value.
  * @param mask The bit mask to mask out the padding y-value.
  * @param pSrc Pointer to the RunLengthImage.
  * @param pDst Pointer to an array of ExtendedRuns, where the result is written to.
  * @param numberOfRuns Guess what?!
  */
  void buildColorDifferences(Image::Pixel orange, const RunLengthImage::Run* pSrc, ExtendedRun* pDst, int numberOfRuns) const;

  /**
  * Processes each run in the RunLengthImage and computes the sum of color differences
  * of the three color channels using MMX-intrinsics. The result is written into an array of ExtendedRuns.
  *
  * @param orange The color prototype for orange.
  * @param pSrc Pointer to the RunLengthImage.
  * @param pDst Pointer to an array of ExtendedRuns, where the result is written to.
  * @param numberOfRuns Guess what?!
  */
  void buildColorDifferencesMMXIntrinsics(Image::Pixel orange, const RunLengthImage::Run* pSrc, ExtendedRun* pDst, int numberOfRuns) const;

  /**
  * Processes each run in the RunLengthImage and computes the difference of the run length
  * to the expected diameter of the ball in the image and adds it to the rating in the ExtendedRuns.
  *
  * @param runs Pointer to the RunLengthImage.
  * @param extendedRuns Pointer to an array of ExtendedRuns, where the rating is updated.
  * @param numberOfRuns Guess what?!
  */
  void buildWidthDifferences(const RunLengthImage::Run* runs, ExtendedRun* extendedRuns, const int numberOfRuns) const;

  /**
  * Processes each run in the RunLengthImage and computes the difference of the run length
  * to the expected diameter of the ball in the image and adds it to the rating in the ExtendedRuns.
  * The expected width is "cached" for each line.
  *
  * @param runs Pointer to the RunLengthImage.
  * @param extendedRuns Pointer to an array of ExtendedRuns, where the rating is updated.
  * @param numberOfRuns Guess what?!
  */
  void buildWidthDifferences2(const RunLengthImage::Run* runs, ExtendedRun* extendedRuns, const int numberOfRuns) const;

  /**
  * Processes each run in the RunLengthImage and computes the difference of the run length
  * to the expected diameter of the ball in the image and adds it to the rating in the ExtendedRuns.
  * The implementation uses MMX-intrinsics to speed up the computation.
  *
  * @param runs Pointer to the RunLengthImage.
  * @param extendedRuns Pointer to an array of ExtendedRuns, where the rating is updated.
  * @param numberOfRuns Guess what?!
  */
  void buildWidthDifferencesMMX(const RunLengthImage::Run* runs, ExtendedRun* extendedRuns, const int numberOfRuns) const;

  
  /**
  * Builds a region starting at a given run. The region is grown as long as there are adjacent
  * runs with similar color, and as long as the bounding box of a hypothetical ball in the image
  * is not exceeded. Runs are first searched upwards and then downwards. The found potential ball
  * is put into potentialBalls.
  *
  * @param pRun Pointer to the run serving as seed for the region.
  * @param visitedRuns BoolArray holding whether a single run has been visited yet.
  * @param searchUpAndDown Whether to search only downwards (false) or upwards too (true)
  * @return Whether a valid region is found (a valid region is a region not larger than a back projected ball in the image).
  */
  bool buildCircle(const RunLengthImage::Run* pRun, BoolArray& visitedRuns, bool searchUpAndDown);

  /**
  * Builds a region starting at a given run. The region is grown as long as there are adjacent
  * runs with similar color, and as long as the bounding box of a hypothetical ball in the image
  * is not exceeded. Runs are first searched upwards and then downwards. The found potential ball
  * is put into potentialBalls. The color comparison is speeded up with MMX
  *
  * @param pRun Pointer to the run serving as seed for the region.
  * @param visitedRuns BoolArray holding whether a single run has been visited yet.
  * @param searchUpAndDown Whether to search only downwards (false) or upwards too (true)
  * @return Whether a valid region is found (a valid region is a region not larger than a back projected ball in the image).
  */
  bool buildCircleMMX(const RunLengthImage::Run* pRun, BoolArray& visitedRuns, bool searchUpAndDown);

  /**
  * Tries to enlarge the region of a potential ball to the left and right with simliarly
  * colored runs.
  *
  * @param potentialBall The potential ball that is tried to be extended.
  */
  void extendCircle(PotentialBall& potentialBall) const;

  /**
  * Matches a circle in the image to the border points of a given potential ball and
  * checks if the roundness is not too large. The circle is fitted using least squares.
  * The result is written into the given potentialBall.
  *
  * @param potentialBall The potential ball to be processed.
  * @return Whether a valid and round enough circle could be found.
  */
  bool getCenterAndRadius(PotentialBall& potentialBall) const;

  /**
  * Matches a circle in the image to the border points of a given potential ball and
  * checks if the roundness is not too large. The circle is fitted using least squares
  * using RANSAC beforehand. The result is written into the given potentialBall.
  *
  * @param potentialBall The potential ball to be processed.
  * @return Whether a valid and round enough circle could be found.
  */
  bool getCenterAndRadiusRANSAC(PotentialBall& potentialBall) const;

  /**
  * Computes the position of the ball in robot relative coordinates and writes the result
  * into the given potentialBall. The computation projects the circle center onto the ground,
  * the size based distance is ignored because it is to imprecise.
  *
  * @param potentialBall The potential ball to be processed.
  * @return Whether the ball is above the horizon and hence not on the field.
  */
  bool getRelativePosition(PotentialBall& potentialBall) const;

  /**
  * Checks whether the measured radius in image approximately matches the radius
  * of the back projection based on the center in image. The computation considers
  * the fact that the border in the image is formed by tangential rays, and the
  * projective distortion in the image.
  *
  * @param potentialBall The potential ball to be checked.
  * @return Whether the measured radius approximately matches the back projection.
  */
  bool checkBackProjection(PotentialBall& potentialBall) const;

  /**
  * Approximates the width in image at a certain y-coordinate of
  * the ball diameter on the ground.
  *
  * @param yCoord The y-coordinate to which the length should be projected.
  * @return The projected length in the image.
  */
  float getWidthInImage(float yCoord) const;

  /**
  * Draws some debug drawings.
  */
  void draw() const;

  void drawRatingBallRuns(const ExtendedRun* const runs, const RunLengthImage::Run* startRun, const int numberOfRuns) const;
  void drawnormalizedImages() const;

  int startCoord; /**< The y-coordinate of the starting distance in the image. */
  float widthSlope, widthIntercept; /**< Precomputed values to compute the ball diameter in image. (widthInImage = y * widthSlope + widthIntercept) */

  float cameraHeight; /**< The height of the camera above the plane of the ball center. */

  list<PotentialBall> potentialBalls;

  DECLARE_DEBUG_IMAGE(ratingBallRuns);
  DECLARE_DEBUG_IMAGE(normalizedYColors);
  DECLARE_DEBUG_IMAGE(normalizedYRuns);
  DECLARE_DEBUG_IMAGE(normalizedColors);
  DECLARE_DEBUG_IMAGE(normalizedRuns);
  DECLARE_DEBUG_IMAGE(normalizedHue);
  DECLARE_DEBUG_IMAGE(saturation);
  DECLARE_DEBUG_IMAGE(brightness);
  DECLARE_DEBUG_IMAGE(ballRegions);

  // some members used for back projection
  Matrix<3, 3> P2;
  __m64 mThreshold;
  __m64 divisionLUT[128];

  /**
  * An entry of the color-LUT containing normalized colors and saturation.
  * The LUT is accessed via colorLUT[cr * 256 + cb];
  */
  union ColorLutEntry
  {
    struct
    {
      unsigned char cb, cr; /**< Normalized colors. */
      unsigned char saturation; /**< (approximately) saturation of the color. */
      unsigned char padding; /**< unused byte to ensure sizeof(ColorLutEntry) = 4 for faster access. */
    };
    unsigned int dword;
  } colorLUT[256 * 256];
};

#endif// __BallPerceptorRLE2_h_
