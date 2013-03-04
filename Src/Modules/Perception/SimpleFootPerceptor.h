/*
 * @file FootPerceptor.cpp
 * Declares a class, that finds evidenve for robot feet(robot parts on the ground)
 * in the current image.
 * Only works for feet that are in the vicinity and not too far away.
 * @author Tobias Kastner
 * @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Perception/FootPercept.h"
#include "Representations/Perception/LineSpots.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Configuration/ColorTable64.h"
#include "Tools/Debugging/DebugDrawings.h"
#include <vector>

MODULE(SimpleFootPerceptor)
  REQUIRES(CameraMatrix)
  REQUIRES(CameraInfo)
  REQUIRES(LineSpots)
  REQUIRES(FilteredJointData)
  REQUIRES(RobotPose)
  REQUIRES(FrameInfo)
  REQUIRES(Image)
  REQUIRES(ColorTable64)
  REQUIRES(OdometryData)
  REQUIRES(OpponentTeamInfo)
  PROVIDES_WITH_MODIFY_AND_DRAW(FootPercept)
  REQUIRES(FootPercept)
  PROVIDES_WITH_MODIFY_AND_DRAW(SimpleFootModel)
END_MODULE


class SimpleFootPerceptor : public SimpleFootPerceptorBase
{
private:
  class FootPerceptorParameter : public Streamable
  {

    virtual void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN;
      STREAM(numOfNls);
      STREAM(minHeadPitchAngle);
      STREAM(maxHeadPitchAngle);
      STREAM(minNlsLength);
      STREAM(maxNlsLength);
      STREAM(stepSize);
      STREAM(count);
      STREAM(yStep);
      STREAM(runTolerance);
      STREAM(runDistance);
      STREAM(failCount);
      STREAM(bottomTolerance);
      STREAM(minAssociatedSpots);
      STREAM_REGISTER_FINISH;
    }

  public:
    /**< Contructor */
    FootPerceptorParameter() : numOfNls(4), minHeadPitchAngle(-35.f),
      maxHeadPitchAngle(0.f), minNlsLength(10), maxNlsLength(180),
      stepSize(4), count(5), yStep(5), runTolerance(20), runDistance(30),
      failCount(2), bottomTolerance(10), minAssociatedSpots(4) {};

    unsigned numOfNls; /**< min number of NonLineSpots seen to start checking for feet */
    float minHeadPitchAngle; /**< min head pitch angle to start checking for feet */
    float maxHeadPitchAngle; /**< max " ^ " */
    int minNlsLength; /**< min NonLineSpot length */
    int maxNlsLength; /**< max " ^ " */
    int stepSize; /**< stepsize for scanning on the image for the top white horizontal segments/runs */
    int count; /**< min count for steps to complete a min run */
    int yStep; /**< needed for sanity color checks */
    int runTolerance; /**< additional tolerance for runs */
    int runDistance; /**< max distance (in pixels atm) between two runs */
    int failCount; /**< number of points on the convex hull that are to low in the Image to be concerned */
    int bottomTolerance; /**< tolerance (in pixels) for convex hull points (see comment above) */
    unsigned minAssociatedSpots; /**< min number of NonLineSpots that have to be associated with a run */
  };

  /**
  * @class Run
  *
  * Represents white/none/red pixels merged into a run.
  * This class is neede to check if there might be feet in the image.
  * If there are no white runs on the first upper line in the image, ther are no feet to find.
  */
  class Run
  {
  public:
    /**< Contructor */
    Run(int start, int stop): start(start), stop(stop) {};

    int start; /**< start position of this run (pixel count) */
    int stop; /**< stop " ^ " */
    std::vector<LineSpots::NonLineSpot> associatedSpots; /**< associated NonLineSpots with this run */
  };

  FootPerceptorParameter p; /**< parameter set for this module */

  unsigned timeLastSeenPercept;    /**< timestamp when the last FootPercept was observed */
  FootPercept lastFootPercept;     /**< the last frames FootPercept */
  Pose2D lastOdometryWhenFootSeen; /**< Odometry value when the foot has been seen the last time*/
  int seenFrames;

public:
  SimpleFootPerceptor() : timeLastSeenPercept(0), seenFrames(0) {};

  void init(); /**< init stuff...load config */
  void update(FootPercept& theFootPercept); /**< update the FootPercept */
  void update(SimpleFootModel& theSimpleFootModel); /**< update the FootModel */
  void draw(const std::vector<Run>& runs, const std::vector<Vector2<> >& points); /**< draw some intersting stuff */

  ColorClasses::Color imageColor(int x, int y); /**< get color class for the given pixel */
  void calcTopWhiteRuns(std::vector<Run>& run); /**< check for runs on the top horizontal line in image */
  bool pixelAboveIsOrangeOrGreen(int x, int y); /**< check wether the pixel above (or the give config value for yTStep)
                                                    the given pixel is green or orange */
  void respectThisSpot(LineSpots::NonLineSpot spot, std::vector<Run>& runs); /**< check if the given spot should be respected for furher computations.
                                                                                 associate this NonLineSpot with a run */
  void checkRuns(std::vector<Run>& runs); /**< check sanity of runs */
  bool pointsValid(const std::vector<Vector2<> >& points); /**< check count of points in the point cloud that are to low in the image */
};
