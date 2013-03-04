/**
* @file RobotPoseUnscentedFilter.h
* Declaration of module RobotPoseUnscentedFilter
* @author Colin Graf
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Perception/LinePercept.h"
#include "Representations/Perception/GoalPercept.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Tools/Math/Matrix.h"

MODULE(RobotPoseUnscentedFilter)
  REQUIRES(PotentialRobotPose)
  REQUIRES(CameraMatrix)
  REQUIRES(FieldDimensions)
  REQUIRES(CameraInfo)
  REQUIRES(LinePercept)
  REQUIRES(MotionInfo)
  REQUIRES(OdometryData)
  REQUIRES(GoalPercept)
  REQUIRES(GroundContactState)
  REQUIRES(FallDownState)
  REQUIRES(FrameInfo)
  PROVIDES_WITH_MODIFY(FilteredRobotPose)
END_MODULE

/**
* @class RobotPoseUnscentedFilter
* A modules that refines the PotentialRobotPose using LinePercept and GoalPercept.
*/
class RobotPoseUnscentedFilter : public RobotPoseUnscentedFilterBase
{
public:
  /**
  * Default constructor.
  */
  RobotPoseUnscentedFilter();

private:
  /**
  * A collection of parameters for the robot pose validator.
  */
  class Parameters : public Streamable
  {
  public:
    /**
    * Default constructor.
    */
    Parameters() {}

    Pose2D respawnMaxPoseDistance; /**< The maximal admissible distance between the estimate from the self locator and the pose from the validator. */
    Pose2D respawnPoseDeviation; /**< The "deviation" of the pose from the self locator. (As long as it is near the correct pose.) */
    float lineRelationCorridor; /**< The corridor used for relating seen lines with field lines. */
    Pose2D odometryDeviation; /**< The percentage inaccuracy of the odometry. */
    Vector2<> odometryRotationDeviation; /**< A rotation deviation of each walked mm. */
    Pose2D filterProcessDeviation; /**< The process noise for estimating the robot pose. */
    Vector2<> robotRotationDeviation; /**< Deviation of the rotation of the robot's torso */
    Vector2<> robotRotationDeviationInStand; /**< Deviation of the rotation of the robot's torso when he is standing. */

  private:
    /**
    * The method makes the object streamable.
    * @param in The stream from which the object is read.
    * @param out The stream to which the object is written.
    */
    virtual void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN;
      STREAM(respawnMaxPoseDistance);
      STREAM(respawnPoseDeviation);
      STREAM(lineRelationCorridor);
      STREAM(odometryDeviation);
      STREAM(odometryRotationDeviation);
      STREAM(filterProcessDeviation);
      STREAM(robotRotationDeviation);
      STREAM(robotRotationDeviationInStand);
      STREAM_REGISTER_FINISH;
    }
  };

  /**
  * A field line relative to the robot.
  */
  class FieldLine
  {
  public:
    Vector2<> start; /**< The starting point of the line. */
    Vector2<> end; /**< The ending point of the line. */
    Vector2<> dir; /**< The normalized direction of the line (from starting point). */
    float length; /**< The length of the line. */
    bool vertical; /**< Whether this is a vertical or horizontal line. */
  };

  /**
  * A dedicated line percept relative to the robot (in floats).
  */
  class LinePercept
  {
  public:
    Vector2<> start; /**< The starting point on field */
    Vector2<> end; /**< The ending point on field */
    int index; /**< field line index */
  };

  /**
  * A dedicated goal percept relative to the robot (in floats).
  */
  class GoalPercept
  {
  public:
    Vector2<> position;
    ENUM(Membership, OWN_TEAM, OPPONENT_TEAM);
    Membership membership;
    GoalPost::Position side;
  };

  Parameters p; /**< A set of parameters for the module. */
  FieldLine fieldLines[24]; /**< Relevant field lines  */
  int countOfFieldLines; /**< Count of relevant field lines. */
  Vector2<> goalPosts[GoalPercept::numOfMemberships][GoalPost::numOfPositions]; /**< The positions of the goal posts. */

  std::vector<LinePercept> dedicatedLinePercepts;
  std::vector<GoalPercept> dedicatedGoalPercepts;

  unsigned int lastPoseSetTime; /**< time stamp of the last manual pose regulation, pose mirroring or particle cluster changeover of PotentialRobotPose */
  unsigned int lastResetTime; /**< The time when the robot pose was reset. */

  /**
  * Initializes the module.
  */
  void init();

  /**
  * Updates the filtered robot pose provided from this module.
  * @param filteredRobotPose The robotPose updated from the module
  */
  void update(FilteredRobotPose& filteredRobotPose);
  Vector3f mean; /**< The estimated pose. */
  Matrix3x3f cov; /**< The covariance matrix of the estimate. */

  void computeCholeskyDecompositionOfCov();
  Matrix3x3f l; /**< the computed cholesky decomposition */
  void generateSigmaPoints();
  Vector3f sigmaPoints[7];

  void motionUpdate();
  void addOdometryToSigmaPoints();
  Pose2D odometryOffset;
  Pose2D lastOdometryData; /**< OdometryData of the previous iteration. */
  void computeMeanOfSigmaPoints();
  void computeCovOfSigmaPoints();
  void addProcessNoise();

  void lineSensorUpdate(bool vertical, const Vector2f& reading, const Matrix2x2f& readingCov);
  void computeLineReadings(bool vertical);
  Vector2f lineReadings[7];
  void computeMeanOfLineReadings();
  Vector2f lineReadingMean;
  void computeCovOfLineReadingsAndSigmaPoints();
  Matrix2x3f lineReadingAndMeanCov;
  void computeCovOfLineReadingsReadings();
  Matrix2x2f lineReadingCov;

  void landmarkSensorUpdate(const Vector2<>& landmarkPosition, const Vector2f& reading, const Matrix2x2f& readingCov);
  void computeLandmarkReadings(const Vector2<>& landmarkPosition);
  Vector2f landmarkReadings[7];
  void computeMeanOfLandmarkReadings();
  Vector2f landmarkReadingMean;
  void computeCovOfLandmarkReadingsAndSigmaPoints();
  Matrix2x3f landmarkReadingAndMeanCov;
  void computeCovOfLandmarkReadingsReadings();
  Matrix2x2f landmarkReadingCov;

  void poseSensorUpdate(const Vector3f& reading, const Matrix3x3f& readingCov);
  void computePoseReadings();
  Vector3f poseReadings[7];
  void computeMeanOfPoseReadings();
  Vector3f poseReadingMean;
  void computeCovOfPoseReadingsAndSigmaPoints();
  Matrix3x3f poseReadingAndMeanCov;
  void computeCovOfPoseReadingsReadings();
  Matrix3x3f poseReadingCov;

  bool intersectLineWithLine(const Vector2<>& lineBase1, const Vector2<>& lineDir1, const Vector2<>& lineBase2, const Vector2<>& lineDir2, Vector2<>& intersection) const;
  float getSqrDistanceToLine(const Vector2<>& base, const Vector2<>& dir, float length, const Vector2<>& point) const;
  float getSqrDistanceToLine(const Vector2<>& base, const Vector2<>& dir, const Vector2<>& point) const;
  Vector2<> getOrthogonalProjection(const Vector2<>& base, const Vector2<>& dir, const Vector2<>& point) const;

  void useLinePercept(const LinePercept& linePercept, const FieldLine& fieldLine);
  void useCenterCircle(const Vector2<>& circlePos);
  void useCenterCircleWithMiddleLine(const Vector2<>& circlePos, const LinePercept& linePercept);
  void useGoalPost(const Vector2<>& postPos, const Vector2<>& postOnField);

  Matrix2x2f getCovOfPointInWorld(const Vector2<>& pointInWorld, float pointZInWorld) const;
  Matrix2x2f getCovOfCircle(const Vector2<>& circlePos) const;
};
