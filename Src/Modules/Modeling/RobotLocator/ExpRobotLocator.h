/**
 * @file ExpRobotLocator.h
 * This module provides the RobotsModel. A Kalman filter is used to smooth the RobotPercept measurements.
 * @author Colin Graf
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Perception/RobotPercept.h"
#include "Representations/Perception/TeamMarkerSpots.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/RobotsModel.h"

MODULE(ExpRobotLocator)
  REQUIRES(FrameInfo)
  REQUIRES(FieldDimensions)
  REQUIRES(CameraInfo)
  REQUIRES(RobotPercept)
  REQUIRES(TeamMarkerSpots)
  REQUIRES(CameraMatrix)
  REQUIRES(ImageCoordinateSystem)
  REQUIRES(OdometryData)
  REQUIRES(RobotPose)
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(RobotsModel)
END_MODULE

/**
 * @class RobotLocator
 * This module implements the robot localization with a Kalman filter.
 */
class ExpRobotLocator : public ExpRobotLocatorBase
{
  /**
   * @class Parameters
   * The parameters for the RobotLocator.
   */
  class Parameters : public Streamable
  {
  public:
    Vector3<> processDeviation; /**< The process noise */
    Vector2<> robotRotationDeviation; /**< Deviation of the rotation of the robot's torso (used for estimating covariances of percepted team markers) */
    Pose2D odometryDeviation; /**< The percentage inaccuracy of the odometry */
    float heightDeviation;

    float robotFeetRadius;
    float robotWaistbandRadius;
    float robotMaxHorizontalHeight;
    float robotDefaultHeight;

    float angleTolerance;
    float distanceTolerance;

    unsigned robotNotUpdatedTimeout;
    unsigned robotNotSeenTimeout;

  private:
    void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN;
      STREAM(processDeviation);
      STREAM(robotRotationDeviation);
      STREAM(odometryDeviation);
      STREAM(heightDeviation);
      STREAM(robotFeetRadius);
      STREAM(robotWaistbandRadius);
      STREAM(robotMaxHorizontalHeight);
      STREAM(robotDefaultHeight);
      STREAM(angleTolerance);
      STREAM(distanceTolerance);
      STREAM(robotNotUpdatedTimeout);
      STREAM(robotNotSeenTimeout);
      STREAM_REGISTER_FINISH;
    }
  };

  /**
   * @class Robot
   * A Kalman filter state for a robot.
   */
  class Filter
  {
  public:
    bool teamRed;
    Vector2<> pos;
    Matrix2x2<> posCov;
    float height;
    float heightVar;
    float angle;
    float distance;
    unsigned timestamp;
    unsigned timeSinceNotSeen;

    Filter(bool teamRed, const Vector2<>& pos, const Matrix2x2<>& posCov, float height, float heightVar, float angle, float distance, unsigned timestamp) : teamRed(teamRed), pos(pos), posCov(posCov), height(height), heightVar(heightVar), angle(angle), distance(distance), timestamp(timestamp), timeSinceNotSeen(0) {}

    void updateHeight(float markerHeight, float markerHeightVar, float robotMaxHorizontalHeight);
    void updatePos(const Vector2<>& posOnField, const Matrix2x2<>& cov);
  };

  Parameters p;

  void init();

  /**
   * Processes the current robotPercept and updates the robotsModel.
   * @param robotsModel The percepted robots on the field.
   */
  void update(RobotsModel& robotsModel);
  std::vector<Filter> filters;

  /**
   * Applies the motion model on all robots.
   */
  void motionUpdate();
  OdometryData lastOdometryData; /**< This is used to calculate the odometry offset. */

  /**
   * Updates the robotsModel with the perceived robots.
   */
  void sensorUpdate();
  float averageHeight[2]; /**< average (not horizontal) team marker height for each team color */
  float averageHeightVar[2]; /**< variance of average team maker height */

  void removeDatedFilters();

  Matrix2x2<> getCovOfPixelInWorld(const Vector2<>& correctedPointInImage, float pointZInWorld) const;
};
