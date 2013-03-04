/**
 * @file RobotLocatorUKF.h
 * This module provides the RobotsModel. A Kalman filter is used to smooth the RobotPercept measurements.
 * @author <a href="mailto:afabisch@tzi.de">Alexander Fabisch</a>
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/TeamMateData.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Perception/RobotPercept.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/RobotsModel.h"
#include "UKF.h"
#include "Tools/Math/Geometry.h"
#include <list>

MODULE(RobotLocatorUKF)
  REQUIRES(FieldDimensions)
  REQUIRES(CameraInfo)
  REQUIRES(FrameInfo)
  REQUIRES(Image)
  REQUIRES(GameInfo)
  REQUIRES(RobotPercept)
  REQUIRES(CameraMatrix)
  REQUIRES(ImageCoordinateSystem)
  REQUIRES(OdometryData)
  REQUIRES(RobotPose)
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(RobotsModel)
END_MODULE

/**
 * @class RobotLocatorUKF
 * This module implements the robot localization with a Kalman filter.
 */
class RobotLocatorUKF : public RobotLocatorUKFBase
{
public:
  class Prediction : public UKF::Prediction
  {
  public:
    virtual Vector2<> operator()(const Vector3<>& u, const Vector2<>& x) const
    {
      const float s = sin(u.z);
      const float c = cos(u.z);
      const Matrix2x2<> a(Vector2<>(c, -s), Vector2<>(s, c));
      return a * x - Vector2<>(u.x, u.y);
    }
  };

  class Measurement : public UKF::Measurement
  {
  public:
    virtual Vector2<> operator()(const Vector2<>& x,
                                 const CameraMatrix& theCameraMatrix,
                                 const CameraInfo& theCameraInfo,
                                 const ImageCoordinateSystem& theImageCoordinateSystem) const
    {
      Vector2<int> pointInImage;
      Geometry::calculatePointInImage(
        Vector2<int>((int) x.x, (int) x.y),
        theCameraMatrix, theCameraInfo, pointInImage);
      return theImageCoordinateSystem.fromCorrectedApprox(pointInImage);
    }
  };

private:
  /**
   * @class Parameters
   * The parameters for the RobotLocatorUKF.
   */
  class Parameters : public Streamable
  {
    void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN;
      STREAM(maxDistOfRobots);
      STREAM(maxAngleDist);
      STREAM(maxRobots);
      STREAM(minMeanProbability);
      STREAM(robotWidth);
      STREAM(relativeOdometryRotationsNoiseStdDev);
      STREAM(motionNoiseStdDev);
      STREAM(initialStdDev);
      STREAM(maxAllowedDistance);
      STREAM(sensorNoiseStdDev);
      STREAM(distanceNoiseWeight);
      STREAM(angleSensorNoiseStdDev);
      STREAM(noSensorNoiseStdDev);
      STREAM(minDistanceToOpponentGoalPost);
      STREAM_REGISTER_FINISH;
    }
  public:
    float maxDistOfRobots;                      /**< The maximal euclidean distance of one robot between two cognition frames. */
    float maxAngleDist;                         /**< Maximal angle distance for unlocalizable robots. */
    unsigned maxRobots;                         /**< Maximal number of robots on the field. */
    float minMeanProbability;                   /**< Minimal probability in the mean area of the robot's probability function. */
    float robotWidth;                           /**< The approximated width of a standing robot. */
    float relativeOdometryRotationsNoiseStdDev; /**< The the ratio of the odometry rotation that is considered as noise. */
    Vector2<> motionNoiseStdDev;                /**< The standard deviations of the noise of the other robots' motion. */
    Vector2<> initialStdDev;                    /**< The standard deviations of an initial distribution of a percepted robot. */
    float maxAllowedDistance;                   /**< Maximal allowed distance of the percepted and the tracked robot for matching. */
    Vector2<> sensorNoiseStdDev;                /**< The standard deviations of the noise of the measurement. */
    float distanceNoiseWeight;                  /**< The weight with which the distance affects the sensor noise. */
    Vector2<> angleSensorNoiseStdDev;           /**< The standard deviations of the noise of an imprecise measurement. */
    Vector2<> noSensorNoiseStdDev;              /**< This will be added if there is no measurement update. */
    float minDistanceToOpponentGoalPost;        /**< This is used to avoid recognition of blue robots in blue goals. */
  };

  /**
   * @class Robot
   * A Kalman filter state.
   */
  class Robot
  {
  public:
    ENUM(RobotState, NEW, UPDATED, DELETED);

    UKF kf;                    /**< Contains the mean (relative position of the robot on the field) and the covariance. */
    bool standing;             /**< Is the robot standing? */
    bool teamRed;              /**< TEAM_RED or TEAM_BLUE? */
    int cyclesSinceLastUpdate; /**< Number of cognition frames since last update. */
    RobotState state;          /**< The robot's state. */
    unsigned timeStamp;        /**< Timestamp of the last update. */
    unsigned int confirmations; /**< Confirmed recognitions. */

    Robot() : state(DELETED), confirmations(-1) // this should only be used for vector::resize
    {}

    Robot(const Vector2<>& relPosOnField, const Matrix2x2<>& covariance,
          bool standing, bool teamRed, unsigned timeStamp)
      : kf(),
        standing(standing),
        teamRed(teamRed),
        cyclesSinceLastUpdate(0),
        state(NEW),
        timeStamp(timeStamp),
        confirmations(1)
    {
      kf.init(g, h, relPosOnField, covariance);
    }

    inline bool operator<(const Robot& other) const
    {
      return this->kf.approximateProbabilityAtMeanArea(0.5f) < other.kf.approximateProbabilityAtMeanArea(0.5f);
    }

    inline void cycle()
    {
      cyclesSinceLastUpdate++;
    }

    inline bool notUpdatedYet() const
    {
      return cyclesSinceLastUpdate != 0;
    }
  };

  class RobotIsDeleted
  {
  public:
    bool operator()(const Robot& robot)
    {
      return robot.state == Robot::DELETED;
    }
  };

  typedef std::vector<Robot>::const_iterator RLCI;
  typedef std::vector<Robot>::iterator RLI;

  Parameters params;
  static Prediction g;
  static Measurement h;
  RobotIsDeleted robotIsDeleted;

  int lastState;

  /** The currently localized robots. */
  std::vector<Robot> robots;
  /** A vector that contains all robots that will be united during the current frame. */
  std::vector<Robot> unitedRobots;
  /** This is used to calculate the odometry offset. */
  OdometryData lastOdometryData;

  void init();
  /**
   * Processes the current robotPercept and updates the robotsModel.
   * @param robotsModel The percepted robots on the field.
   */
  void update(RobotsModel& robotsModel);
  /**
   * Applies the motion model on the current robots.
   */
  void motionUpdate();
  /**
   * Updates the robotsModel with the percepted robots.
   */
  void sensorUpdate();
  /**
   * Unites robots that would actually stand upon each other.
   */
  void uniteNearRobots();
  /**
   * Marks robots that will be deleted.
   */
  void markRobotsToDelete();
  /**
   * Rejects robots that are too uncertain, updates the RobotsModel.
   * @param robotsModel The old RobotsModel.
   */
  void cycle(RobotsModel& robotsModel);

  /**
   * Updates the position of a robot in the model or adds a new robot.
   * @param newRobot The new percepted robot.
   */
  void updateMostLikelyOldRobot(const RobotPercept::Robot& newRobot);
  void updateRobotInDirection(const RobotPercept::Robot& newRobot);

  /**
   * @param relPosOnField The relative position of the robot on the field.
   * @return Should the robot be visible on the current image?
   */
  bool shouldBeVisible(
    const CameraMatrix& theCameraMatrix,
    const CameraInfo& theCameraInfo,
    const Image& theImage,
    const ImageCoordinateSystem& theImageCoordinateSystem,
    const Vector2<>& relPosOnField);
};
