/**
 * @file RobotsModel.h
 *
 * @author <a href="mailto:afabisch@tzi.de>Alexander Fabisch</a>
 */

#pragma once

#include "Tools/Math/Matrix2x2.h"
#include <vector>

/**
 * @class RobotsModel
 * This representation contains all percepted and smoothed robots in relative
 * field coordinates.
 */
class RobotsModel : public Streamable
{
  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(robots);
    STREAM_REGISTER_FINISH;
  }
public:
  /**
   * @class Robot
   * A robot on the field.
   */
  class Robot : public Streamable
  {
    void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN;
      STREAM(relPosOnField);
      STREAM(teamRed);
      STREAM(standing);
      STREAM(covariance);
      STREAM(timeStamp);
      STREAM_REGISTER_FINISH;
    }

  public:
    Vector2<> relPosOnField;  /**< Relative position of the robot on the field. */
    bool teamRed;             /**< Red team or blue team? */
    bool standing;            /**< Is this robot standing or horizontal? */
    Matrix2x2<> covariance;   /**< covariance of a seen robot */
    unsigned timeStamp;       /**< Timestamp of the last update. */

    Robot() {}

    Robot(const Robot& other)
      : relPosOnField(other.relPosOnField),
        teamRed(other.teamRed),
        standing(other.standing),
        covariance(other.covariance),
        timeStamp(other.timeStamp) {}

    Robot(const Vector2<>& relPosOnField, bool teamRed, bool standing,
          const Matrix2x2<>& covariance, unsigned timeStamp)
      : relPosOnField(relPosOnField),
        teamRed(teamRed),
        standing(standing),
        covariance(covariance),
        timeStamp(timeStamp) {}
  };

  typedef std::vector<Robot>::const_iterator RCIt;
  typedef std::vector<Robot>::iterator RIt;

  std::vector<Robot> robots;

  void draw();
};

/**
 * @class GroundTruthRobotsModel
 * The class contains the true RobotsModel.
 */
class GroundTruthRobotsModel : public RobotsModel
{
};

/**
 * @class RobotsModelCompressed
 * This class contains a compressed RobotsModel for team communication.
 */
class RobotsModelCompressed : public Streamable
{
  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(robots);
    STREAM_REGISTER_FINISH;
  }
public:
  class Robot : public Streamable
  {
    void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN;
      STREAM_COMPRESSED_POSITION(relPosOnField);
      STREAM(teamRed);
      STREAM(standing);
      STREAM(covXX);
      STREAM(covYY);
      STREAM(covXY);
      STREAM(timeStamp);
      STREAM_REGISTER_FINISH;
    }
  public:
    // 36 Bytes -> 22 Bytes
    Vector2<> relPosOnField;
    bool teamRed,
         standing;
    float covXX,
          covYY,
          covXY; // == covYX
    unsigned timeStamp;

    Robot()
    {
    }

    Robot(const RobotsModel::Robot& robot)
      : relPosOnField(robot.relPosOnField),
        teamRed(robot.teamRed),
        standing(robot.standing),
        covXX(robot.covariance[0][0]),
        covYY(robot.covariance[1][1]),
        covXY(robot.covariance[0][1]),
        timeStamp(robot.timeStamp)
    {
    }

    RobotsModel::Robot unpack()
    {
      return RobotsModel::Robot(
               relPosOnField,
               teamRed,
               standing,
               Matrix2x2<>(covXX, covXY, covXY, covYY),
               timeStamp
             );
    }
  };

  std::vector<Robot> robots;

  RobotsModelCompressed()
  {
  }

  RobotsModelCompressed(const RobotsModel& robotsModel)
  {
    robots.reserve(robotsModel.robots.size());
    for(size_t i = 0; i < robotsModel.robots.size(); i++)
    {
      robots.push_back(Robot(robotsModel.robots[i]));
    }
  }

  RobotsModel unpack()
  {
    RobotsModel robotsModel;
    robotsModel.robots.reserve(robots.size());
    for(size_t i = 0; i < robots.size(); i++)
    {
      robotsModel.robots.push_back(robots[i].unpack());
    }
    return robotsModel;
  }
};
