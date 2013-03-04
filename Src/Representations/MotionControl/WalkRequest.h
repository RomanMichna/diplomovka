/**
* @file Representations/MotionControl/WalkRequest.h
* This file declares a class that represents a walk request.
* @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas R�fer</A>
* @author Colin Graf
*/

#pragma once

#include "Tools/Math/Pose2D.h"
#include "Tools/Enum.h"

/**
* @class WalkRequest
* A class that represents a walk request.
*/
class WalkRequest : public Streamable
{
public:
  ENUM(Mode,
    speedMode, /**< Interpret \c speed as absolute walking speed and ignore \c target. */
    percentageSpeedMode, /**< Interpret \c speed as percentage walking speed and ignore \c target. */
    targetMode /**< Use \c target as walking target relative to the current position of the robot and interpret \c speed as percentage walking speed. */
  );

  Mode mode; /**< The walking mode. */
  Pose2D speed; /**< Walking speeds, in percentage or mm/s and radian/s. */
  Pose2D target; /**< Walking target, in mm and radians, relative to the robot. Use either a speed or a target. */
  bool pedantic; /**< Allows to disable the step size stabilization. set it when precision is indispensable. */
  bool dribbling;
  bool armContactReactionEnabled;

  class DribbleTarget : public Streamable
  {
  public:
    DribbleTarget() : left(false) {}

    bool isValid() const
    {
      return !isnan(target.x) && !isnan(target.y)
          && !isnan(dribbleSpeed.x) && !isnan(dribbleSpeed.y)
          && !isnan(ballPosition.rotation) && !isnan(ballPosition.translation.x) && !isnan(ballPosition.translation.y);
    }

    Vector2<> target;
    Vector2<> dribbleSpeed;
    Pose2D ballPosition;
    bool left;
  private:
    virtual void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN;
      STREAM(target);
      STREAM(dribbleSpeed);
      STREAM(ballPosition);
      STREAM(left);
      STREAM_REGISTER_FINISH;
    }

    friend class WalkRequest;
  } dribbleTarget;

  ENUM(KickType,
    none, /**< do not kick */
    left, /**< kick using the left foot */
    right, /**< kick using the right foot */
    sidewardsLeft,
    sidewardsRight
  );

  KickType kickType;
  Vector2<> kickBallPosition;
  Vector2<> kickTarget;

  /** Default constructor. */
  WalkRequest() : mode(speedMode), pedantic(false), dribbling(false), armContactReactionEnabled(false), kickType(none) {}
  
  bool isValid() const
  {
    return !isnan(speed.rotation) && !isnan(speed.translation.x) && !isnan(speed.translation.y)
        && (mode != targetMode || (!isnan(target.rotation) && !isnan(target.translation.x) && !isnan(target.translation.y)))
        && (!dribbling || dribbleTarget.isValid())
        && (kickType == none || (!isnan(kickBallPosition.x) && !isnan(kickBallPosition.y)
                             && !isnan(kickTarget.x) && !isnan(kickTarget.y)));
  }

private:
  /**
  * The method makes the object streamable.
  * @param in The stream from which the object is read
  * @param out The stream to which the object is written
  */
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(mode);
    STREAM(speed);
    STREAM(target);
    STREAM(pedantic);
    STREAM(dribbling);
    STREAM(armContactReactionEnabled);
    STREAM(dribbleTarget);
    STREAM(kickType);
    STREAM(kickBallPosition);
    STREAM(kickTarget);
    STREAM_REGISTER_FINISH;
  }
};