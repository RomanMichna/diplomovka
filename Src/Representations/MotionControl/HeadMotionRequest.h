/**
* @file Representations/MotionControl/HeadMotionRequest.h
* This file declares a class that represents the requested head motion.
* @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
*/

#pragma once

#include "Tools/Math/Vector3.h"
#include "Tools/Enum.h"

/**
* @class HeadMotionRequest
* A class that represents the requested head motion.
*/
class HeadMotionRequest : public Streamable
{
public:
  ENUM(Mode,
    panTiltMode,        /**< Use \c pan, \c tilt and \c speed. */
    targetMode,         /**< (A target relative to the center of hip.) Use \c target and \c speed. */
    targetOnGroundMode  /**< Use \c target and \c speed. */
  );

  ENUM(CameraControlMode,
    autoCamera,
    lowerCamera,
    upperCamera
  );

  Mode mode;        /**< The active head motion mode. */
  CameraControlMode cameraControlMode; /**< The active camera control mode. */
  bool watchField;  /**< True, if as much as possible of the field should be watched instead of centering the target in the image. */
  float pan,        /**< Head pan target angle in radians. */
        tilt,       /**< Head tilt target angle in radians. */
        speed;      /**< Maximum joint speed to reach target angles in radians/s. */
  Vector3<> target; /**< Look at target relative to the robot. */

  /** Default constructor. */
  HeadMotionRequest() : mode(panTiltMode), cameraControlMode(lowerCamera), watchField(false), pan(0), tilt(0), speed(1), target(1, 0, 0) {}

protected:
  /**
  * The method makes the object streamable.
  * @param in The stream from which the object is read
  * @param out The stream to which the object is written
  */
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(mode);
    STREAM(cameraControlMode);
    STREAM(watchField);
    STREAM(pan);
    STREAM(tilt);
    STREAM(speed);
    STREAM(target);
    STREAM_REGISTER_FINISH;
  }
};

class TeamHeadControlState : public Streamable
{
public:
  TeamHeadControlState() : checksBall(false), usesActiveVision(false) {};
  bool checksBall;
  bool usesActiveVision;

private:
  /**
  * The method makes the object streamable.
  * @param in The stream from which the object is read.
  * @param out The stream to which the object is written.
  */
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(checksBall);
    STREAM(usesActiveVision);
    STREAM_REGISTER_FINISH;
  }
};
