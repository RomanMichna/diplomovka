/**
* @file Representations/MotionControl/HeadAngleRequest.h
* This file declares a class that represents a request to set specific angles for the head joint.
* @author Felix Wenk
*/

#pragma once

#include "Tools/Streams/Streamable.h"
#include "Tools/Math/Vector3.h"

/**
* @class HeadAngleRequest
* A class that represents the requested head angles.
*/
class HeadAngleRequest : public Streamable
{
public:
  float pan,        /**< Head pan target angle in radians. */
        tilt,       /**< Head tilt target angle in radians. */
        speed;      /**< Maximum joint speed to reach target angles in radians/s. */

  /** Default constructor. */
  HeadAngleRequest() : pan(0), tilt(0), speed(1) {}

protected:
  /**
  * The method makes the object streamable.
  * @param in The stream from which the object is read
  * @param out The stream to which the object is written
  */
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(pan);
    STREAM(tilt);
    STREAM(speed);
    STREAM_REGISTER_FINISH;
  }
};
