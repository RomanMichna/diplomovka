/**
* @file GroundContactState.h
* Declaration of class GroundContactState.
* @author Colin Graf
*/

#pragma once

#include "Tools/Streams/Streamable.h"

/**
* @class GroundContactState
* Describes whether we got contact with ground or not.
*/
class GroundContactState : public Streamable
{
public:
  /** Default constructor. */
  GroundContactState() : contact(true) {}

  bool contact; /**< a foot of the robot touches the ground */

private:
  /**
  * The method makes the object streamable.
  * @param in The stream from which the object is read
  * @param out The stream to which the object is written
  */
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(contact);
    STREAM_REGISTER_FINISH;
  }
};
