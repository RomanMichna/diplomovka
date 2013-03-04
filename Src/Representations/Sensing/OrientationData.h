/**
* @file OrientationData.h
* Declaration of class OrientationData.
* @author Colin Graf
*/

#pragma once

#include "Tools/Math/RotationMatrix.h"

/**
* @class OrientationData
* Encapsulates the orientation and velocity of the torso.
*/
class OrientationData : public Streamable
{
public:
  RotationMatrix rotation; /**< The rotation of the torso. */
  Vector3<> velocity; /**< The velocity along the x-, y- and z-axis relative to the torso. (in m/s) */

  /** Default constructor. */
  OrientationData() {}

private:
  /**
  * The method makes the object streamable.
  * @param in The stream from which the object is read
  * @param out The stream to which the object is written
  */
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(rotation);
    STREAM(velocity);
    STREAM_REGISTER_FINISH;
  }
};

class GroundTruthOrientationData  : public OrientationData {};
