/*
 * Odometer.h
 *
 *  Created on: 25.05.2012
 *      Author: marcel
 */

#pragma once

#include "Tools/Streams/Streamable.h"

class Odometer : public Streamable
{
public:
  float distanceWalked;

  /** Constructor */
  Odometer() : distanceWalked(10000.f) {}
  
  float getDistanceFrom(float oldDist) const {return distanceWalked - oldDist;}

private:
  /**
  * The method makes the object streamable.
  * @param in The stream from which the object is read (if in != 0).
  * @param out The stream to which the object is written (if out != 0).
  */
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(distanceWalked);
    STREAM_REGISTER_FINISH;
  }
};
