/**
* @file Representations/Infrastructure/JointDataDeg.h
* This file declares a class to represent the joint angles in degrees.
*/

#pragma once

#include "JointData.h"
#include "Tools/Math/Common.h"
#include "Platform/BHAssert.h"

/**
* @class JointDataDeg
* A class that wraps joint data to be transmitted in degrees.
*/
class JointDataDeg : public JointData
{
private:
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    if(jointData)
    {
      ASSERT(out);
      for(int i = 0; i < JointData::numOfJoints; ++i)
        angles[i] = jointData->angles[i] == JointData::off ? JointData::off
                    : floor(toDegrees(jointData->angles[i]) * 10.0f + 0.5f) / 10.0f;
      timeStamp = jointData->timeStamp;

      STREAM(angles);
      STREAM(timeStamp);
      float cycleTime = 0.1f;
      STREAM(cycleTime); // obsolete, kept for log file compatibility
    }
    else
    {
      STREAM_BASE(JointData);
    }
    STREAM_REGISTER_FINISH;
  }

  JointData* jointData; /**< The joint data that is wrapped. */

public:
  /**
  * Default constructor.
  */
  JointDataDeg() : jointData(0) {}

  /**
  * Constructor.
  * @param jointData The joint data that is wrapped.
  */
  JointDataDeg(JointData& jointData) : jointData(&jointData) {}

  /**
  * Assignment operator.
  */
  JointDataDeg& operator=(const JointDataDeg& other)
  {
    if(jointData)
      for(int i = 0; i < JointData::numOfJoints; ++i)
        jointData->angles[i] = other.angles[i] == JointData::off ? JointData::off
                               : fromDegrees(other.angles[i]);
    else
      *((JointData*) this) = other;
    return *this;
  }
};
