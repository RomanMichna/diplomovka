/**
* @file SensorFilter.h
* Declaration of module SensorFilter.
* @author Colin Graf
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Representations/Sensing/InertiaSensorData.h"
#include "Representations/Sensing/OrientationData.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Tools/DynamicRingBuffer.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Settings.h"
#include <algorithm>

#define buffersize 8

MODULE(SensorFilter)
  REQUIRES(SensorData)
  REQUIRES(InertiaSensorData)
  REQUIRES(OrientationData)
  REQUIRES(DamageConfiguration)
  PROVIDES_WITH_MODIFY_AND_OUTPUT(FilteredSensorData)
END_MODULE

/**
* A module for sensor data filtering.
*/
class SensorFilter : public SensorFilterBase, public Streamable
{
public:
  SensorFilter();

  /**
  * Updates the FilteredSensorData representation.
  * @param filteredSensorData The sensor data representation which is updated by this module.
  */
  void update(FilteredSensorData& filteredSensorData);

private:

  SensorData::UsActuatorMode lastUsActuatorMode;
  float gyroAngleXSum;
  unsigned int lastIteration;
  unsigned int medianBufferSize;
  unsigned int maxScanRange;
  vector<DynamicRingBuffer<float> > llBuffer;
  vector<DynamicRingBuffer<float> > lrBuffer;
  vector<DynamicRingBuffer<float> > rlBuffer;
  vector<DynamicRingBuffer<float> > rrBuffer;

  vector<float> medianBuffer;

  float getMedian(const DynamicRingBuffer<float>& buffer);

  /**
   * Returns whether the given value is inside the bounds
   * of the the us-sensor
   * @param value the value the check
   */
  inline bool isUsSensorDataValid(float value);

  /**
  * The method makes the object streamable.
  * @param in The stream from which the object is read
  * @param out The stream to which the object is written
  */
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(medianBufferSize);
    STREAM(maxScanRange);
    STREAM_REGISTER_FINISH;
  }
};
