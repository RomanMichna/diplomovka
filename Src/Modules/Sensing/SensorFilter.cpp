/**
* @file SensorFilter.cpp
* Implementation of module SensorFilter.
* @author Colin Graf
*/

#include "SensorFilter.h"
#include "Tools/Debugging/DebugDrawings.h"

MAKE_MODULE(SensorFilter, Sensing)

SensorFilter::SensorFilter()
{
  InConfigMap stream(Global::getSettings().expandRobotFilename("usCalibration.cfg"));
  if(stream.exists())
  {
    stream >> *this;
  }
  
  ASSERT(medianBufferSize > 0);
  ASSERT(maxScanRange > 0);

  medianBuffer.reserve(medianBufferSize);
  
  for (int i = 0; i < 3; i++)
  {
    llBuffer.push_back(DynamicRingBuffer<float>(medianBufferSize));
    rrBuffer.push_back(DynamicRingBuffer<float>(medianBufferSize));
    lrBuffer.push_back(DynamicRingBuffer<float>(medianBufferSize));
    rlBuffer.push_back(DynamicRingBuffer<float>(medianBufferSize));
  }
}

inline bool SensorFilter::isUsSensorDataValid(float value)
{
  return value > 0 && value <= maxScanRange;
}

void SensorFilter::update(FilteredSensorData& filteredSensorData)
{
  // copy sensor data (except gyro and acc)
  Vector2<> gyro(filteredSensorData.data[SensorData::gyroX], filteredSensorData.data[SensorData::gyroY]);
  Vector3<> acc(filteredSensorData.data[SensorData::accX], filteredSensorData.data[SensorData::accY], filteredSensorData.data[SensorData::accZ]);
  (SensorData&)filteredSensorData = theSensorData;
  filteredSensorData.data[SensorData::gyroX] = gyro.x;
  filteredSensorData.data[SensorData::gyroY] = gyro.y;
  filteredSensorData.data[SensorData::accX] = acc.x;
  filteredSensorData.data[SensorData::accY] = acc.y;
  filteredSensorData.data[SensorData::accZ] = acc.z;

  if(theDamageConfiguration.USLDefect)
  {
    for(int i = SensorData::usL; i < SensorData::usLEnd; ++i)
      filteredSensorData.data[i] = 2550.0f;
  }

  if(theDamageConfiguration.USRDefect)
  {
    for(int i = SensorData::usR; i < SensorData::usREnd; ++i)
      filteredSensorData.data[i] = 2550.0f;
  }

  //A Very simple USData Filter. It just adds the first three valid measurements

  if(theSensorData.usActuatorMode == SensorData::leftToBoth)
  {
    filteredSensorData.usChanged = lastUsActuatorMode != SensorData::leftToBoth;
    if(filteredSensorData.usChanged)
    {
      // ensure to drop old values
      filteredSensorData.leftToLeft.clear();
      filteredSensorData.leftToRight.clear();

      float data;

      // add up to three valid values for leftToleft and leftToRight
      for (unsigned int i = 0; i < 3; i++)
      {
        data = filteredSensorData.data[SensorData::usL + i];
        llBuffer[i].add(data);
        data = getMedian(llBuffer[i]);

        if (isUsSensorDataValid(data))
        {
          filteredSensorData.leftToLeft.push_back(data);
        }

        data = filteredSensorData.data[SensorData::usR + i];
        lrBuffer[i].add(data);
        data = getMedian(lrBuffer[i]);

        if (isUsSensorDataValid(data))
        {
          filteredSensorData.leftToRight.push_back(data);
        }
      }
    }
  }
  else if(theSensorData.usActuatorMode == SensorData::rightToBoth)
  {
    filteredSensorData.usChanged = lastUsActuatorMode != SensorData::rightToBoth;

    if(filteredSensorData.usChanged)
    {
      ASSERT(theSensorData.usActuatorMode == SensorData::rightToBoth || SystemCall::getMode() == SystemCall::logfileReplay);
      // ensure to drop old values
      filteredSensorData.rightToLeft.clear();
      filteredSensorData.rightToRight.clear();

      float data;

      // add up to 3 valid values for rightToLeft and rightToRight
      for (unsigned int i = 0; i < 3; i++)
      {
        data = filteredSensorData.data[SensorData::usL + i];
        rlBuffer[i].add(data);
        data = getMedian(rlBuffer[i]);

        if (isUsSensorDataValid(data))
        {
          filteredSensorData.rightToLeft.push_back(data);
        }

        data = filteredSensorData.data[SensorData::usR + i];
        rrBuffer[i].add(data);
        data = getMedian(rrBuffer[i]);

        if (isUsSensorDataValid(data))
        {
          filteredSensorData.rightToRight.push_back(data);
        }
      }
    }
  }

  lastUsActuatorMode = theSensorData.usActuatorMode;

  // take calibrated inertia sensor data
  for(int i = 0; i < 2; ++i)
  {
    if(theInertiaSensorData.gyro[i] != InertiaSensorData::off)
      filteredSensorData.data[SensorData::gyroX + i] = theInertiaSensorData.gyro[i];
    else if(filteredSensorData.data[SensorData::gyroX + i] == SensorData::off)
      filteredSensorData.data[SensorData::gyroX + i] = 0.f;
  }
  filteredSensorData.data[SensorData::gyroZ] = 0.f;
  for(int i = 0; i < 3; ++i)
  {
    if(theInertiaSensorData.acc[i] != InertiaSensorData::off)
      filteredSensorData.data[SensorData::accX + i] = theInertiaSensorData.acc[i] / 9.80665f;
    else if(filteredSensorData.data[SensorData::accX + i] == SensorData::off)
      filteredSensorData.data[SensorData::accX + i] = 0.f;
  }

  // take orientation data
  filteredSensorData.data[SensorData::angleX] = atan2(theOrientationData.rotation.c1.z, theOrientationData.rotation.c2.z);
  filteredSensorData.data[SensorData::angleY] = atan2(-theOrientationData.rotation.c0.z, theOrientationData.rotation.c2.z);

  // some code for calibrating the gain of the gyro sensors:
#ifndef RELEASE
  if(filteredSensorData.data[SensorData::gyroX] != SensorData::off)
  {
    gyroAngleXSum += filteredSensorData.data[SensorData::gyroX] * (theSensorData.timeStamp - lastIteration) * 0.001f;
    gyroAngleXSum = normalize(gyroAngleXSum);
    lastIteration = theSensorData.timeStamp;
  }
  PLOT("module:SensorFilter:gyroAngleXSum", gyroAngleXSum);
  DEBUG_RESPONSE_ONCE("module:SensorFilter:gyroAngleXSum:reset", gyroAngleXSum = 0.f;);
#endif


  if(theSensorData.usActuatorMode == SensorData::leftToBoth)
  {

    PLOT("module:SensorFilter:LeftToRight", theSensorData.data[SensorData::usR]);
    PLOT("module:SensorFilter:LeftToLeft", theSensorData.data[SensorData::usL]);

    if (filteredSensorData.leftToLeft.size())
    {
      PLOT("module:SensorFilter:LeftToLeftMedian", filteredSensorData.leftToLeft.front());
    } 
    else
    {
      PLOT("module:SensorFilter:LeftToLeftMedian", 2550);
    }
    
    if (filteredSensorData.rightToLeft.size())
    {
      PLOT("module:SensorFilter:RightToLeftMedian", filteredSensorData.rightToLeft.front());
    } 
    else
    {
      PLOT("module:SensorFilter:RightToLeftMedian", 2550);
    }
  }
  else if(theSensorData.usActuatorMode == SensorData::rightToBoth)
  {
    PLOT("module:SensorFilter:RightToLeft", theSensorData.data[SensorData::usL]);
    PLOT("module:SensorFilter:RightToRight", theSensorData.data[SensorData::usR]); 

    if (filteredSensorData.leftToRight.size())
    {
      PLOT("module:SensorFilter:LeftToRightMedian", filteredSensorData.leftToRight.front());
    } 
    else
    {
      PLOT("module:SensorFilter:LeftToRightMedian", 2550);
    }
    
    if (filteredSensorData.rightToRight.size())
    {
      PLOT("module:SensorFilter:RightToRightMedian", filteredSensorData.rightToRight.front());
    } 
    else
    {
      PLOT("module:SensorFilter:RightToRightMedian", 2550);
    }
  }
  
  PLOT("module:SensorFilter:UsChanged", filteredSensorData.usChanged);

  //
  PLOT("module:SensorFilter:rawAngleX", theSensorData.data[SensorData::angleX]);
  PLOT("module:SensorFilter:rawAngleY", theSensorData.data[SensorData::angleY]);

  PLOT("module:SensorFilter:rawAccX", theSensorData.data[SensorData::accX]);
  PLOT("module:SensorFilter:rawAccY", theSensorData.data[SensorData::accY]);
  PLOT("module:SensorFilter:rawAccZ", theSensorData.data[SensorData::accZ]);

  PLOT("module:SensorFilter:rawGyroX", theSensorData.data[SensorData::gyroX]);
  PLOT("module:SensorFilter:rawGyroY", theSensorData.data[SensorData::gyroY]);
  PLOT("module:SensorFilter:rawGyroZ", theSensorData.data[SensorData::gyroZ]);

  PLOT("module:SensorFilter:angleX", filteredSensorData.data[SensorData::angleX]);
  PLOT("module:SensorFilter:angleY", filteredSensorData.data[SensorData::angleY]);

  PLOT("module:SensorFilter:accX", filteredSensorData.data[SensorData::accX]);
  PLOT("module:SensorFilter:accY", filteredSensorData.data[SensorData::accY]);
  PLOT("module:SensorFilter:accZ", filteredSensorData.data[SensorData::accZ]);

  PLOT("module:SensorFilter:gyroX", filteredSensorData.data[SensorData::gyroX] != float(SensorData::off) ? filteredSensorData.data[SensorData::gyroX] : 0);
  PLOT("module:SensorFilter:gyroY", filteredSensorData.data[SensorData::gyroY] != float(SensorData::off) ? filteredSensorData.data[SensorData::gyroY] : 0);
  PLOT("module:SensorFilter:gyroZ", filteredSensorData.data[SensorData::gyroZ] != float(SensorData::off) ? filteredSensorData.data[SensorData::gyroZ] : 0);
}

float SensorFilter::getMedian(const DynamicRingBuffer<float>& buffer)
{
  medianBuffer.clear();

  for (int i = 0; i < buffer.size(); ++i)
  {
    medianBuffer.push_back(buffer.get(i));
  }

  sort(medianBuffer.begin(), medianBuffer.end());
  return medianBuffer[medianBuffer.size() / 2];
}

