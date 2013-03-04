/**
* @file Representations/Infrastructure/SensorData.h
*
* This file declares a class to represent the sensor data received from the robot.
*
* @author <A href="mailto:Harlekin@tzi.de">Philippe Schober</A>
*/

#pragma once

#include "Representations/Infrastructure/JointData.h"
#include <vector>

/**
* @class SensorData
* A class to represent the sensor data received from the robot.
*/
class SensorData : public Streamable
{
public:
  ENUM(Sensor,
    gyroX,
    gyroY,
    gyroZ,
    accX,
    accY,
    accZ,
    batteryLevel,
    fsrLFL,     //the feetsensors of the Nao-Robot
    fsrLFR,
    fsrLBL,
    fsrLBR,
    fsrRFL,
    fsrRFR,
    fsrRBL,
    fsrRBR,
    usL,
    usL1,
    usL2,
    usL3,
    usL4,
    usL5,
    usL6,
    usL7,
    usL8,
    usL9,
    usLEnd,
    usR = usLEnd,
    usR1,
    usR2,
    usR3,
    usR4,
    usR5,
    usR6,
    usR7,
    usR8,
    usR9,
    usREnd,
    angleX = usREnd,
    angleY
  );

  enum
  {
    off = JointData::off /**< A special value to indicate that the sensor is missing. */
  };

  ENUM(UsActuatorMode,
    leftToBoth,
    rightToBoth
  );

  float data[numOfSensors]; /**< The data of all sensors. */
  short currents[JointData::numOfJoints]; /**< The currents of all motors. */
  unsigned char temperatures[JointData::numOfJoints]; /**< The temperature of all motors. */
  unsigned timeStamp; /**< The time when the sensor data was received. */

  UsActuatorMode usActuatorMode; /**< The ultrasonice measure method which was used for measuring \c data[usL] and \c data[usR]. */
  unsigned usTimeStamp; /**< The time when the ultrasonic measurements were taken. */

  /**
  * Default constructor.
  */
  SensorData() : timeStamp(0), usActuatorMode(leftToBoth), usTimeStamp(0)
  {
    for(int i = 0; i < numOfSensors; ++i)
      data[i] = off;
    for(int i = 0; i < JointData::numOfJoints; ++i)
      currents[i] = temperatures[i] = 0;
  }

  /**
  * The method makes the object streamable.
  * @param in The stream from which the object is read.
  * @param out The stream to which the object is written.
  */
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(data);
    STREAM(currents);
    STREAM(temperatures);
    STREAM(timeStamp);
    STREAM(usActuatorMode);
    STREAM(usTimeStamp);
    STREAM_REGISTER_FINISH;
  }
};

/**
* A class to represent filtered sensor data.
*/
class FilteredSensorData : public SensorData 
{
public:
  std::vector<float> leftToLeft;
  std::vector<float> leftToRight;
  std::vector<float> rightToLeft;
  std::vector<float> rightToRight;

  bool usChanged;


  FilteredSensorData()
  {
    leftToLeft.reserve(10);
    leftToRight.reserve(10);
    rightToLeft.reserve(10);
    rightToRight.reserve(10);
    usChanged = false;
  }

  /**
  * The method makes the object streamable.
  * @param in The stream from which the object is read.
  * @param out The stream to which the object is written.
  */
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM_BASE(SensorData);
    STREAM(leftToLeft);
    STREAM(leftToRight);
    STREAM(rightToLeft);
    STREAM(rightToRight);
    STREAM(usChanged);
    STREAM_REGISTER_FINISH;
  }

};
