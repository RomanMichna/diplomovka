/**
* @file bhuman.cpp
* Implementation of a NaoQi module that provides basic ipc NaoQi DCM access via semaphore and shared memory.
*/

#include <sys/types.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <semaphore.h>
#include <csignal>
#include <sys/resource.h>
#include <time.h>

#include <alcore/altypes.h>
#include <alcore/alerror.h>
#include <alcommon/albroker.h>
#include <alcommon/alproxy.h>
#include <alproxies/dcmproxy.h>
#include <alproxies/almemoryproxy.h>

#include "bhuman.h"

#define ALLOWED_FRAMEDROPS 3

int fd = -1;
LBHData* data = (LBHData*)MAP_FAILED;
sem_t* sem = SEM_FAILED;

AL::DCMProxy* proxy = 0;
AL::ALValue* positionRequest = 0;
AL::ALValue* hardnessRequest = 0;
AL::ALValue* usRequest = 0;
AL::ALValue* ledRequest = 0;
AL::ALMemoryProxy* memory = 0;
float* sensorPtrs[lbhNumOfSensorIds];
int timeOffset;
int dcmTime = 0; /**< Current dcm time, updated at each onPostProcess call. */
float requestedActuators[lbhNumOfActuatorIds];
unsigned lastUSSensorChangeTime = 0; /**< The last time we changed us sensor side. */

int frameDrops = ALLOWED_FRAMEDROPS + 1;

bool shuttingDown = false; /**< Is Nao shutting down? */

float lbhRatio = 1.f;

float zeroFloat = 0.f;

int ledIndex = 0;

static const char* sensorNames[] =
{
  "Device/SubDeviceList/HeadYaw/Position/Sensor/Value",
  "Device/SubDeviceList/HeadYaw/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/HeadYaw/Temperature/Sensor/Value",
  "Device/SubDeviceList/HeadPitch/Position/Sensor/Value",
  "Device/SubDeviceList/HeadPitch/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/HeadPitch/Temperature/Sensor/Value",
  "Device/SubDeviceList/LShoulderPitch/Position/Sensor/Value",
  "Device/SubDeviceList/LShoulderPitch/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/LShoulderPitch/Temperature/Sensor/Value",
  "Device/SubDeviceList/LShoulderRoll/Position/Sensor/Value",
  "Device/SubDeviceList/LShoulderRoll/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/LShoulderRoll/Temperature/Sensor/Value",
  "Device/SubDeviceList/LElbowYaw/Position/Sensor/Value",
  "Device/SubDeviceList/LElbowYaw/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/LElbowYaw/Temperature/Sensor/Value",
  "Device/SubDeviceList/LElbowRoll/Position/Sensor/Value",
  "Device/SubDeviceList/LElbowRoll/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/LElbowRoll/Temperature/Sensor/Value",
  "Device/SubDeviceList/RShoulderPitch/Position/Sensor/Value",
  "Device/SubDeviceList/RShoulderPitch/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/RShoulderPitch/Temperature/Sensor/Value",
  "Device/SubDeviceList/RShoulderRoll/Position/Sensor/Value",
  "Device/SubDeviceList/RShoulderRoll/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/RShoulderRoll/Temperature/Sensor/Value",
  "Device/SubDeviceList/RElbowYaw/Position/Sensor/Value",
  "Device/SubDeviceList/RElbowYaw/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/RElbowYaw/Temperature/Sensor/Value",
  "Device/SubDeviceList/RElbowRoll/Position/Sensor/Value",
  "Device/SubDeviceList/RElbowRoll/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/RElbowRoll/Temperature/Sensor/Value",
  "Device/SubDeviceList/LHipYawPitch/Position/Sensor/Value",
  "Device/SubDeviceList/LHipYawPitch/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/LHipYawPitch/Temperature/Sensor/Value",
  "Device/SubDeviceList/LHipRoll/Position/Sensor/Value",
  "Device/SubDeviceList/LHipRoll/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/LHipRoll/Temperature/Sensor/Value",
  "Device/SubDeviceList/LHipPitch/Position/Sensor/Value",
  "Device/SubDeviceList/LHipPitch/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/LHipPitch/Temperature/Sensor/Value",
  "Device/SubDeviceList/LKneePitch/Position/Sensor/Value",
  "Device/SubDeviceList/LKneePitch/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/LKneePitch/Temperature/Sensor/Value",
  "Device/SubDeviceList/LAnklePitch/Position/Sensor/Value",
  "Device/SubDeviceList/LAnklePitch/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/LAnklePitch/Temperature/Sensor/Value",
  "Device/SubDeviceList/LAnkleRoll/Position/Sensor/Value",
  "Device/SubDeviceList/LAnkleRoll/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/LAnkleRoll/Temperature/Sensor/Value",
  "Device/SubDeviceList/RHipRoll/Position/Sensor/Value",
  "Device/SubDeviceList/RHipRoll/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/RHipRoll/Temperature/Sensor/Value",
  "Device/SubDeviceList/RHipPitch/Position/Sensor/Value",
  "Device/SubDeviceList/RHipPitch/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/RHipPitch/Temperature/Sensor/Value",
  "Device/SubDeviceList/RKneePitch/Position/Sensor/Value",
  "Device/SubDeviceList/RKneePitch/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/RKneePitch/Temperature/Sensor/Value",
  "Device/SubDeviceList/RAnklePitch/Position/Sensor/Value",
  "Device/SubDeviceList/RAnklePitch/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/RAnklePitch/Temperature/Sensor/Value",
  "Device/SubDeviceList/RAnkleRoll/Position/Sensor/Value",
  "Device/SubDeviceList/RAnkleRoll/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/RAnkleRoll/Temperature/Sensor/Value",

  "Device/SubDeviceList/InertialSensor/GyrX/Sensor/Value",
  "Device/SubDeviceList/InertialSensor/GyrY/Sensor/Value",
  "Device/SubDeviceList/InertialSensor/GyrRef/Sensor/Value",
  "Device/SubDeviceList/InertialSensor/AccX/Sensor/Value",
  "Device/SubDeviceList/InertialSensor/AccY/Sensor/Value",
  "Device/SubDeviceList/InertialSensor/AccZ/Sensor/Value",
  "Device/SubDeviceList/Battery/Charge/Sensor/Value",
  "Device/SubDeviceList/LFoot/FSR/FrontLeft/Sensor/Value",
  "Device/SubDeviceList/LFoot/FSR/FrontRight/Sensor/Value",
  "Device/SubDeviceList/LFoot/FSR/RearLeft/Sensor/Value",
  "Device/SubDeviceList/LFoot/FSR/RearRight/Sensor/Value",
  "Device/SubDeviceList/RFoot/FSR/FrontLeft/Sensor/Value",
  "Device/SubDeviceList/RFoot/FSR/FrontRight/Sensor/Value",
  "Device/SubDeviceList/RFoot/FSR/RearLeft/Sensor/Value",
  "Device/SubDeviceList/RFoot/FSR/RearRight/Sensor/Value",
  "Device/SubDeviceList/US/Left/Sensor/Value",
  "Device/SubDeviceList/US/Left/Sensor/Value1",
  "Device/SubDeviceList/US/Left/Sensor/Value2",
  "Device/SubDeviceList/US/Left/Sensor/Value3",
  "Device/SubDeviceList/US/Left/Sensor/Value4",
  "Device/SubDeviceList/US/Left/Sensor/Value5",
  "Device/SubDeviceList/US/Left/Sensor/Value6",
  "Device/SubDeviceList/US/Left/Sensor/Value7",
  "Device/SubDeviceList/US/Left/Sensor/Value8",
  "Device/SubDeviceList/US/Left/Sensor/Value9",
  "Device/SubDeviceList/US/Right/Sensor/Value",
  "Device/SubDeviceList/US/Right/Sensor/Value1",
  "Device/SubDeviceList/US/Right/Sensor/Value2",
  "Device/SubDeviceList/US/Right/Sensor/Value3",
  "Device/SubDeviceList/US/Right/Sensor/Value4",
  "Device/SubDeviceList/US/Right/Sensor/Value5",
  "Device/SubDeviceList/US/Right/Sensor/Value6",
  "Device/SubDeviceList/US/Right/Sensor/Value7",
  "Device/SubDeviceList/US/Right/Sensor/Value8",
  "Device/SubDeviceList/US/Right/Sensor/Value9",
  "Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value",
  "Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value",
  "Device/SubDeviceList/RFoot/Bumper/Right/Sensor/Value",
  "Device/SubDeviceList/RFoot/Bumper/Left/Sensor/Value",
  "Device/SubDeviceList/LFoot/Bumper/Right/Sensor/Value",
  "Device/SubDeviceList/LFoot/Bumper/Left/Sensor/Value",
  "Device/SubDeviceList/ChestBoard/Button/Sensor/Value",
};

static const char* actuatorNames[] =
{
  "HeadYaw/Position/Actuator/Value",
  "HeadPitch/Position/Actuator/Value",
  "LShoulderPitch/Position/Actuator/Value",
  "LShoulderRoll/Position/Actuator/Value",
  "LElbowYaw/Position/Actuator/Value",
  "LElbowRoll/Position/Actuator/Value",
  "RShoulderPitch/Position/Actuator/Value",
  "RShoulderRoll/Position/Actuator/Value",
  "RElbowYaw/Position/Actuator/Value",
  "RElbowRoll/Position/Actuator/Value",
  "LHipYawPitch/Position/Actuator/Value",
  "LHipRoll/Position/Actuator/Value",
  "LHipPitch/Position/Actuator/Value",
  "LKneePitch/Position/Actuator/Value",
  "LAnklePitch/Position/Actuator/Value",
  "LAnkleRoll/Position/Actuator/Value",
  "RHipRoll/Position/Actuator/Value",
  "RHipPitch/Position/Actuator/Value",
  "RKneePitch/Position/Actuator/Value",
  "RAnklePitch/Position/Actuator/Value",
  "RAnkleRoll/Position/Actuator/Value",

  "HeadYaw/Hardness/Actuator/Value",
  "HeadPitch/Hardness/Actuator/Value",
  "LShoulderPitch/Hardness/Actuator/Value",
  "LShoulderRoll/Hardness/Actuator/Value",
  "LElbowYaw/Hardness/Actuator/Value",
  "LElbowRoll/Hardness/Actuator/Value",
  "RShoulderPitch/Hardness/Actuator/Value",
  "RShoulderRoll/Hardness/Actuator/Value",
  "RElbowYaw/Hardness/Actuator/Value",
  "RElbowRoll/Hardness/Actuator/Value",
  "LHipYawPitch/Hardness/Actuator/Value",
  "LHipRoll/Hardness/Actuator/Value",
  "LHipPitch/Hardness/Actuator/Value",
  "LKneePitch/Hardness/Actuator/Value",
  "LAnklePitch/Hardness/Actuator/Value",
  "LAnkleRoll/Hardness/Actuator/Value",
  "RHipRoll/Hardness/Actuator/Value",
  "RHipPitch/Hardness/Actuator/Value",
  "RKneePitch/Hardness/Actuator/Value",
  "RAnklePitch/Hardness/Actuator/Value",
  "RAnkleRoll/Hardness/Actuator/Value",

  "Face/Led/Red/Left/0Deg/Actuator/Value",
  "Face/Led/Red/Left/45Deg/Actuator/Value",
  "Face/Led/Red/Left/90Deg/Actuator/Value",
  "Face/Led/Red/Left/135Deg/Actuator/Value",
  "Face/Led/Red/Left/180Deg/Actuator/Value",
  "Face/Led/Red/Left/225Deg/Actuator/Value",
  "Face/Led/Red/Left/270Deg/Actuator/Value",
  "Face/Led/Red/Left/315Deg/Actuator/Value",
  "Face/Led/Green/Left/0Deg/Actuator/Value",
  "Face/Led/Green/Left/45Deg/Actuator/Value",
  "Face/Led/Green/Left/90Deg/Actuator/Value",
  "Face/Led/Green/Left/135Deg/Actuator/Value",
  "Face/Led/Green/Left/180Deg/Actuator/Value",
  "Face/Led/Green/Left/225Deg/Actuator/Value",
  "Face/Led/Green/Left/270Deg/Actuator/Value",
  "Face/Led/Green/Left/315Deg/Actuator/Value",
  "Face/Led/Blue/Left/0Deg/Actuator/Value",
  "Face/Led/Blue/Left/45Deg/Actuator/Value",
  "Face/Led/Blue/Left/90Deg/Actuator/Value",
  "Face/Led/Blue/Left/135Deg/Actuator/Value",
  "Face/Led/Blue/Left/180Deg/Actuator/Value",
  "Face/Led/Blue/Left/225Deg/Actuator/Value",
  "Face/Led/Blue/Left/270Deg/Actuator/Value",
  "Face/Led/Blue/Left/315Deg/Actuator/Value",
  "Face/Led/Red/Right/0Deg/Actuator/Value",
  "Face/Led/Red/Right/45Deg/Actuator/Value",
  "Face/Led/Red/Right/90Deg/Actuator/Value",
  "Face/Led/Red/Right/135Deg/Actuator/Value",
  "Face/Led/Red/Right/180Deg/Actuator/Value",
  "Face/Led/Red/Right/225Deg/Actuator/Value",
  "Face/Led/Red/Right/270Deg/Actuator/Value",
  "Face/Led/Red/Right/315Deg/Actuator/Value",
  "Face/Led/Green/Right/0Deg/Actuator/Value",
  "Face/Led/Green/Right/45Deg/Actuator/Value",
  "Face/Led/Green/Right/90Deg/Actuator/Value",
  "Face/Led/Green/Right/135Deg/Actuator/Value",
  "Face/Led/Green/Right/180Deg/Actuator/Value",
  "Face/Led/Green/Right/225Deg/Actuator/Value",
  "Face/Led/Green/Right/270Deg/Actuator/Value",
  "Face/Led/Green/Right/315Deg/Actuator/Value",
  "Face/Led/Blue/Right/0Deg/Actuator/Value",
  "Face/Led/Blue/Right/45Deg/Actuator/Value",
  "Face/Led/Blue/Right/90Deg/Actuator/Value",
  "Face/Led/Blue/Right/135Deg/Actuator/Value",
  "Face/Led/Blue/Right/180Deg/Actuator/Value",
  "Face/Led/Blue/Right/225Deg/Actuator/Value",
  "Face/Led/Blue/Right/270Deg/Actuator/Value",
  "Face/Led/Blue/Right/315Deg/Actuator/Value",
  "Ears/Led/Left/36Deg/Actuator/Value",
  "Ears/Led/Left/72Deg/Actuator/Value",
  "Ears/Led/Left/108Deg/Actuator/Value",
  "Ears/Led/Left/144Deg/Actuator/Value",
  "Ears/Led/Left/180Deg/Actuator/Value",
  "Ears/Led/Left/216Deg/Actuator/Value",
  "Ears/Led/Left/252Deg/Actuator/Value",
  "Ears/Led/Left/288Deg/Actuator/Value",
  "Ears/Led/Left/324Deg/Actuator/Value",
  "Ears/Led/Left/0Deg/Actuator/Value",
  "Ears/Led/Right/0Deg/Actuator/Value",
  "Ears/Led/Right/36Deg/Actuator/Value",
  "Ears/Led/Right/72Deg/Actuator/Value",
  "Ears/Led/Right/108Deg/Actuator/Value",
  "Ears/Led/Right/144Deg/Actuator/Value",
  "Ears/Led/Right/180Deg/Actuator/Value",
  "Ears/Led/Right/216Deg/Actuator/Value",
  "Ears/Led/Right/252Deg/Actuator/Value",
  "Ears/Led/Right/288Deg/Actuator/Value",
  "Ears/Led/Right/324Deg/Actuator/Value",
  "ChestBoard/Led/Red/Actuator/Value",
  "ChestBoard/Led/Green/Actuator/Value",
  "ChestBoard/Led/Blue/Actuator/Value",
  "LFoot/Led/Red/Actuator/Value",
  "LFoot/Led/Green/Actuator/Value",
  "LFoot/Led/Blue/Actuator/Value",
  "RFoot/Led/Red/Actuator/Value",
  "RFoot/Led/Green/Actuator/Value",
  "RFoot/Led/Blue/Actuator/Value",

  "US/Actuator/Value"
};

#define lbh_pi_2 1.5707963267948966f
static const float sitDownAngles[21] =
{
  -0.0135177,
  -(0.043983),
  0, //-( 0.780565-lbh_pi_2),
  -1.42764 + lbh_pi_2,
  0.126611,
  -0.25758,
  0, //-( 0.780528-lbh_pi_2),
  -(-1.43436 + lbh_pi_2),
  -(0.135423),
  -(-0.250192),
  -0.00440383,
  -(0.00831337),
  -0.88, //-1.00943,
  2.17113,
  -1.20265,
  -(0.00354111),
  0.00174092,
  -0.88, //-1.01021,
  2.17649,
  -1.20632,
  -0.00542875,
};

inline unsigned int getSystemTime()
{
  static unsigned int base = 0;
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  unsigned int time = (unsigned int)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000l);
  if(!base)
    base = time - 10000; // avoid time == 0, because it is often used as a marker
  return time - base;
}

void setEyeLeds(float* actuators)
{
  for(int i = faceLedRedLeft0DegActuator; i <= faceLedBlueRight315DegActuator; ++i)
    actuators[i] = 0.f;

  if(shuttingDown)
  {
    actuators[faceLedRedLeft180DegActuator] = 1.f;
    actuators[faceLedRedRight180DegActuator] = 1.f;
  }
  else if(data->state != okState)
  {
    // set the "libbhuman is active and bhuman crashed" leds
    float blink = float(dcmTime / 500 & 1);
    for(int i = faceLedRedLeft0DegActuator; i <= faceLedRedLeft315DegActuator; ++i)
      actuators[i] = blink;
    for(int i = faceLedRedRight0DegActuator; i <= faceLedRedRight315DegActuator; ++i)
      actuators[i] = 1. - blink;
  }
  else
  {
    // set the "libbhuman is active and bhuman is not running" leds
    float blink = float(dcmTime / 500 & 1);
    actuators[faceLedBlueLeft180DegActuator] = blink;
    actuators[faceLedBlueRight180DegActuator] = blink;
  }
}

void setBatteryLeds(float* actuators)
{
  static int rightEarLEDsChanged = dcmTime - 5000 - 1; // Last time when the right ear leds were changed by the B-Human code
  static float rightEarLEDs[earsLedRight324DegActuator - earsLedRight0DegActuator + 1]; // The previous state of the right ear LEDs

  for(int i = earsLedRight0DegActuator; i <= earsLedRight324DegActuator; ++i)
    if(actuators[i] != rightEarLEDs[i - earsLedRight0DegActuator])
    {
      rightEarLEDsChanged = dcmTime;
      rightEarLEDs[i - earsLedRight0DegActuator] = actuators[i];
    }

  if(lbhRatio > 0.f || dcmTime - rightEarLEDsChanged > 5000)
    for(int i = 0; i < int(*(sensorPtrs[batteryChargeSensor]) * 10.f) && i < 10; ++i)
      actuators[earsLedRight0DegActuator + i] = 1.f;
}

void copyLeds(const float* srcActuators, float* destActuators)
{
  for(int i = faceLedRedLeft0DegActuator; i <= rFootLedBlueActuator; ++i)
    destActuators[i] = srcActuators[i];
}

void sitDown(float lastRatio, float* actuators)
{
  static bool isSittingDown = false;
  if(lbhRatio > lastRatio && lbhRatio < 1.f)
  {
    static float startAngles[headYawHardnessActuator];
    static float startHardness[headYawHardnessActuator];
    if(!isSittingDown)
    {
      for(int i = 0; i < headYawHardnessActuator; ++i)
      {
        startAngles[i] = (*positionRequest)[5][i][0];
        startHardness[i] = (*hardnessRequest)[5][i][0];
      }
      lbhRatio = 0.f;
      isSittingDown = true;
    }
    for(int i = 0; i < headYawHardnessActuator; ++i)
    {
      actuators[i] = startAngles[i] * (1.f - lbhRatio) + sitDownAngles[i] * lbhRatio;
      actuators[i + headYawHardnessActuator] = startHardness[i];
      if(actuators[i + headYawHardnessActuator] > 0.3f)
        actuators[i + headYawHardnessActuator] = 0.3f; // reduced hardness
    }
  }
  else
    isSittingDown = false;
}

void standUp(float lastRatio, float* destActuators, float* actuators)
{
  static bool isStandingUp = false;
  if(lbhRatio < lastRatio && lbhRatio > 0.f)
  {
    static float startAngles[headYawHardnessActuator];
    if(!isStandingUp)
    {
      if(lastRatio == 1.f)
        for(int i = 0; i < headYawHardnessActuator; ++i)
          startAngles[i] = *(sensorPtrs[i * 3]);
      else
        for(int i = 0; i < headYawHardnessActuator; ++i)
          startAngles[i] = (*positionRequest)[5][i][0];
      lbhRatio = 1.f;
      isStandingUp = true;
    }
    for(int i = 0; i < headYawHardnessActuator; ++i)
    {
      actuators[i] = destActuators[i] * (1.f - lbhRatio) + startAngles[i] * lbhRatio;
      float h = destActuators[i + headYawHardnessActuator];
      if(h > 0.5f)
        h = 0.5f;
      actuators[i + headYawHardnessActuator] = destActuators[i + headYawHardnessActuator] * (1.f - lbhRatio) + h * lbhRatio;
    }
  }
  else
    isStandingUp = false;
}

void onPreProcess()
{
  // set all actuator values according to the values in the shared memory block
  if(data != MAP_FAILED && proxy && positionRequest && hardnessRequest)
    try
    {
      dcmTime = (int)getSystemTime() + timeOffset;

      data->readingActuators = data->newestActuators;
      static int lastReadingActuators = -1;
      static int actuatorDrops = 0;
      if(data->readingActuators == lastReadingActuators)
      {
        if(actuatorDrops == 0)
          fprintf(stderr, "libbhuman: missed actuator request.\n");
        ++actuatorDrops;
      }
      else
        actuatorDrops = 0;
      lastReadingActuators = data->readingActuators;
      float* actuators = data->actuators[data->readingActuators];

      // update lbhRatio
      float lastRatio = lbhRatio;
      if(frameDrops > ALLOWED_FRAMEDROPS ||  // bhuman is not responding
         shuttingDown ||
         (lbhRatio >= 1.f && actuators[lHipPitchHardnessActuator] == 0.f && actuators[rHipPitchHardnessActuator] == 0.f)) // detect initial playDead motion
        lbhRatio += 0.005f;
      else
        lbhRatio -= 0.005f;
      if(lbhRatio < 0.f)
        lbhRatio = 0.f;
      else if(lbhRatio > 1.f)
        lbhRatio = 1.f;

      //
      if(lbhRatio > 0.f)
      {
        static float lbhControlledActuators[lbhNumOfActuatorIds];
        for(int i = 0; i < lbhNumOfActuatorIds; ++i)
          lbhControlledActuators[i] = 0.f;
        static bool wasActive = false;
        if(lbhRatio < 0.2f && actuators[lHipPitchHardnessActuator] > 0.11f)
          wasActive  = true;
        if(wasActive)
        {
          lbhControlledActuators[lShoulderPitchPositionActuator] = sitDownAngles[lShoulderPitchPositionActuator - headYawPositionActuator];
          lbhControlledActuators[rShoulderPitchPositionActuator] = sitDownAngles[rShoulderPitchPositionActuator - headYawPositionActuator];
          lbhControlledActuators[lShoulderPitchHardnessActuator] = 0.11f;
          lbhControlledActuators[rShoulderPitchHardnessActuator] = 0.11f;
          lbhControlledActuators[lHipPitchPositionActuator] = sitDownAngles[lHipPitchPositionActuator - headYawPositionActuator];
          lbhControlledActuators[rHipPitchPositionActuator] = sitDownAngles[rHipPitchPositionActuator - headYawPositionActuator];
          lbhControlledActuators[lHipPitchHardnessActuator] = 0.1f;
          lbhControlledActuators[rHipPitchHardnessActuator] = 0.1f;
        }
        sitDown(lastRatio, lbhControlledActuators);
        standUp(lastRatio, actuators, lbhControlledActuators);
        setEyeLeds(lbhControlledActuators);
        if(frameDrops == 0 && !shuttingDown)
          copyLeds(actuators, lbhControlledActuators);
        actuators = lbhControlledActuators;
      }

      setBatteryLeds(actuators);

      // set position actuators
      (*positionRequest)[4][0] = dcmTime; // 0 delay!
      for(int i = 0; i < lbhNumOfPositionActuatorIds; ++i)
        (*positionRequest)[5][i][0] = actuators[i];
      proxy->setAlias(*positionRequest);

      // set hardness actuators
      bool requestedHardness = false;
      for(int i = headYawHardnessActuator; i < headYawHardnessActuator + lbhNumOfHardnessActuatorIds; ++i)
        if(actuators[i] != requestedActuators[i])
        {
          (*hardnessRequest)[4][0] = dcmTime; // 0 delay!
          for(int i = 0; i < lbhNumOfHardnessActuatorIds; ++i)
            (*hardnessRequest)[5][i][0] = requestedActuators[headYawHardnessActuator + i] = actuators[headYawHardnessActuator + i];
          proxy->setAlias(*hardnessRequest);
          requestedHardness = true;
          break;
        }

      // set us actuator
      bool requestedUs = false;
      if(requestedActuators[usActuator] != actuators[usActuator])
      {
        requestedActuators[usActuator] = actuators[usActuator];
        if(actuators[usActuator] >= 0.f)
        {
          (*usRequest)[4][0] = dcmTime;
          (*usRequest)[5][0][0] = actuators[usActuator];
          proxy->setAlias(*usRequest);
          requestedUs = true;
        }
      }

      // set led
      if(!requestedHardness && !requestedUs)
        for(int i = 0; i < lbhNumOfLedActuatorIds; ++i)
        {
          int index = faceLedRedLeft0DegActuator + ledIndex;
          if(++ledIndex == lbhNumOfLedActuatorIds)
            ledIndex = 0;
          if(actuators[index] != requestedActuators[index])
          {
            (*ledRequest)[0] = std::string(actuatorNames[index]);
            (*ledRequest)[2][0][0] = requestedActuators[index] = actuators[index];
            (*ledRequest)[2][0][1] = dcmTime;
            proxy->set(*ledRequest);
            break;
          }
        }
    }
    catch(AL::ALError& e)
    {
      fprintf(stderr, "libbhuman: %s\n", e.toString().c_str());
    }
}

void onPostProcess()
{
  // get new sensor values and copy them to the shared memory block
  if(data != MAP_FAILED && memory && proxy)
    try
    {
      // copy sensor values into the shared memory block
      int writingSensors = 0;
      if(writingSensors == data->newestSensors)
        ++writingSensors;
      if(writingSensors == data->readingSensors)
        if(++writingSensors == data->newestSensors)
          ++writingSensors;
      assert(writingSensors != data->newestSensors);
      assert(writingSensors != data->readingSensors);
      float* sensors = data->sensors[writingSensors];
      for(int i = 0; i < lbhNumOfSensorIds; ++i)
        sensors[i] = *(sensorPtrs[i]);
      data->newestSensors = writingSensors;

      // detect shutdown request via chest-button
      static int startPressedTime = dcmTime;
      if(!*(sensorPtrs[chestButtonSensor]))
        startPressedTime = dcmTime;
      else if(startPressedTime && !shuttingDown && dcmTime - startPressedTime > 3000)
      {
        if(*(sensorPtrs[rBumperRightSensor]) || *(sensorPtrs[rBumperLeftSensor]) ||
           *(sensorPtrs[lBumperRightSensor]) || *(sensorPtrs[lBumperLeftSensor]))
          (void)!system("sudo shutdown -n -r now");
        else
          (void)!system("sudo shutdown -h now");
        shuttingDown = true;
      }
    }
    catch(AL::ALError& e)
    {
      fprintf(stderr, "libbhuman: %s\n", e.toString().c_str());
    }

  // raise the semaphore
  if(sem != SEM_FAILED)
  {
    int sval;
    if(sem_getvalue(sem, &sval) == 0)
    {
      if(sval < 1)
      {
        sem_post(sem);
        frameDrops = 0;
      }
      else
      {
        if(frameDrops == 0)
          fprintf(stderr, "libbhuman: dropped sensor data.\n");
        frameDrops++;
      }
    }
  }
}

void close()
{
  fprintf(stderr, "libbhuman: Stop.\n");

  // unmap the shared memory
  if(data != MAP_FAILED)
  {
    munmap(data, sizeof(LBHData));
    data = (LBHData*)MAP_FAILED;
  }

  // close shared memory
  if(fd != -1)
  {
    close(fd);
    fd = -1;
  }

  // close semaphore
  if(sem != SEM_FAILED)
  {
    sem_close(sem);
    sem = SEM_FAILED;
  }

  //
  if(proxy != 0)
  {
    delete proxy;
    proxy = 0;
  }
  if(positionRequest != 0)
  {
    delete positionRequest;
    positionRequest = 0;
  }
  if(hardnessRequest != 0)
  {
    delete hardnessRequest;
    hardnessRequest = 0;
  }
  if(usRequest != 0)
  {
    delete usRequest;
    usRequest = 0;
  }
  if(ledRequest != 0)
  {
    delete ledRequest;
    ledRequest = 0;
  }
  if(memory != 0)
  {
    delete memory;
    memory = 0;
  }

  fprintf(stderr, "libbhuman: Stopped.\n");
}

int create(boost::shared_ptr<AL::ALBroker> pBroker)
{
  fprintf(stderr, "libbhuman: Start.\n");

  assert(lbhNumOfSensorIds == sizeof(sensorNames) / sizeof(*sensorNames));
  assert(lbhNumOfActuatorIds == sizeof(actuatorNames) / sizeof(*actuatorNames));

  // created shared memory
  if((fd = shm_open(LBH_MEM_NAME, O_CREAT | O_RDWR, S_IRUSR | S_IWUSR)) == -1)
  {
    perror("libbhuman: shm_open");
    close();
    return 0;
  }

  if(ftruncate(fd, sizeof(LBHData)) == -1)
  {
    perror("libbhuman: ftruncate");
    close();
    return 0;
  }

  // map the shared memory
  if((data = (LBHData*)mmap(NULL, sizeof(LBHData), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0)) == MAP_FAILED)
  {
    perror("libbhuman: mmap");
    close();
    return 0;
  }
  memset(data, 0, sizeof(LBHData));
  memset(requestedActuators, 0, sizeof(requestedActuators));
  for(int i = faceLedRedLeft0DegActuator; i < faceLedRedLeft0DegActuator + lbhNumOfLedActuatorIds; ++i)
    requestedActuators[i] = -1.f;

  // open semaphore
  if((sem = sem_open(LBH_SEM_NAME, O_CREAT | O_RDWR, S_IRUSR | S_IWUSR, 0)) == SEM_FAILED)
  {
    perror("libbhuman: sem_open");
    close();
    return 0;
  }

  try
  {
    // get the robot name
    memory = new AL::ALMemoryProxy(pBroker);
    {
      std::string robotName = (std::string)memory->getData("Device/DeviceList/ChestBoard/BodyNickName", 0);
      int len = std::min<>(robotName.length(), sizeof(data->robotName) - 1);
      memcpy(data->robotName, robotName.c_str(), len);
      data->robotName[len] = '\0';
    }

    // create "positionRequest" and "hardnessRequest" alias
    proxy = new AL::DCMProxy(pBroker);
    {
      AL::ALValue params;
      AL::ALValue result;
      params.arraySetSize(1);
      params.arraySetSize(2);

      params[0] = std::string("positionActuators");
      params[1].arraySetSize(lbhNumOfPositionActuatorIds);
      for(int i = 0; i < lbhNumOfPositionActuatorIds; ++i)
        params[1][i] = std::string(actuatorNames[i]);
      result = proxy->createAlias(params);


      params[0] = std::string("hardnessActuators");
      params[1].arraySetSize(lbhNumOfHardnessActuatorIds);
      for(int i = 0; i < lbhNumOfHardnessActuatorIds; ++i)
        params[1][i] = std::string(actuatorNames[headYawHardnessActuator + i]);
      result = proxy->createAlias(params);

      params[0] = std::string("usRequest");
      params[1].arraySetSize(1);
      params[1][0] = std::string(actuatorNames[usActuator]);
      result = proxy->createAlias(params);
    }

    // prepare "positionRequest" request
    positionRequest = new AL::ALValue;
    (*positionRequest).arraySetSize(6);
    (*positionRequest)[0] = std::string("positionActuators");
    (*positionRequest)[1] = std::string("ClearAll"); //"Merge";
    (*positionRequest)[2] = std::string("time-separate");
    (*positionRequest)[3] = 0;
    (*positionRequest)[4].arraySetSize(1);
    (*positionRequest)[5].arraySetSize(lbhNumOfPositionActuatorIds);
    for(int i = 0; i < lbhNumOfPositionActuatorIds; ++i)
      (*positionRequest)[5][i].arraySetSize(1);

    // prepare "hardnessRequest" request
    hardnessRequest = new AL::ALValue;
    (*hardnessRequest).arraySetSize(6);
    (*hardnessRequest)[0] = std::string("hardnessActuators");
    (*hardnessRequest)[1] = std::string("ClearAll"); //"Merge";
    (*hardnessRequest)[2] = std::string("time-separate");
    (*hardnessRequest)[3] = 0;
    (*hardnessRequest)[4].arraySetSize(1);
    (*hardnessRequest)[5].arraySetSize(lbhNumOfHardnessActuatorIds);
    for(int i = 0; i < lbhNumOfHardnessActuatorIds; ++i)
      (*hardnessRequest)[5][i].arraySetSize(1);

    usRequest = new AL::ALValue;
    (*usRequest).arraySetSize(6);
    (*usRequest)[0] = std::string("usRequest");
    (*usRequest)[1] = std::string("Merge"); // doesn't work with "ClearAll"
    (*usRequest)[2] = std::string("time-separate");
    (*usRequest)[3] = 0;
    (*usRequest)[4].arraySetSize(1);
    (*usRequest)[5].arraySetSize(1);
    (*usRequest)[5][0].arraySetSize(1);

    // prepare ledRequest
    ledRequest = new AL::ALValue;
    (*ledRequest).arraySetSize(3);
    //(*ledRequest)[0] = std::string("Face/Led/Green/Right/180Deg/Actuator/Value");
    (*ledRequest)[1] = std::string("ClearAll");
    (*ledRequest)[2].arraySetSize(1);
    (*ledRequest)[2][0].arraySetSize(2);
    //(*ledRequest)[2][0][0] = 1.0;
    (*ledRequest)[2][0][1] = 0;

    // prepare sensor pointers
    for(int i = 0; i < lbhNumOfSensorIds; ++i)
      if(!strcmp(sensorNames[i], ""))
        sensorPtrs[i] = &zeroFloat;
      else
        sensorPtrs[i] = (float*)memory->getDataPtr(sensorNames[i]);

    // determine time offset between libbhuman and naoqi time
    {
      int delta = 0;
      dcmTime = proxy->getTime(delta);
      timeOffset = dcmTime - getSystemTime();
    }

    // register "onPreProcess" and "onPostProcess" callbacks
    proxy->getGenericProxy()->getModule()->atPreProcess(&onPreProcess);
    proxy->getGenericProxy()->getModule()->atPostProcess(&onPostProcess);
  }
  catch(AL::ALError& e)
  {
    fprintf(stderr, "libbhuman: %s\n", e.toString().c_str());
    close();
    return 0;
  }

  fprintf(stderr, "libbhuman: Started!\n");
  return 0;
}


class BHuman : public AL::ALModule
{
public:

  BHuman(boost::shared_ptr<AL::ALBroker> pBroker, const std::string& pName): ALModule(pBroker, pName)
  {
    setModuleDescription("A module that provides basic ipc NaoQi DCM access using shared memory.");

    if(create(pBroker) != 0)
      throw ALERROR("BHuman", "constructor", "");
  }

  virtual ~BHuman()
  {
    close();
  }

  void dataChanged(const std::string& pDataName, const AL::ALValue& pValue, const std::string& pMessage) {}
  bool innerTest() { return true; }
};


extern "C" int _createModule(boost::shared_ptr<AL::ALBroker> pBroker)
{
  AL::ALModule::createModule<BHuman>(pBroker, "BHuman");
  return 0;
}

extern "C" int _closeModule() // NaoQi doesn't invoke this function
                              // thats why we register the "BHuman" module for using the constructor and destructor
{
  return 0;
}
