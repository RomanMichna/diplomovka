/**
 * @file FallDownStateDetector.cpp
 *
 * This file implements a module that provides information about the current state of the robot's body.
 *
 * @author <a href="mailto:maring@informatik.uni-bremen.de">Martin Ring</a>
 */

#include "FallDownStateDetector.h"
#include "Representations/Infrastructure/JointData.h"
#include "Platform/SoundPlayer.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Settings.h"
#include "Tools/Debugging/DebugDrawings.h"

FallDownStateDetector::FallDownStateDetector()
{
  InConfigMap stream(Global::getSettings().expandLocationFilename("FallDownStateDetector.cfg"));
  if(stream.exists())
    stream >> parameters;
  else
  {
    InConfigMap stream("fallDownStateDetector.cfg");
    ASSERT(stream.exists());
    stream >> parameters;
  }
  parameters.fallDownAngleX *= pi_180;
  parameters.fallDownAngleY *= pi_180;
  parameters.onGroundAngle  *= pi_180;
  parameters.staggeringAngleX *= pi_180;
  parameters.staggeringAngleY *= pi_180;

  lastFallDetected = (unsigned int) - parameters.fallTime;
}

void FallDownStateDetector::update(FallDownState& fallDownState)
{
  DECLARE_PLOT("module:FallDownStateDetector:accelerationAngleXZ");
  DECLARE_PLOT("module:FallDownStateDetector:accelerationAngleYZ");

  // Buffer data:
  buffers[accX].add(theFilteredSensorData.data[SensorData::accX]);
  buffers[accY].add(theFilteredSensorData.data[SensorData::accY]);
  buffers[accZ].add(theFilteredSensorData.data[SensorData::accZ]);

  // Compute average acceleration values and angles:
  float accXaverage(buffers[accX].getAverage());
  float accYaverage(buffers[accY].getAverage());
  float accZaverage(buffers[accZ].getAverage());
  float accelerationAngleXZ(atan2(accZaverage, accXaverage));
  float accelerationAngleYZ(atan2(accZaverage, accYaverage));
  MODIFY("module:FallDownStateDetector:accX",  accXaverage);
  MODIFY("module:FallDownStateDetector:accY",  accYaverage);
  MODIFY("module:FallDownStateDetector:accZ",  accZaverage);
  MODIFY("module:FallDownStateDetector:accAngleXZ", accelerationAngleXZ);
  MODIFY("module:FallDownStateDetector:accAngleYZ", accelerationAngleYZ);
  PLOT("module:FallDownStateDetector:accelerationAngleXZ", accelerationAngleXZ);
  PLOT("module:FallDownStateDetector:accelerationAngleYZ", accelerationAngleYZ);

  fallDownState.odometryRotationOffset = 0;

  if(theMotionInfo.motion == MotionRequest::specialAction
     && theMotionInfo.specialActionRequest.specialAction == SpecialActionRequest::keeperJumpLeft)
  {
    if(theMotionInfo.specialActionRequest.mirror)
    {
      keeperJumped = KeeperJumpedRight;
    }
    else
    {
      keeperJumped = KeeperJumpedLeft;
    }
  }

  if(isCalibrated() && !specialSpecialAction())
  {
    if(theFrameInfo.getTimeSince(lastFallDetected) <= parameters.fallTime)// && !impact(fallDownState))
    {
      fallDownState.state = FallDownState::falling;
    }
    else if((abs(theFilteredSensorData.data[SensorData::angleX]) <= parameters.staggeringAngleX - pi_180
             && abs(theFilteredSensorData.data[SensorData::angleY]) <= parameters.staggeringAngleY - pi_180)
            || (fallDownState.state == FallDownState::upright && !isStaggering()))
    {
      fallDownState.state = FallDownState::upright;
      fallDownState.direction = FallDownState::none;
      fallDownState.sidewards = FallDownState::noot;
    }
    else if(fallDownState.state == FallDownState::staggering && isFalling())
    {
      //SoundPlayer::play("doh.wav");
      lastFallDetected = theFrameInfo.time;
      fallDownState.state = FallDownState::falling;
      fallDownState.direction = directionOf(theFilteredSensorData.data[SensorData::angleX], theFilteredSensorData.data[SensorData::angleY]);
      if(fallDownState.sidewards != FallDownState::fallen)
      {
        fallDownState.sidewards = sidewardsOf(fallDownState.direction);
      }
    }
    else if((isUprightOrStaggering(fallDownState)
             && isStaggering())
            || (fallDownState.state == FallDownState::staggering
                && abs(theFilteredSensorData.data[SensorData::angleX]) <= parameters.staggeringAngleX - pi_180
                && abs(theFilteredSensorData.data[SensorData::angleY]) <= parameters.staggeringAngleY - pi_180))
    {
      fallDownState.state = FallDownState::staggering;
      fallDownState.direction = directionOf(theFilteredSensorData.data[SensorData::angleX], theFilteredSensorData.data[SensorData::angleY]);
      if(fallDownState.sidewards != FallDownState::fallen)
      {
        fallDownState.sidewards = sidewardsOf(fallDownState.direction);
      }
    }
    else
    {
      fallDownState.state = FallDownState::undefined;

      if(!isGettingUp())
      {
        if(abs(accelerationAngleXZ) < 0.5f)
        {
          fallDownState.state = FallDownState::onGround;
          fallDownState.direction = FallDownState::front;
          if(fallDownState.sidewards == FallDownState::leftwards)
          {
            fallDownState.odometryRotationOffset = -pi_2;
            fallDownState.sidewards = FallDownState::fallen;
            keeperJumped = None; // keeper jumped has been detected and must not be handled explicitly
          }
          else if(fallDownState.sidewards == FallDownState::rightwards)
          {
            fallDownState.odometryRotationOffset = pi_2;
            fallDownState.sidewards = FallDownState::fallen;
            keeperJumped = None; // keeper jumped has been detected and must not be handled explicitly
          }
        }
        else if(abs(accelerationAngleXZ) > 2.5f)
        {
          fallDownState.state = FallDownState::onGround;
          fallDownState.direction = FallDownState::back;
          if(fallDownState.sidewards == FallDownState::leftwards)
          {
            fallDownState.odometryRotationOffset = pi_2;
            fallDownState.sidewards = FallDownState::fallen;
            keeperJumped = None; // keeper jumped has been detected and must not be handled explicitly
          }
          else if(fallDownState.sidewards == FallDownState::rightwards)
          {
            fallDownState.odometryRotationOffset = -pi_2;
            fallDownState.sidewards = FallDownState::fallen;
            keeperJumped = None; // keeper jumped has been detected and must not be handled explicitly
          }
        }
        else if(abs(accelerationAngleYZ) < 0.5f)
        {
          fallDownState.state = FallDownState::onGround;
          fallDownState.direction = FallDownState::left;
          if(fallDownState.sidewards != FallDownState::fallen)
          {
            fallDownState.sidewards = FallDownState::leftwards;
          }
        }
        else if(abs(accelerationAngleYZ) > 2.5f)
        {
          fallDownState.state = FallDownState::onGround;
          fallDownState.direction = FallDownState::right;
          if(fallDownState.sidewards != FallDownState::fallen)
          {
            fallDownState.sidewards = FallDownState::rightwards;
          }
        }
      }
      else // isGettingUp()
      {
        if(keeperJumped != None)
        {
          if(theMotionInfo.specialActionRequest.specialAction == SpecialActionRequest::standUpFrontNao)
          {
            if(keeperJumped == KeeperJumpedLeft)
            {
              fallDownState.odometryRotationOffset = pi_2;
            }
            else
            {
              fallDownState.odometryRotationOffset = -pi_2;
            }
          }
          else
          {
            // standUpBack
            if(keeperJumped == KeeperJumpedLeft)
            {
              fallDownState.odometryRotationOffset = -pi_2;
            }
            else
            {
              fallDownState.odometryRotationOffset = pi_2;
            }
          }
          keeperJumped = None;
        }
      }
    }
  }
  else
  {
    fallDownState.state = FallDownState::undefined;
  }
}

bool FallDownStateDetector::isUprightOrStaggering(FallDownState& fallDownState)
{
  return fallDownState.state == FallDownState::upright
         || fallDownState.state == FallDownState::staggering;
}

bool FallDownStateDetector::isGettingUp()
{
  return theMotionInfo.motion == MotionRequest::specialAction
         && (theMotionInfo.specialActionRequest.specialAction == SpecialActionRequest::standUpFrontNao
             || theMotionInfo.specialActionRequest.specialAction == SpecialActionRequest::standUpBackNao);
}

bool FallDownStateDetector::specialSpecialAction()
{
  return (theMotionInfo.motion == MotionRequest::specialAction
          && (theMotionInfo.specialActionRequest.specialAction == SpecialActionRequest::playDead ||
              theMotionInfo.specialActionRequest.specialAction == SpecialActionRequest::keeperJumpLeft));
}

bool FallDownStateDetector::isStaggering()
{
  return abs(theFilteredSensorData.data[SensorData::angleX]) >= parameters.staggeringAngleX + pi_180
         || abs(theFilteredSensorData.data[SensorData::angleY]) >= parameters.staggeringAngleY + pi_180;
}

bool FallDownStateDetector::isFalling()
{
  return abs(theFilteredSensorData.data[SensorData::angleX]) >= parameters.fallDownAngleX
         || abs(theFilteredSensorData.data[SensorData::angleY]) >= parameters.fallDownAngleY;
}

bool FallDownStateDetector::isCalibrated()
{
  return theInertiaSensorData.calibrated;
}

bool FallDownStateDetector::impact(FallDownState& fallDownState)
{
  switch(fallDownState.direction)
  {
  case FallDownState::back:
    return theFilteredSensorData.accX < -1.15f;
  case FallDownState::front:
    return theFilteredSensorData.accX > 1.15f;
  case FallDownState::left:
    return theFilteredSensorData.accY < -1.15f;
  case FallDownState::right:
    return theFilteredSensorData.accY < -1.15f;
  default:
    return false;
  }
}

FallDownState::Direction FallDownStateDetector::directionOf(float angleX, float angleY)
{
  if(abs(angleX) > abs(angleY) + 0.2f)
  {
    if(angleX > 0.f) return FallDownState::left;
    else return FallDownState::right;
  }
  else
  {
    if(angleY > 0.f) return FallDownState::front;
    else return FallDownState::back;
  }
}

FallDownState::Sidestate FallDownStateDetector::sidewardsOf(FallDownState::Direction dir)
{
  switch(dir)
  {
  case FallDownState::left:
    return FallDownState::leftwards;
  case FallDownState::right:
    return FallDownState::rightwards;
  default:
    return FallDownState::noot;
  }
}

MAKE_MODULE(FallDownStateDetector, Sensing)
