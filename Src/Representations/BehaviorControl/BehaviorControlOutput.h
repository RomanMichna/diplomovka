/**
 * @file BehaviorControlOutput.h
 * Declaration of class BehaviorControlOutput
 *
 * @author Max Risler
 */

#pragma once

#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Infrastructure/SoundRequest.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Representations/BehaviorControl/BehaviorLEDRequest.h"

/**
 * A class collecting the output from the behavior control module
 */
class BehaviorControlOutput : public Streamable
{
public:
  MotionRequest motionRequest;
  HeadMotionRequest headMotionRequest;
  SoundRequest soundRequest;
  OwnTeamInfo ownTeamInfo;
  RobotInfo robotInfo;
  GameInfo gameInfo;
  BehaviorData behaviorData;
  BehaviorLEDRequest behaviorLEDRequest;

  BehaviorControlOutput() {}

private:
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(motionRequest);
    STREAM(headMotionRequest);
    STREAM(soundRequest);
    STREAM(robotInfo);
    STREAM(gameInfo);
    STREAM(ownTeamInfo);
    STREAM(behaviorData);
    STREAM(behaviorLEDRequest);
    STREAM_REGISTER_FINISH;
  }
};
