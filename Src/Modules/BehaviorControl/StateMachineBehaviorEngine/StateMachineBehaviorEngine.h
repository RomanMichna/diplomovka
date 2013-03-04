/**
* @file StateMachineBehaviorEngine.h
* Declaration of a C++ state machine behavior engine module
* @author Colin Graf
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Platform/SoundPlayer.h"

#include "Representations/BehaviorControl/BehaviorControlOutput.h"
#include "Representations/BehaviorControl/BehaviorLEDRequest.h"
#include "Representations/BehaviorControl/BehaviorDebugOutput.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/CombinedWorldModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/GlobalFieldCoverage.h"
#include "Representations/Modeling/FieldCoverage.h"
#include "Representations/Modeling/SideConfidence.h"
#include "Representations/Infrastructure/SoundRequest.h"
#include "Representations/Infrastructure/KeyStates.h"
#include "Representations/Infrastructure/TeamMateData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/MotionControl/HeadJointRequest.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/FreePartOfOpponentGoalModel.h"
#include "Representations/Perception/FootPercept.h"
#include "Representations/Perception/GoalPercept.h"
#include "Representations/Perception/LinePercept.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Modeling/ArmContactModel.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/HeadLimits.h"
#include "Representations/Configuration/PassParameters.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Representations/Modeling/RobotsModel.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Modeling/FootContactModel.h"
#include "StateMachineBehavior.h"

MODULE(StateMachineBehaviorEngine)
  REQUIRES(ArmContactModel)
  REQUIRES(BallModel)
  REQUIRES(CombinedWorldModel)
  REQUIRES(FallDownState)
  REQUIRES(FrameInfo)
  REQUIRES(FreePartOfOpponentGoalModel)
  REQUIRES(FieldDimensions)
  REQUIRES(FieldCoverage)
  REQUIRES(GameInfo)
  REQUIRES(GlobalFieldCoverage)
  REQUIRES(GoalPercept)
  REQUIRES(HeadJointRequest)
  REQUIRES(HeadLimits)
  REQUIRES(KeyStates)
  REQUIRES(LinePercept)
  REQUIRES(MotionInfo)
  REQUIRES(PassParameters)
  REQUIRES(RobotPose)
  REQUIRES(RobotInfo)
  REQUIRES(SideConfidence)
  REQUIRES(SimpleFootModel)
  REQUIRES(TeamMateData)
  REQUIRES(TorsoMatrix) // required for bike kicks
  REQUIRES(ObstacleModel)
  REQUIRES(OwnTeamInfo)
  REQUIRES(RobotsModel)
  REQUIRES(CameraMatrix)
  REQUIRES(CameraInfo)
  REQUIRES(FilteredRobotPose)
  REQUIRES(RobotDimensions)
  REQUIRES(CameraCalibration)
  REQUIRES(RobotModel)
  REQUIRES(GroundContactState)
  REQUIRES(FootContactModel)
  PROVIDES_WITH_MODIFY(BehaviorControlOutput)
  REQUIRES(BehaviorControlOutput)
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(MotionRequest)
  PROVIDES_WITH_MODIFY(HeadMotionRequest)
  PROVIDES_WITH_MODIFY(SoundRequest)
  PROVIDES_WITH_MODIFY(BehaviorLEDRequest)
  PROVIDES_WITH_MODIFY(BehaviorDebugOutput)
END_MODULE


/**
* @class StateMachineBehaviorEngine
* A module to execute behavior declarations modeled as state machine
* @author Colin Graf
*/
class StateMachineBehaviorEngine : public StateMachineBehaviorEngineBase
{
public:
  /** Constructor. */
  StateMachineBehaviorEngine();

private:
  class Agent : public Streamable
  {
  public:
    std::string agent;

  private:
    /**
    * Makes the object streamable
    * @param in The stream from which the object is read
    * @param out The stream to which the object is written.
    */
    void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN;
      STREAM(agent);
      STREAM_REGISTER_FINISH;
    }
  };

  Agent agent;
  std::string currentAgent;

  StateMachineBehavior stateMachineBehavior;
  BehaviorControlOutput behaviorControlOutput; /**< the output generated by the behavior control */
  MotionRequest& theMotionRequest;
  SoundRequest& theSoundRequest;
  HeadMotionRequest& theHeadMotionRequest;
  GameInfo& theGameInfo;
  RobotInfo& theRobotInfo;
  OwnTeamInfo& theOwnTeamInfo;
  BehaviorData& theBehaviorData;

  void init();

  /** updates the behavior control output */
  void update(BehaviorControlOutput& behaviorControlOutput);

  /** updates the motion request by copying from behavior control output */
  void update(MotionRequest& motionRequest);

  void update(BehaviorDebugOutput& debugOutput);

  /** updates the head motion request by copying from behavior control output */
  void update(HeadMotionRequest& headMotionRequest) {headMotionRequest = theBehaviorControlOutput.headMotionRequest;}

  /** updates the sound request by copying from behavior control output */
  void update(SoundRequest& soundRequest) {soundRequest = theBehaviorControlOutput.soundRequest;}

  /** update the behavior ledRequest by cpoying from behavior control output */
  void update(BehaviorLEDRequest& behaviorLEDRequest) {behaviorLEDRequest = theBehaviorControlOutput.behaviorLEDRequest;}

  void sendDebugSymbols();
  void sendDebugMessages();
  std::vector<StateMachineBehavior::ActiveOptionInfo> activeOptions;

  friend class StateMachineBehavior;
};
