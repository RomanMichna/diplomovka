/**
* @file StateMachineBehaviorEngine.cpp
* Implementation of a C++ powered state machine behavior engine module
* @author Colin Graf
*/

#include "StateMachineBehaviorEngine.h"
#include "Tools/Debugging/ReleaseOptions.h"
#include "Tools/Team.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Settings.h"

MAKE_MODULE(StateMachineBehaviorEngine, Behavior Control)

StateMachineBehaviorEngine::StateMachineBehaviorEngine() :
  theMotionRequest(behaviorControlOutput.motionRequest),
  theSoundRequest(behaviorControlOutput.soundRequest),
  theHeadMotionRequest(behaviorControlOutput.headMotionRequest),
  theGameInfo(behaviorControlOutput.gameInfo),
  theRobotInfo(behaviorControlOutput.robotInfo),
  theOwnTeamInfo(behaviorControlOutput.ownTeamInfo),
  theBehaviorData(behaviorControlOutput.behaviorData)
{
  agent.agent = "Root"; // default agent
}

void StateMachineBehaviorEngine::init()
{
  // load agent from config file
  InConfigMap stream(Global::getSettings().expandLocationFilename("agent.cfg"));
  if(stream.exists())
    stream >> agent;
  else
  {
    InConfigMap stream("agent.cfg");
    if(stream.exists())
      stream >> agent;
  }
}

void StateMachineBehaviorEngine::update(BehaviorControlOutput& behaviorControlOutput)
{
  EXECUTE_ONLY_IN_DEBUG(theBallModel.draw3D(theRobotPose););

  MODIFY("agent", agent.agent);
  if(agent.agent != currentAgent)
  {
    if(!stateMachineBehavior.init(*this, agent.agent))
    {
      OUTPUT_ERROR("Behavior: Could not find agent " << agent.agent);
    }
    else
    {
      OUTPUT(idText, text, "Behavior: Started agent " << agent.agent);
    }
    currentAgent = agent.agent;
  }

  this->behaviorControlOutput.ownTeamInfo = StateMachineBehaviorEngineBase::theOwnTeamInfo;
  this->behaviorControlOutput.robotInfo = StateMachineBehaviorEngineBase::theRobotInfo;
  this->behaviorControlOutput.gameInfo = StateMachineBehaviorEngineBase::theGameInfo;

  stateMachineBehavior.update(theFrameInfo.time);

  if(this->behaviorControlOutput.motionRequest.motion == MotionRequest::walk &&
     !this->behaviorControlOutput.motionRequest.walkRequest.isValid())
  {
#ifndef NDEBUG    
    {
      OutConfigMap stream("walkRequest.log");
      stream << this->behaviorControlOutput.motionRequest.walkRequest;
    }
#endif
    ASSERT(false);
    this->behaviorControlOutput.motionRequest = behaviorControlOutput.motionRequest;
  }
  
  behaviorControlOutput = this->behaviorControlOutput;
  behaviorControlOutput.behaviorData.teamColor = theOwnTeamInfo.teamColor == TEAM_BLUE ? BehaviorData::blue : BehaviorData::red;
  TEAM_OUTPUT_FAST(idTeamMateBehaviorData, bin, behaviorControlOutput.behaviorData);

  DEBUG_RESPONSE("automated requests:StateMachineBehaviorEngine:debugSymbols", sendDebugSymbols(););
  DEBUG_RESPONSE("automated requests:StateMachineBehaviorEngine:debugMessages", sendDebugMessages(););
}

void StateMachineBehaviorEngine::update(MotionRequest& motionRequest)
{
  motionRequest = theBehaviorControlOutput.motionRequest;
  if(Global::getReleaseOptions().motionRequest)
  {
    TEAM_OUTPUT_FAST(idMotionRequest, bin, motionRequest);
  }
}

void StateMachineBehaviorEngine::sendDebugSymbols()
{
  OutMessage& outMessage = Global::getDebugOut();
  OutBinaryMessage& out = outMessage.bin;
  std::vector<StateMachineBehavior::OptionInfo> options;
  stateMachineBehavior.getOptions(options);
  out << static_cast<unsigned int>(options.size());
  if(options.size() > 0)
    for(const StateMachineBehavior::OptionInfo* option = &options[0], * end = option + options.size(); option < end; ++option)
    {
      out << option->name << static_cast<unsigned int>(option->states.size());
      if(option->states.size() > 0)
        for(const std::string* state = &option->states[0], * end = state + option->states.size(); state < end; ++state)
          out << *state;
    }
  outMessage.finishMessage(idStateMachineBehaviorDebugSymbols);
}


void StateMachineBehaviorEngine::update(BehaviorDebugOutput& debugOutput)
{
  debugOutput.optionDefinitions.clear();
  debugOutput.activeOptions.clear();
  stateMachineBehavior.getOptions(debugOutput.optionDefinitions);
  stateMachineBehavior.getActiveOptions(debugOutput.activeOptions);
}

void StateMachineBehaviorEngine::sendDebugMessages()
{
  OutMessage& outMessage = Global::getDebugOut();
  OutBinaryMessage& out = outMessage.bin;
  activeOptions.resize(0);
  activeOptions.reserve(32);
  stateMachineBehavior.getActiveOptions(activeOptions);
  out << static_cast<unsigned int>(activeOptions.size());
  if(activeOptions.size() > 0)
    for(const StateMachineBehavior::ActiveOptionInfo* activeOption = &activeOptions[0], * end = activeOption + activeOptions.size(); activeOption < end; ++activeOption)
      out << activeOption->option << activeOption->depth << activeOption->state << activeOption->optionTime << activeOption->stateTime;
  outMessage.finishMessage(idStateMachineBehaviorDebugMessage);
}
