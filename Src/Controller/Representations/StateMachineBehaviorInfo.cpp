/**
* @file Representations/Debugging/StateMachineBehaviorInfo.cpp
* Implementation of class StateMachineBehaviorInfo
* @author Colin Graf
*/

#include "StateMachineBehaviorInfo.h"
#include "Platform/SystemCall.h"
//#include "Platform/BHAssert.h"

#include "Tools/MessageQueue/InMessage.h"

bool StateMachineBehaviorInfo::handleMessage(InMessage& message)
{
  switch(message.getMessageID())
  {
  case idStateMachineBehaviorDebugSymbols:
    {
      int count;

      // read options
      message.bin >> count;
      options.resize(count);
      for(int i = 0; i < count; ++i)
      {
        Option& option = options[i];
        std::vector<std::string>& states = option.states;
        int stateCount;
        message.bin >> option.name >> stateCount;
        states.resize(stateCount);
        for(int j = 0; j < stateCount; ++j)
          message.bin >> states[j];
      }

      timeStamp = SystemCall::getCurrentSystemTime();
      return true;
    }
  case idStateMachineBehaviorDebugMessage:
    {
      if(options.empty())
        return false; // idStateMachineBehaviorDebugSymbols required first

      int count;
      message.bin >> count;
      activeOptions.resize(count);
      for(int i = 0; i < count; ++i)
      {
        ActiveOption& activeOption = activeOptions[i];
        message.bin >> activeOption.option >> activeOption.depth >> activeOption.state >> activeOption.optionTime >> activeOption.stateTime;
        /*
        ASSERT(activeOption.option < options.size());
#ifndef NDEBUG
        Option& option = options[activeOption.option];
#endif
        ASSERT(activeOption.state < option.states.size());
        */
      }

      timeStamp = SystemCall::getCurrentSystemTime();
      return true;
    }
  default:
    return false;
  }
}
