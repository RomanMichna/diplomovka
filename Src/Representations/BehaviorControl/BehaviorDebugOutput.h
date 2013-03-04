/**
 * @author Arne Böckmann
 */

#pragma once

#include "Modules/BehaviorControl/StateMachineBehaviorEngine/StateMachineBehavior.h"
#include "Tools/Streams/Streamable.h"


/**
 * Contains data about the current SMBE option and state.
 */
class BehaviorDebugOutput : public Streamable
{

public:

  /**
   * Contains all possible options and their states.
   */
  std::vector<StateMachineBehavior::OptionInfo> optionDefinitions;

  /**
   * The currently active options
   */
  std::vector<StateMachineBehavior::ActiveOptionInfo> activeOptions;

private:

  virtual void serialize(In* in, Out* out){
    STREAM_REGISTER_BEGIN;
    STREAM(optionDefinitions);
    STREAM(activeOptions);
    STREAM_REGISTER_FINISH;
  }

};
