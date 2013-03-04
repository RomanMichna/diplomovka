/**
* @file Representations/Debugging/StateMachineBehaviorInfo.h
* Declaration of class StateMachineBehaviorInfo
* @author Colin Graf
*/

#ifndef StateMachineBehaviorInfo_H
#define StateMachineBehaviorInfo_H

#include <vector>
#include <string>

class InMessage;

/**
* @class StateMachineBehaviorInfo 
* A class to represent information about a StateMachineBehavior behavior.
*/
class StateMachineBehaviorInfo
{
public:
  
  class Option
  {
  public:
    std::string name;
    std::vector<std::string> states;
  };

  class ActiveOption
  {
  public:
    unsigned short option;
    unsigned char depth;
    unsigned short state;
    unsigned int stateTime;
    unsigned int optionTime;
  };

  unsigned timeStamp;
  std::vector<Option> options;
  std::vector<ActiveOption> activeOptions;

  /**
  * The function handles a StateMachineBehavior debug messages.
  * @param message The message.
  * @return Was is actually a StateMachineBehavior message?
  */
  bool handleMessage(InMessage& message);
};

#endif // StateMachineBehaviorInfo_H
