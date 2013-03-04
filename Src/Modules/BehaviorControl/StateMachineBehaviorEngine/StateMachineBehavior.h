
#pragma once

#include <string>
#include <vector>
#include "Tools/Streams/Streamable.h"


class StateMachineBehaviorEngine;
class StateMachineBehaviorData;

class StateMachineBehavior
{
public:
  class OptionInfo : public Streamable
  {
  public:
    std::string name;
    std::vector<std::string> states;

  private:
    /**
    * The method makes the object streamable.
    * @param in The stream from which the object is read.
    * @param out The stream to which the object is written.
    */
    virtual void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN;
      STREAM(name);
      STREAM(states);
      STREAM_REGISTER_FINISH;
    }

  };

  class ActiveOptionInfo : public Streamable
  {
  public:
    unsigned short option;
    unsigned char depth;
    unsigned short state;
    unsigned int optionTime;
    unsigned int stateTime;

  private:
    virtual void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN;
      STREAM(option);
      STREAM(depth);
      STREAM(state);
      STREAM(optionTime);
      STREAM(stateTime);
      STREAM_REGISTER_FINISH;
    }

  };

  StateMachineBehavior();
  ~StateMachineBehavior();

  bool init(StateMachineBehaviorEngine& engine, const std::string& agent);
  void update(unsigned int time);

  void getOptions(std::vector<OptionInfo>& options);
  void getActiveOptions(std::vector<ActiveOptionInfo>& options);

private:
  StateMachineBehaviorData* data;

  void lock();
  void unlock();
};
