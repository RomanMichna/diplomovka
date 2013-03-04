/**
* @file KeyStates.h
*
* Declaration of class KeyStates
*/

#pragma once

#include "Tools/Streams/Streamable.h"
#include "Tools/Enum.h"

/**
* The class represents the states of the keys.
*/
class KeyStates : public Streamable
{
private:
  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(pressed);
    STREAM_REGISTER_FINISH;
  }

public:
  ENUM(Key,
    rightFootRight,
    rightFootLeft,
    leftFootRight,
    leftFootLeft,
    chest
  );

  bool pressed[numOfKeys];

  KeyStates()
  {
    for(int i = 0; i < numOfKeys; ++i)
    {
      pressed[i] = false;
    }
  }
};
