/**
 * @file DamageConfiguration.h
 * Provides data about disabling some functions because of hardware failures.
 *
 * @author Benjamin Markowsky
 */

#pragma once

#include <string>
#include "Tools/Streams/Streamable.h"

class DamageConfiguration : public Streamable
{
public:
  
  bool weakLeftLeg;
  bool weakRightLeg;
  bool USLDefect;
  bool USRDefect;

  DamageConfiguration() :
    weakLeftLeg(false), weakRightLeg(false), USLDefect(false), USRDefect(false)
  {}

  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(weakLeftLeg)
    STREAM(weakRightLeg)
    STREAM(USLDefect);
    STREAM(USRDefect);
    STREAM_REGISTER_FINISH;
  }
};
