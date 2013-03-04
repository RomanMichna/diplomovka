/**
* @file Representations/BehaviorControl/BehaviorLEDRequest.h
* This file contains the BehaviorLEDRequest class.
* @author jeff
*/

#pragma once

#include "Representations/Infrastructure/LEDRequest.h"
#include "Tools/Streams/Streamable.h"
#include "Tools/Enum.h"

class BehaviorLEDRequest : public Streamable
{
private:
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(modifiers, LEDRequest);
    STREAM(leftEyeColor);
    STREAM(rightEyeColor);
    STREAM_REGISTER_FINISH;
  }

public:
  ENUM(BehaviorLED,
    leftEye,
    rightEye,
    leftEar,
    rightEar
  );
  
  ENUM(EyeColor,
    default_color,
    red,
    green,
    blue,
    white,
    magenta,
    yellow
  );

  LEDRequest::LEDState modifiers[numOfBehaviorLEDs];
  EyeColor leftEyeColor, rightEyeColor;

  BehaviorLEDRequest()
  {
    for(int i = 0; i < numOfBehaviorLEDs; ++i)
      modifiers[i] = LEDRequest::on;

    leftEyeColor = default_color;
    rightEyeColor = default_color;
  }

  bool operator==(const BehaviorLEDRequest& other) const
  {
    for(int i = 0; i < numOfBehaviorLEDs; i++)
      if(modifiers[i] != other.modifiers[i])
        return false;
    return true;
  }

  bool operator!=(const BehaviorLEDRequest& other) const
  {
    return !(*this == other);
  }
};

