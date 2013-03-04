/**
 * @file SoundRequest.h
 * Declaration of class SoundRequest.
 * @author Philippe Schober
 */

#pragma once

#include "Tools/Streams/Streamable.h"
#include "Tools/Enum.h"

class SoundRequest : public Streamable
{
private:
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(sound);
    STREAM_REGISTER_FINISH;
  }

public:
  ENUM(Sound,
    none,
    marioInjury,
    red,
    blue,
    penaltyShootout,
    penaltyStriker,
    ownKickoff,
    opponentKickoff,
    penaltyKeeper,

    penalized,
    notPenalized,

    jumpLeft,
    jumpRight,
    genuflect,
    wifi,
    exhausted,

    confused
  );

  Sound sound; /**< The requested sound to be played. */

  /**
  * Default constructor.
  */
  SoundRequest() : sound(none) {}
};

/**
* A dummy class so SoundControl is selectable.
*/
class SoundOutput : public Streamable
{
private:
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    char dummy(0);
    STREAM(dummy);
    STREAM_REGISTER_FINISH;
  }
};
