#pragma once

#include "Tools/Streams/Streamable.h"

class GroundTruthResult : public Streamable
{
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM_REGISTER_FINISH;
  }

  // TODO store your results here
};
