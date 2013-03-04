/**
 * @file USRequest.h
 * This file declares a class that represents a request for controlling
 * which sonar is fired next.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Tools/Streams/Streamable.h"

/**
 * @class USRequest
 * A class that represents a request for controlling
 * which sonar is fired next.
 */
class USRequest : public Streamable
{
private:
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(sendMode);
    STREAM(receiveMode);
    STREAM_REGISTER_FINISH;
  }

public:
  int sendMode; /**< The firing mode for sending. -1 -> don't send. */
  int receiveMode; /**< The firing mode assumed for received readings. -1 -> ignore. */

  /** Default constructor. */
  USRequest() : sendMode(-1), receiveMode(-1) {}
};
