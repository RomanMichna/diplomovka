/**
* @file LoggerOutput.h
*
*  Created on: Jan 27, 2012
*      Author: arne
*/

#pragma once

#include "Tools/Streams/Streamable.h"

/**
* Dummy output for the logger Module. Right now it does nothing :)
*/
class CognitionLoggerOutput : public Streamable
{
  /**
  * The method makes the object streamable.
  * @param in The stream from which the object is read
  * @param out The stream to which the object is written
  */
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    char dummy = 42;
    STREAM(dummy);
    STREAM_REGISTER_FINISH;
  }
};
