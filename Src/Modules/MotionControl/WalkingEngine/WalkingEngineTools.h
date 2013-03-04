/**
* @file AsymmetricWalkingEngineTools.h
* Declaration of tools utilized by the AsymmetricWalkingEngine
* @author Colin Graf
*/

#include "Tools/Streams/Streamable.h"

class VectorYZ : public Streamable
{
public:
  float y;
  float z;

  VectorYZ() : y(0.f), z(0.f) {}

  VectorYZ(float y, float z) : y(y), z(z) {}

private:
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(y);
    STREAM(z);
    STREAM_REGISTER_FINISH;
  }
};

/**
* An one-dimensional downhill simplex optimizer
*/
class FunctionMinimizer
{
public:
  virtual float func(float pos) const = 0;
  float minimize(float minPos, float maxPos, float startPos, float startPosDelta, float minVal, bool& clipped, const char* debugstr) const;
};
