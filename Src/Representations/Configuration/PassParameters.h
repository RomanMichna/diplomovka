/*
* @author Tobias Kastner
*/

#pragma once

#include <string>
#include <vector>
#include "Tools/Streams/Streamable.h"
#include "Tools/Math/Vector2.h"

class PassParameters : public Streamable
{
public:

  PassParameters() : bx(160), by(-62), kickToX(50.f), kickToY(40.f), kickToZ(230.f),
    strikeOutX(-120.f), strikeOutY(-30.f), strikeOutZ(-190.f), minStrikeOutX(-30.f),
    passOffsetX(-170.f), passOffsetY(60.f), degreeOfPolynom(2) {}

  float bx;
  float by;
  float kickToX;
  float kickToY;
  float kickToZ;
  float strikeOutX;
  float strikeOutY;
  float strikeOutZ;
  float minStrikeOutX;
  float passOffsetX;
  float passOffsetY;
  int degreeOfPolynom;
  std::vector<float> co;
  std::vector<Vector2<> > measurements;


private:
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(bx);
    STREAM(by);
    STREAM(kickToX);
    STREAM(kickToY);
    STREAM(kickToZ);
    STREAM(strikeOutX);
    STREAM(strikeOutY);
    STREAM(strikeOutZ);
    STREAM(minStrikeOutX);
    STREAM(passOffsetX);
    STREAM(passOffsetY);
    STREAM(degreeOfPolynom);
    STREAM(co);
    STREAM(measurements);
    STREAM_REGISTER_FINISH;
  }
};
