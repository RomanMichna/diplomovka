
#pragma once

#include "Tools/Math/Vector2.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Platform/BHAssert.h"


class BallAfterKickPose : public Streamable
{
  /** Streaming (with specifications) */
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM_COMPRESSED_POSITION(position);
    STREAM(timeWhenLastKickWasPerformed);
    if(in) { rot = position.angle(); }
    STREAM_AS_UCHAR(playerNumber);
    STREAM_REGISTER_FINISH;
  }

public:

  BallAfterKickPose() : timeWhenLastKickWasPerformed(0), rot(0.f), playerNumber(0) {};
  Vector2<> position;
  unsigned timeWhenLastKickWasPerformed;
  Vector2<> dev;
  float rot;
  int playerNumber;

  void draw()
  {
    DECLARE_DEBUG_DRAWING("representation:BallAfterKickPose", "drawingOnField"); // drawing of the ball model
    COMPLEX_DRAWING("representation:BallAfterKickPose",
    {
      Vector2<>& pos(position);
      ELLIPSE("representation:BallAfterKickPose",
      pos,
      dev.x,
      dev.y,
      rot + pi_2,
      3, // pen width
      Drawings::ps_solid,
      ColorRGBA(255, 100, 100, 100),
      Drawings::bs_solid,
      ColorRGBA(50, 200, 200, 100));
    });
  }
};

class PassTarget : public Streamable
{
public:
  PassTarget() : playerNumber(0), timeToReachPassPose(0) {}
  PassTarget(Vector2<> target, int playerNumber, unsigned timeToReachPassPose) : target(target), playerNumber(playerNumber), timeToReachPassPose(timeToReachPassPose) {}
  Vector2<> target;
  int playerNumber;
  unsigned timeToReachPassPose;

private:
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM_COMPRESSED_POSITION(target);
    STREAM_AS_UCHAR(playerNumber);
    STREAM(timeToReachPassPose);
    STREAM_REGISTER_FINISH;
  }
};
