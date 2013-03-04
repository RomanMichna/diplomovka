
#pragma once

#include "Tools/Math/Pose2D.h"

class DribbleTarget : public Streamable
{
public:
  Vector2<> nextTarget;
  Vector2<> finalTarget;
  unsigned int estimateTimeToReachPose;
  Vector2<> closestObstacle;

  DribbleTarget() {};

  void draw() const;

private:
  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(nextTarget);
    STREAM(finalTarget);
    STREAM(estimateTimeToReachPose);
    STREAM(closestObstacle);
    STREAM_REGISTER_FINISH;
  }
};
