/**
* @file TeamMarkerSpots.h
* @author <a href="afabisch@tzi.de">Alexander Fabisch</a>
*/

#pragma once

#include "Tools/Math/Vector2.h"
#include "Tools/ColorClasses.h"
#include <vector>

class TeamMarkerSpots : public Streamable
{
  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(teamMarkers);
    STREAM_REGISTER_FINISH;
  }
public:
  class TeamMarkerSpot : public Streamable
  {
    void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN;
      STREAM(color, ColorClasses);
      STREAM(centerOfGravity);
      STREAM(left);
      STREAM(right);
      STREAM(minX);
      STREAM(maxX);
      STREAM(minY);
      STREAM(maxY);
      STREAM(area);
      STREAM(standing);
      STREAM_REGISTER_FINISH;
    }

  public:
    ColorClasses::Color color;
    Vector2<int> centerOfGravity;
    Vector2<int> left;
    Vector2<int> right;
    int minX, maxX, minY, maxY;
    int area;
    bool standing;
  };

  std::vector<TeamMarkerSpot> teamMarkers;

  void draw();
};
