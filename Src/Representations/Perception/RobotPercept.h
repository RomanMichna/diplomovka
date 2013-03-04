/**
* @file RobotPercept.h
* @author <a href="afabisch@tzi.de">Alexander Fabisch</a>
*/

#pragma once

#include "Tools/Math/Vector2.h"
#include "Tools/ColorClasses.h"
#include "Tools/Debugging/DebugDrawings.h"
#include <vector>

class RobotPercept : public Streamable
{
  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(robots);
    STREAM_REGISTER_FINISH;
  }
public:
  class Robot : public Streamable
  {
    void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN;
      STREAM(lowestPx);
      STREAM(teamRed);
      STREAM(standing);
      STREAM_REGISTER_FINISH;
    }
  public:
    /** The center of gravity of the team maker */
    Vector2<int> centerOfMarker;
    /** coordinates of the first pixel below the robot */
    Vector2<int> lowestPx;
    /** the team color of the seen robot (either TEAM_BLUE or TEAM_RED) */
    bool teamRed;
    /** whether the robot is upright or lying on the field */
    bool standing;

    /** Use this constructor only for streaming! */
    Robot() {}

    Robot(const Vector2<int>& centerOfMarker, const Vector2<int>& lowestPx, bool standing, ColorClasses::Color color)
      : centerOfMarker(centerOfMarker), lowestPx(lowestPx), teamRed(color == ColorClasses::red), standing(standing)
    {
    }
  };

  typedef std::vector<Robot>::const_iterator RCIt;
  typedef std::vector<Robot>::iterator RIt;

  std::vector<Robot> robots;
  std::vector<Robot> unlocalizableRobots;

  /** Draws the position of the robot on the ground. */
  void draw()
  {
    DECLARE_DEBUG_DRAWING("representation:RobotPercept:pixels", "drawingOnImage");
    COMPLEX_DRAWING("representation:RobotPercept:pixels",
    {
      for(RCIt r(robots.begin()); r != robots.end(); r++)
      {
        ColorClasses::Color color(r->teamRed ? ColorClasses::red : ColorClasses::robotBlue);
        MID_DOT("representation:RobotPercept:pixels", (int) r->lowestPx.x, (int) r->lowestPx.y, color, color);
      }
    });
  }
};
