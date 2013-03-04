/*
 * @file FootPercept.h
 * Declaration of af
 * @author Tobias Kastner
 */

#pragma once

#include "Tools/Math/Vector2.h"
#include "Tools/Debugging/DebugDrawings.h"
#include <vector>
#include "Tools/ConvexHull.h"

class FootPercept : public Streamable
{
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(leftPart);
    STREAM(rightPart);
    STREAM(bodyPoints);
    STREAM(wasSeen);
    STREAM_REGISTER_FINISH;
  }

public:
  /**< Constructor */
  FootPercept() : wasSeen(false)
  {
    bodyPoints.reserve(20);
  };


  Vector2<> leftPart; /**< the most left part of the robots hull, projected on the ground */
  Vector2<> rightPart; /**<         right                                                 */

  std::vector<Vector2<> > bodyPoints; /**< convex hull of the observed points on the robots body/feet */
  bool wasSeen; /**< as the name implies */

  /**< draw cool stuff */
  void draw()
  {
    DECLARE_DEBUG_DRAWING("representation:FootPercept:convexHull", "drawingOnImage");
    DECLARE_DEBUG_DRAWING("representation:FootPercept:Field", "drawingOnField");

    if(!wasSeen)
      return;

    COMPLEX_DRAWING("representation:FootPercept:Field",
    {
      LINE("representation:FootPercept:Field", 0, 0, leftPart.x, leftPart.y, 20, Drawings::ps_solid, ColorClasses::black);
      LINE("representation:FootPercept:Field", 0, 0, rightPart.x, rightPart.y, 20, Drawings::ps_solid, ColorClasses::black);
      LINE("representation:FootPercept:Field", leftPart.x, leftPart.y, rightPart.x, rightPart.y, 20, Drawings::ps_solid, ColorClasses::black);
      DRAWTEXT("representation:FootPercept:Field", leftPart.x, leftPart.y, 20, ColorClasses::red, leftPart.abs());
      DRAWTEXT("representation:FootPercept:Field", rightPart.x, rightPart.y, 20, ColorClasses::red, rightPart.abs());
    });

    COMPLEX_DRAWING("representation:FootPercept:convexHull",
    {
      std::vector<Vector2<> > points = bodyPoints;
      ConvexHull::jarvisMarch(points);
      for(size_t i = 0; i < points.size(); ++i)
      {
        Vector2<> from = points[i];
        if(i + 1 == points.size())
        {
          LINE("representation:FootPercept:convexHull", from.x, from.y, points[0].x, points[0].y, 2, Drawings::ps_solid, ColorClasses::orange);
          break;
        }

        Vector2<> to = points[i + 1];
        LINE("representation:FootPercept:convexHull", from.x, from.y, to.x, to.y, 2, Drawings::ps_solid, ColorClasses::orange);
      }
    });
  }

  FootPercept& operator=(FootPercept const& other)
  {
    leftPart = other.leftPart;
    rightPart = other.rightPart;
    bodyPoints = other.bodyPoints;
    wasSeen = other.wasSeen;
    return *this;    
  }
};

class SimpleFootModel : public Streamable
{
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(leftPart);
    STREAM(rightPart);
    STREAM(isValid);
    STREAM_REGISTER_FINISH;
  }

public:
  SimpleFootModel(): isValid(false) {};

  Vector2<> leftPart;
  Vector2<> rightPart;
  bool isValid;

  void draw()
  {
    DECLARE_DEBUG_DRAWING("representation:SimpleFootModel", "drawingOnField");
    if(isValid)
    {
      COMPLEX_DRAWING("representation:SimpleFootModel",
      {
        LINE("representation:SimpleFootModel", 0, 0, leftPart.x, leftPart.y, 20, Drawings::ps_solid, ColorClasses::yellow);
        LINE("representation:SimpleFootModel", 0, 0, rightPart.x, rightPart.y, 20, Drawings::ps_solid, ColorClasses::yellow);
        LINE("representation:SimpleFootModel", leftPart.x, leftPart.y, rightPart.x, rightPart.y, 20, Drawings::ps_solid, ColorClasses::yellow);
      });
    }
  }
};
