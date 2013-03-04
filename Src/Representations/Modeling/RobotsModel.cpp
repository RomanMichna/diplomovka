/**
* @file RobotsModel.cpp
* Debug drawings for the RobotsModel.
* @author <a href="mailto:afabisch@tzi.de">Alexander Fabisch</a>
*/

#include "RobotsModel.h"
#include "Platform/BHAssert.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Covariance.h"

void RobotsModel::draw()
{
  DECLARE_DEBUG_DRAWING("representation:RobotsModel:robots", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:RobotsModel:covariance", "drawingOnField");

  COMPLEX_DRAWING("representation:RobotsModel:covariance",
  {
    for(RIt r(robots.begin()); r != robots.end(); r++)
    {
      float s1 = 0.0, s2 = 0.0, theta = 0.0;
      Covariance::errorEllipse(r->covariance, s1, s2, theta);
      ELLIPSE("representation:RobotsModel:covariance", r->relPosOnField, sqrt(3.0f)*s1, sqrt(3.0f)*s2, theta,
              10, Drawings::ps_solid, ColorRGBA(100, 100, 255, 100), Drawings::bs_solid, ColorRGBA(100, 100, 255, 100));
      ELLIPSE("representation:RobotsModel:covariance", r->relPosOnField, sqrt(2.0f)*s1, sqrt(2.0f)*s2, theta,
              10, Drawings::ps_solid, ColorRGBA(150, 150, 100, 100), Drawings::bs_solid, ColorRGBA(150, 150, 100, 100));
      ELLIPSE("representation:RobotsModel:covariance", r->relPosOnField, s1, s2, theta,
              10, Drawings::ps_solid, ColorRGBA(255, 100, 100, 100), Drawings::bs_solid, ColorRGBA(255, 100, 100, 100));
    }
  });

  COMPLEX_DRAWING("representation:RobotsModel:robots",
  {
    for(RIt r(robots.begin()); r != robots.end(); r++)
    {
      ColorClasses::Color color(r->teamRed ? ColorClasses::red : ColorClasses::robotBlue);
      CROSS("representation:RobotsModel:robots", r->relPosOnField.x, r->relPosOnField.y,
            50, 20, Drawings::ps_solid, color);
    }
  });
}

