#include "TeamMarkerSpots.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Common.h"

void TeamMarkerSpots::draw()
{
  DECLARE_DEBUG_DRAWING("representation:TeamMarkerSpots", "drawingOnImage");
  COMPLEX_DRAWING("representation:TeamMarkerSpots",
  {
    for(std::vector<TeamMarkerSpot>::iterator s = teamMarkers.begin(); s != teamMarkers.end(); s++)
    {
      // bounding box
      RECTANGLE("representation:TeamMarkerSpots", s->minX, s->minY, s->maxX, s->maxY,
                1, Drawings::ps_solid, ColorClasses::robotBlue);

      // center of gravity
      DOT("representation:TeamMarkerSpots",
          s->centerOfGravity.x, s->centerOfGravity.y,
          ColorClasses::black, ColorClasses::white);

      // principal axis
      LINE("representation:TeamMarkerSpots", s->left.x, s->left.y, s->right.x, s->right.y,
           1, Drawings::ps_solid, ColorClasses::red);
    }
  });
}

