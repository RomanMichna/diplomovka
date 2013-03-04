#include "DribbleTarget.h"
#include "Tools/Debugging/DebugDrawings.h"

void DribbleTarget::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:DribbleTarget", "drawingOnField");
  COMPLEX_DRAWING("representation:DribbleTarget",
  {
    //Vector2<> dir = nextTarget * Vector2<>(1000.f, 0);
    CROSS("representation:DribbleTarget", nextTarget.x, nextTarget.y, 70, 0, Drawings::ps_solid, ColorRGBA(0, 0xff, 0xff));
    //ARROW("representation:DribbleTarget", nextTarget.x, nextTarget.y, dir.x, dir.y, 0, Drawings::ps_solid, ColorRGBA(0, 0xff, 0xff));
  });
}
