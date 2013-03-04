#include "SSLVisionData.h"
#include "Tools/Debugging/DebugDrawings.h"

void SSLVisionData::draw()
{
  DECLARE_DEBUG_DRAWING("representation:GroundTruth:image", "drawingOnField");

  COMPLEX_DRAWING("representation:GroundTruth:image",
  {
    if(recentData.size())
    {
      const SSLVisionFrame& lastFrame = recentData.top();

      for(size_t i = 0; i < lastFrame.yellowRobots.size(); i++)
      {
        const Pose2D& robot = lastFrame.yellowRobots[i];
        CIRCLE("representation:GroundTruth:image", robot.translation.x, robot.translation.y, 200, 0,
               Drawings::ps_solid, ColorClasses::yellow, Drawings::bs_solid, ColorClasses::yellow);
        const Vector2<> target = Vector2<>(200.0f, 0.0f).rotate(robot.rotation) + robot.translation;
        ARROW("representation:GroundTruth:image", robot.translation.x, robot.translation.y, target.x, target.y,
              0, Drawings::ps_solid, ColorClasses::yellow);
      }

      for(size_t i = 0; i < lastFrame.blueRobots.size(); i++)
      {
        const Pose2D& robot = lastFrame.blueRobots[i];
        CIRCLE("representation:GroundTruth:image", robot.translation.x, robot.translation.y, 200, 0,
               Drawings::ps_solid, ColorClasses::blue, Drawings::bs_solid, ColorClasses::blue);
        const Vector2<> target = Vector2<>(200.0f, 0.0f).rotate(robot.rotation) + robot.translation;
        ARROW("representation:GroundTruth:image", robot.translation.x, robot.translation.y, target.x, target.y,
              0, Drawings::ps_solid, ColorClasses::blue);
      }

      CIRCLE("representation:GroundTruth:image", lastFrame.ball.x, lastFrame.ball.y, 50, 0,
             Drawings::ps_solid, ColorClasses::orange, Drawings::bs_solid, ColorClasses::orange);
    }
  });
}
