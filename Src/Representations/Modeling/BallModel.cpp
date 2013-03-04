/**
* @file BallModel.cpp
* Implementation the BallModel's drawing functions BallModel
*/

#include "BallModel.h"
#include "Tools/Math/Pose2D.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"

void BallModel::draw()
{
  // drawing of the ball model in the field view
  DECLARE_DEBUG_DRAWING("representation:BallModel", "drawingOnField",
    const Vector2<>& position(estimate.position);
    const Vector2<>& velocity(estimate.velocity);
    CIRCLE("representation:BallModel",
            position.x,
            position.y,
            45,
            0, // pen width
            Drawings::ps_solid,
            ColorClasses::orange,
            Drawings::bs_solid,
            ColorClasses::orange);
    ARROW("representation:BallModel", position.x, position.y,
          position.x + velocity.x, position.y + velocity.y, 5, 1, ColorClasses::orange);
  );
}

void BallModel::draw3D(const Pose2D& robotPose) const 
{
  // drawing og the ball model in the scene
  DECLARE_DEBUG_DRAWING3D("representation:BallModel", "field",
    Vector2<> ballRelToWorld = robotPose * estimate.position;
    SPHERE3D("representation:BallModel", ballRelToWorld.x, ballRelToWorld.y, 35.f, 35.f, ColorClasses::orange);
    LINE3D("representation:BallModel", robotPose.translation.x, robotPose.translation.y, 1.f, ballRelToWorld.x, ballRelToWorld.y, 1.f, 5.f, ColorClasses::orange);
  );
}

void BallModel::drawEndPosition(float ballFriction) const
{
  // drawing of the end position
  DECLARE_DEBUG_DRAWING("representation:BallModel:endPosition", "drawingOnField",
    Vector2<> position = estimate.getEndPosition(ballFriction);
    CIRCLE("representation:BallModel:endPosition",
            position.x,
            position.y,
            45,
            0, // pen width
            Drawings::ps_solid,
            ColorClasses::black,
            Drawings::bs_solid,
            ColorRGBA(168, 25, 99, 220));
  );
}

void GroundTruthBallModel::draw()
{
  DECLARE_DEBUG_DRAWING("representation:GroundTruthBallModel", "drawingOnField",
    const Vector2<>& position(estimate.position);
    const Vector2<>& velocity(estimate.velocity);
    CIRCLE("representation:GroundTruthBallModel",
            position.x,
            position.y,
            45,
            0, // pen width
            Drawings::ps_solid,
            ColorRGBA(255, 128, 0, 192),
            Drawings::bs_solid,
            ColorRGBA(255, 128, 0, 192));
    ARROW("representation:GroundTruthBallModel", position.x, position.y,
          position.x + velocity.x, position.y + velocity.y, 5, 1, ColorRGBA(255, 128, 0, 192));
  );
}
