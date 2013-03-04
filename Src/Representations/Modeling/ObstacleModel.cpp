/**
* @file ObstacleModel.cpp
* Implementation of class ObstacleModel
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
*/

#include "ObstacleModel.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Math/Pose3D.h"

void ObstacleModel::draw()
{
  DECLARE_DEBUG_DRAWING("representation:ObstacleModel", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:ObstacleModelReduced", "drawingOnField");

  COMPLEX_DRAWING("representation:ObstacleModel",
  {
    for(std::vector<Obstacle>::const_iterator it = obstacles.begin(), end = obstacles.end(); it != end; ++it)
    {
      const Obstacle& obstacle = *it;
      ColorClasses::Color color = obstacle.type == Obstacle::US ? ColorClasses::robotBlue : obstacle.type == Obstacle::ARM ? ColorClasses::yellow : obstacle.type == Obstacle::FOOT ? ColorClasses::green : ColorClasses::black;
      const Vector2<>& left = obstacle.leftCorner;
      Vector2<> right = obstacle.rightCorner;
      const float leftLen = left.abs();
      const float rightLen = right.abs();

      Vector2<> expanded = left;
      expanded.normalize(leftLen + 500.f);
      LINE("representation:ObstacleModel", left.x, left.y, expanded.x, expanded.y, 30, Drawings::ps_solid, color);
      expanded = right;
      expanded.normalize(rightLen + 500.f);
      LINE("representation:ObstacleModel", right.x, right.y, expanded.x, expanded.y, 30, Drawings::ps_solid, color);

      Vector2<> rightNorm = right;
      rightNorm.normalize();
      Vector2<> leftRotated = Vector2<>(rightNorm.x, -rightNorm.y) * left.x + Vector2<>(rightNorm.y, rightNorm.x) * left.y;
      float angle = atan2(leftRotated.y, leftRotated.x);
      if(angle < 0.f)
        angle += pi2;

      int segments = ((int) floor(angle / fromDegrees(30.f))) + 1;
      float segmentAngle = angle / (float) segments;
      Vector2<> newRight;
      for(int i = 1; i < segments; ++i)
      {
        newRight = right;
        newRight.rotate(segmentAngle);
        newRight.normalize(rightLen + (leftLen - rightLen) * i / (float) segments);
        LINE("representation:ObstacleModel", right.x, right.y, newRight.x, newRight.y, 30, Drawings::ps_solid, color);
        right = newRight;
      }
      LINE("representation:ObstacleModel", left.x, left.y, right.x, right.y, 30, Drawings::ps_solid, color);

      CROSS("representation:ObstacleModel", obstacle.center.x, obstacle.center.y, 100, 20, Drawings::ps_solid, ColorClasses::blue);
      CROSS("representation:ObstacleModel", obstacle.closestPoint.x, obstacle.closestPoint.y, 100, 20, Drawings::ps_solid, ColorClasses::red);
    }
  });

  COMPLEX_DRAWING("representation:ObstacleModelReduced",
  {
    for(std::vector<Obstacle>::const_iterator it = obstacles.begin(), end = obstacles.end(); it != end; ++it)
    {
      const Obstacle& obstacle = *it;
      ColorClasses::Color color = obstacle.type == Obstacle::US ? ColorClasses::robotBlue : obstacle.type == Obstacle::ARM ? ColorClasses::yellow : ColorClasses::black;
      const Vector2<>& left = obstacle.leftCorner;
      Vector2<> right = obstacle.rightCorner;
      const float leftLen = left.abs();
      const float rightLen = right.abs();

      Vector2<> expanded = left;
      expanded.normalize(leftLen + 500.f);
      LINE("representation:ObstacleModelReduced", left.x, left.y, expanded.x, expanded.y, 30, Drawings::ps_solid, color);
      expanded = right;
      expanded.normalize(rightLen + 500.f);
      LINE("representation:ObstacleModelReduced", right.x, right.y, expanded.x, expanded.y, 30, Drawings::ps_solid, color);

      Vector2<> rightNorm = right;
      rightNorm.normalize();
      Vector2<> leftRotated = Vector2<>(rightNorm.x, -rightNorm.y) * left.x + Vector2<>(rightNorm.y, rightNorm.x) * left.y;
      float angle = atan2(leftRotated.y, leftRotated.x);
      if(angle < 0.f)
        angle += pi2;

      int segments = ((int) floor(angle / fromDegrees(30.f))) + 1;
      float segmentAngle = angle / (float) segments;
      Vector2<> newRight;
      for(int i = 1; i < segments; ++i)
      {
        newRight = right;
        newRight.rotate(segmentAngle);
        newRight.normalize(rightLen + (leftLen - rightLen) * i / (float) segments);
        LINE("representation:ObstacleModelReduced", right.x, right.y, newRight.x, newRight.y, 30, Drawings::ps_solid, color);
        right = newRight;
      }
      LINE("representation:ObstacleModelReduced", left.x, left.y, right.x, right.y, 30, Drawings::ps_solid, color);
    }
  });
}

void ObstacleModel::draw3D(const Pose3D& torsoMatrix) const
{

#define LINE3D_REL_ODOMETRY_ORIGIN(id, torsoMatrix, point1, point2, size, color) \
  { \
    Pose3D torsoMatrixInv = torsoMatrix.invert(); \
    Vector3<> point1Rel = torsoMatrixInv * Vector3<>(point1.x, point1.y, 0.f); \
    Vector3<> point2Rel = torsoMatrixInv * Vector3<>(point2.x, point2.y, 0.f); \
    LINE3D(id, point1Rel.x, point1Rel.y, 0.f, point2Rel.x, point2Rel.y, 0.f, size, color); \
  }

  DECLARE_DEBUG_DRAWING3D("representation:ObstacleModel", "robot",
    for(std::vector<Obstacle>::const_iterator it = obstacles.begin(), end = obstacles.end(); it != end; ++it)
    {
      const Obstacle& obstacle = *it;
      ColorClasses::Color color = obstacle.type == Obstacle::US ? ColorClasses::robotBlue : obstacle.type == Obstacle::ARM ? ColorClasses::yellow : ColorClasses::black;
      const Vector2<>& left = obstacle.leftCorner;
      Vector2<> right = obstacle.rightCorner;
      const float leftLen = left.abs();
      const float rightLen = right.abs();

      Vector2<> expanded = left;
      expanded.normalize(leftLen + 500.f);
      LINE3D_REL_ODOMETRY_ORIGIN("representation:ObstacleModel", torsoMatrix, left, expanded, 4, color);
      expanded = right;
      expanded.normalize(rightLen + 500.f);
      LINE3D_REL_ODOMETRY_ORIGIN("representation:ObstacleModel", torsoMatrix, right, expanded, 4, color);

      Vector2<> rightNorm = right;
      rightNorm.normalize();
      Vector2<> leftRotated = Vector2<>(rightNorm.x, -rightNorm.y) * left.x + Vector2<>(rightNorm.y, rightNorm.x) * left.y;
      float angle = atan2(leftRotated.y, leftRotated.x);
      if(angle < 0.f)
        angle += pi2;

      int segments = ((int) floor(angle / fromDegrees(30.f))) + 1;
      float segmentAngle = angle / (float) segments;
      Vector2<> newRight;
      for(int i = 1; i < segments; ++i)
      {
        newRight = right;
        newRight.rotate(segmentAngle);
        newRight.normalize(rightLen + (leftLen - rightLen) * i / (float) segments);
        LINE3D_REL_ODOMETRY_ORIGIN("representation:ObstacleModel", torsoMatrix, right, newRight, 4, color);
        right = newRight;
      }
      LINE3D_REL_ODOMETRY_ORIGIN("representation:ObstacleModel", torsoMatrix, left, right, 4, color);
    }
  );
}
