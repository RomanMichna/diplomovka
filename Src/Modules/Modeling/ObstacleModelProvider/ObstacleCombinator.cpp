/**
* @file ObstacleCombinator.cpp
*
* This file implements a module that merges information from the ultrasonic obstacle grid
* and perceptions from vision.
*
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
* @author Nico Lehmann
*/

#include "ObstacleCombinator.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Settings.h"
#include "Tools/Math/Vector3.h"
#include "Tools/Math/Approx.h"
#include "Platform/SoundPlayer.h"
#include <cfloat>


ObstacleCombinator::ObstacleCombinator()
{
  InConfigMap stream(Global::getSettings().expandLocationFilename("obstacleCombinator.cfg"));
  if(stream.exists())
    stream >> parameters;
}

void ObstacleCombinator::update(ObstacleModel& obstacleModel)
{
  MODIFY("parameters:ObstacleCombinator", parameters);

  obstacleModel.obstacles.clear();
  generateCellCluster(theUSObstacleGrid, ObstacleModel::Obstacle::US, obstacleModel);
  addFootObstacles(obstacleModel);
  addArmObstacles(obstacleModel);

  // Robot dimensions:
  const float robotHeight = 580;
  const float robotWidth  = 300; // Guessed value
//  const float robotDepth  = 150; // Guessed value
  const int   robotSize   = 9;   // Guessed value

  // Add robot obstacles:
  const float maxRobotDistanceSqr = (float) sqr(parameters.maxRobotDistance);
  for(RobotsModel::RCIt robot = theRobotsModel.robots.begin(); robot != theRobotsModel.robots.end(); robot++)
  {
    const float robotDistanceSqr = robot->relPosOnField.squareAbs();
    if(((!robot->standing && parameters.considerLyingRobots)
        && robotDistanceSqr < maxRobotDistanceSqr && maxRobotDistanceSqr > 0.f) ||
        ((robot->standing && parameters.considerStandingRobots)
         && robotDistanceSqr < maxRobotDistanceSqr && maxRobotDistanceSqr > 0.f))
    {
      const float robotDistance = sqrt(robotDistanceSqr);
      const float robotDistanceInv = 1.f / robotDistance;
      const Vector2<> widthOffset((robot->standing ? robotWidth : robotHeight) * 0.5f, 0.0f);
      const Vector2<>& robotCenter = robot->relPosOnField;
      const Vector2<> robotCenterNorm = robotCenter * robotDistanceInv;
      const Vector2<> closestRobotPoint = robotCenter * (1.f - widthOffset.x * robotDistanceInv);
      Vector2<> vertDirLeft = robotCenterNorm * widthOffset.x;
      vertDirLeft.rotateLeft();
      const Vector2<> leftCorner = closestRobotPoint + vertDirLeft;
      const Vector2<> rightCorner = closestRobotPoint - vertDirLeft;
      obstacleModel.obstacles.push_back(ObstacleModel::Obstacle(leftCorner, rightCorner, robotCenter, closestRobotPoint,
                                        robotSize, robot->covariance, ObstacleModel::Obstacle::ROBOT));
    }
  }

  EXECUTE_ONLY_IN_DEBUG(obstacleModel.draw3D(theTorsoMatrix););
}

void ObstacleCombinator::generateObstacleFromCurrentCluster(std::vector<ObstacleModel::Obstacle>& obstacles)
{
  Vector2<> centerCells;
  Vector2<int> robotPosition(USObstacleGrid::GRID_LENGTH / 2, USObstacleGrid::GRID_LENGTH / 2);
  Vector2<int> closestPoint(USObstacleGrid::GRID_LENGTH, USObstacleGrid::GRID_LENGTH);
  int closestPointSqrDist(USObstacleGrid::GRID_SIZE * 2);
  for(unsigned int i = 0, end = (int) currentCluster.cells.size(); i < end; ++i)
  {
    const Vector2<int>& c = currentCluster.cells[i];
    centerCells.x += c.x;
    centerCells.y += c.y;
    int sqrDistToRobot = sqr(c.x - robotPosition.x) + sqr(c.y - robotPosition.y);
    if(sqrDistToRobot < closestPointSqrDist)
    {
      closestPoint = c;
      closestPointSqrDist = sqrDistToRobot;
    }
  }
  centerCells /= static_cast<float>(currentCluster.cells.size());

  Vector2<> cossin = centerCells - Vector2<>((float) robotPosition.x, (float) robotPosition.y);
  cossin.normalize();
  const float cosinus = cossin.x;
  const float sinus = -cossin.y ;
  float newX;
  float newY;

  //initializing of the rectangle
  float xMinRotated(FLT_MAX);
  float yMinRotated(FLT_MAX);
  float xMaxRotated(-FLT_MAX);
  float yMaxRotated(-FLT_MAX);

  float rightAngle = 10.0;
  float leftAngle = -10.0;
  Vector2<int> rightCorner;
  Vector2<int> leftCorner;

  for(unsigned int i = 0, end = (int) currentCluster.cells.size(); i < end; ++i)
  {
    const Vector2<int>& c = currentCluster.cells[i];
    Vector2<int> cr = c - robotPosition;
    newX = cosinus * cr.x - sinus * cr.y; // rotates each cell of the cluster
    newY = sinus * cr.x + cosinus * cr.y;
    //sets new values for rectangle
    if(newX < xMinRotated)
      xMinRotated = newX;
    if(newX > xMaxRotated)
      xMaxRotated = newX;
    if(newY < yMinRotated)
      yMinRotated = newY;
    if(newY > yMaxRotated)
      yMaxRotated = newY;

    const float angle = approxAtan2(newY, newX);
    if(angle < rightAngle)
    {
      rightAngle = angle;
      rightCorner = Vector2<int>(c.x, c.y);
    }
    if(angle > leftAngle)
    {
      leftAngle = angle;
      leftCorner = Vector2<int>(c.x, c.y);
    }
  }

  Vector2<> leftCornerWorld = theUSObstacleGrid.gridToWorld(Vector2<>((float) leftCorner.x, (float) leftCorner.y));
  Vector2<> rightCornerWorld = theUSObstacleGrid.gridToWorld(Vector2<>((float) rightCorner.x, (float) rightCorner.y));
  Vector2<> closestPointWorld = theUSObstacleGrid.gridToWorld(Vector2<>(closestPoint.x + 0.5f, closestPoint.y + 0.5f));
  Vector2<> centerWorld = theUSObstacleGrid.gridToWorld(centerCells);

  if(abs(normalize(leftAngle - rightAngle)) < fromDegrees(1.f))
  {
    // obstacle too sparse (may cause issues with floating point arithmetic)
    leftCornerWorld.rotate(fromDegrees(0.5f));
    rightCornerWorld.rotate(fromDegrees(-0.5f));
  }

  //expansion (length of x- and y-axis through the center point) and orientation (dependent on robot rotation) of the cluster
  Vector3<> covarianceEllipse(((xMaxRotated - xMinRotated) * USObstacleGrid::CELL_SIZE) * parameters.covarianceFactor, ((yMaxRotated - yMinRotated) * USObstacleGrid::CELL_SIZE) * parameters.covarianceFactor, approxAtan2(centerWorld.y, centerWorld.x));
  Matrix2x2<> covariance(covarianceEllipse.x, 0, 0, covarianceEllipse.y); // covariance is initialised with uncorrelated values (only expansion of cluster as variances)
  rotateMatrix(covariance, covarianceEllipse.z); // rotates the covariance so that it fits to the orientation and expansion of the cluster

  obstacles.push_back(ObstacleModel::Obstacle(leftCornerWorld, rightCornerWorld, centerWorld, closestPointWorld, static_cast<int>(currentCluster.cells.size()), covariance, ObstacleModel::Obstacle::US));
}

void ObstacleCombinator::rotateMatrix(Matrix2x2<>& matrix, const float angle)
{
  const float cosine = cos(angle);
  const float sine = sin(angle);
  const Matrix2x2<> rotationMatrix(cosine, -sine, sine, cosine);
  matrix = (rotationMatrix * matrix) * rotationMatrix.transpose();
}

void ObstacleCombinator::generateCellCluster(const USObstacleGrid& grid, ObstacleModel::Obstacle::Type type, ObstacleModel& obstacleModel)
{
  // Perform grid cell clustering and add clusters as obstacles:
  memset(clusterAssignment, 0, USObstacleGrid::GRID_SIZE * sizeof(int));
  currentCluster.init();
  int currentClusterIndex(1);
  for(int y = 1; y < USObstacleGrid::GRID_LENGTH - 1; ++y)
  {
    for(int x = 1; x < USObstacleGrid::GRID_LENGTH - 1; ++x)
    {
      const int currentCellIndex = y * USObstacleGrid::GRID_LENGTH + x;
      const USObstacleGrid::Cell& c = theUSObstacleGrid.cells[currentCellIndex];
      if(clusterAssignment[currentCellIndex] == 0 && c.state >= theUSObstacleGrid.cellOccupiedThreshold)
      {
        clusterAssignment[currentCellIndex] = currentClusterIndex;
        // iterative implementation of floodfill algorithm
        cellStack.push(Vector2<int>(x, y));
        while(!cellStack.empty())
        {
          int x = cellStack.top().x;
          int y = cellStack.top().y;
          cellStack.pop();
          currentCluster.cells.push_back(Vector2<int>(x, y));
          if(x == 0 || y == 0 || x == USObstacleGrid::GRID_LENGTH - 1 || y == USObstacleGrid::GRID_LENGTH - 1) //ignore border
            continue;
          // Test all eight neighbors (also stupid check for center cell)
          for(int y2 = y - 1; y2 <= y + 1; ++y2)
          {
            for(int x2 = x - 1; x2 <= x + 1; ++x2)
            {
              const int neighborCellIndex = y2 * USObstacleGrid::GRID_LENGTH + x2;
              const USObstacleGrid::Cell& cn = theUSObstacleGrid.cells[neighborCellIndex];
              if(clusterAssignment[neighborCellIndex] == 0 &&
                 ((cn.state >= theUSObstacleGrid.cellOccupiedThreshold && !parameters.clusterNonMaxThresholdCells) ||
                  (cn.state > 0 && parameters.clusterNonMaxThresholdCells)))
              {
                clusterAssignment[neighborCellIndex] = currentClusterIndex;
                cellStack.push(Vector2<int>(x2, y2));
              }
            }
          }
        }
        ++currentClusterIndex;
        if(currentCluster.cells.size() >= (unsigned int)(parameters.minClusterSize))
          generateObstacleFromCurrentCluster(obstacleModel.obstacles);
        currentCluster.init();
      }
    }
  }
}


// TODO add some parameters
void ObstacleCombinator::addFootObstacles(ObstacleModel& obstacleModel)
{
  if(theFrameInfo.getTimeSince(theFootContactModel.lastContactLeft) < 2000)
  {
    Vector2<float> leftCorner(100.f,200.f);
    Vector2<float> rightCorner(100.f,0.f);
    Vector2<float> center(100.f,100.f);
    Vector2<float> closestPoint(rightCorner);
    Matrix2x2<> covariance(400, 0, 0, 400);
    obstacleModel.obstacles.push_back(ObstacleModel::Obstacle(leftCorner, rightCorner, center, closestPoint, 
      9, covariance, ObstacleModel::Obstacle::FOOT));
  }
  if(theFrameInfo.getTimeSince(theFootContactModel.lastContactRight) < 2000)
  {
    Vector2<float> rightCorner(100.f,-200.f);
    Vector2<float> leftCorner(100.f,0.f);
    Vector2<float> center(100.f,-100.f);
    Vector2<float> closestPoint(leftCorner);
    Matrix2x2<> covariance(400, 0, 0, 400);
    obstacleModel.obstacles.push_back(ObstacleModel::Obstacle(leftCorner, rightCorner, center, closestPoint, 
      9, covariance, ObstacleModel::Obstacle::FOOT));
  }
}


void ObstacleCombinator::addArmObstacles(ObstacleModel& obstacleModel)
{
  if(theFrameInfo.getTimeSince(theArmContactModel.timeOfLastContactLeft) < 2000)
  {
    addArmObstacle(obstacleModel, true, theArmContactModel.pushDirectionLeft);
  }
  if(theFrameInfo.getTimeSince(theArmContactModel.timeOfLastContactRight) < 2000)
  {
    addArmObstacle(obstacleModel, false, theArmContactModel.pushDirectionRight);
  }
}


// This implementation is quite simplistic and deliberately ignores odometry => but the approach works
void ObstacleCombinator::addArmObstacle(ObstacleModel& obstacleModel, bool leftArm, 
  const ArmContactModel::PushDirection& pushDirection)
{
  // ignore pushed from "inside":
  if((leftArm && pushDirection == ArmContactModel::W) || (!leftArm && pushDirection == ArmContactModel::E))
    return;
  // enter obstacle before or behind me:
  float x(100.f);
  if(pushDirection == ArmContactModel::NE || pushDirection == ArmContactModel::N || pushDirection == ArmContactModel::NW)
    x = -100.f;
  Vector2<float> rightCorner(x, leftArm ? 0.f : -200.f);
  Vector2<float> leftCorner(x, leftArm ? 200.f : 0.f);
  Vector2<float> center(x, leftArm ? 100.f : -100.f);
  Vector2<float> closestPoint(leftArm ? rightCorner : leftCorner);
  Matrix2x2<> covariance(400, 0, 0, 400);
  obstacleModel.obstacles.push_back(ObstacleModel::Obstacle(leftCorner, rightCorner, center, closestPoint, 
    9, covariance, ObstacleModel::Obstacle::ARM));
}


MAKE_MODULE(ObstacleCombinator, Modeling)
