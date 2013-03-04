/**
 * @file RobotLocatorUKF.cpp
 * The implementation of the RobotLocatorUKF with a Kalman filter.
 * @author <a href="mailto:afabisch@tzi.de">Alexander Fabisch</a>
 */
#include "RobotLocatorUKF.h"
#include "Tools/Debugging/Asserts.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/Stopwatch.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Matrix2x2.h"
#include "Tools/Math/Vector3.h"
#include "Tools/Math/Common.h"
#include "Tools/Math/Covariance.h"
#include <limits>
#include <algorithm>

RobotLocatorUKF::Prediction RobotLocatorUKF::g;
RobotLocatorUKF::Measurement RobotLocatorUKF::h;

void RobotLocatorUKF::init()
{
  InConfigMap stream("robotLocatorUKF.cfg");
  ASSERT(stream.exists());
  stream >> params;
  lastState = STATE_INITIAL;

  robots.reserve(2 * params.maxRobots);
  unitedRobots.reserve(2 * params.maxRobots);
}

void RobotLocatorUKF::update(RobotsModel& robotsModel)
{
  DEBUG_RESPONSE("module:RobotLocatorUKF:clear", robots.clear(););
  MODIFY("parameters:RobotLocatorUKF", params);
  DECLARE_DEBUG_DRAWING("module:RobotLocatorUKF:projectOnImage", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:RobotLocatorUKF:vectorToPost", "drawingOnField");

  // hacked (RC2011)
  if(lastState == STATE_SET && theGameInfo.state == STATE_PLAYING)
  {
    robots.clear();
  }
  lastState = theGameInfo.state;

  motionUpdate();
  sensorUpdate();
  markRobotsToDelete();
  uniteNearRobots();
  cycle(robotsModel);
}

void RobotLocatorUKF::motionUpdate()
{
  Pose2D odometryOffset(theOdometryData - lastOdometryData);
  lastOdometryData = theOdometryData;

  for(RLI r(robots.begin()); r != robots.end(); r++)
  {
    // prediction noise covariance matrix
    const float distance = r->kf.mean.abs();
    const float noiseFactor = params.relativeOdometryRotationsNoiseStdDev * abs(theOdometryData.rotation);
    const float odometryNoiseStdDevY = distance * sinf(noiseFactor) / cosf(noiseFactor);
    const float odometryNoiseStdDevX = odometryNoiseStdDevY > distance ?
                                       1.0f : distance - sqrtf(distance * distance - odometryNoiseStdDevY * odometryNoiseStdDevY);
    Matrix2x2<> R = Covariance::create(params.motionNoiseStdDev) // observed robot
                    + Covariance::create(odometryNoiseStdDevX, odometryNoiseStdDevY, r->kf.mean.angle()); // observing robot
    // Prediction step: Currently there is actually no motion model of the other robots.
    // So their assumed motion is 0, but we add significant noise.
    Vector3<> control(odometryOffset.translation.x, odometryOffset.translation.y, odometryOffset.rotation);
    r->kf.prediction(control, R);
  }
}

void RobotLocatorUKF::sensorUpdate()
{
  for(RobotPercept::RCIt r(theRobotPercept.robots.begin()); r != theRobotPercept.robots.end(); r++)
    updateMostLikelyOldRobot(*r);
  for(RobotPercept::RCIt r(theRobotPercept.unlocalizableRobots.begin()); r != theRobotPercept.unlocalizableRobots.end(); r++)
    updateRobotInDirection(*r);
}

void RobotLocatorUKF::updateMostLikelyOldRobot(const RobotPercept::Robot& newRobot)
{
  Vector2<> relPosOnField;

  if(!Geometry::calculatePointOnField(theImageCoordinateSystem.toCorrected(newRobot.lowestPx), theCameraMatrix, theCameraInfo, relPosOnField))
    return;

  float shortestDistance(std::numeric_limits<float>::max());
  RLI robot;

  for(RLI r = robots.begin(); r != robots.end(); r++)
  {
    if(r->teamRed == newRobot.teamRed && Geometry::distance(r->kf.mean, relPosOnField) < params.maxAllowedDistance)
    {
      float distance = sqrt(Covariance::squaredMahalanobisDistance(r->kf.mean, r->kf.covariance, relPosOnField));
      if(distance < shortestDistance)
      {
        robot = r;
        shortestDistance = distance;
      }
    }
  }

  if(shortestDistance == std::numeric_limits<float>::max())
  {
    const Matrix2x2<> initialCovariance = Covariance::create(params.initialStdDev, relPosOnField.angle());
    Robot initializedRobot(relPosOnField, initialCovariance, newRobot.standing, newRobot.teamRed, theFrameInfo.time);
    ASSERT(initializedRobot.state == Robot::NEW);
    robots.push_back(initializedRobot);
  }
  else if(robot->notUpdatedYet())
  {
    ASSERT(robot->kf.covariance.det() != 0.0f);
    // Measurement update:
    const Matrix2x2<> Q = Covariance::create(params.sensorNoiseStdDev);
    Vector2<> measurement = theImageCoordinateSystem.toCorrected(newRobot.lowestPx);
    MID_DOT("module:RobotLocatorUKF:projectOnImage", measurement.x, measurement.y, ColorClasses::white, ColorClasses::black);
    robot->kf.update(measurement, Q, theCameraMatrix, theCameraInfo, theImageCoordinateSystem);
    robot->standing = newRobot.standing;
    robot->cyclesSinceLastUpdate = 0;
    robot->timeStamp = theFrameInfo.time;
    robot->state = Robot::UPDATED;
    robot->confirmations++;
  }
}

void RobotLocatorUKF::updateRobotInDirection(const RobotPercept::Robot& newRobot)
{
  Vector2<> relPosOnField;
  if(!Geometry::calculatePointOnField(theImageCoordinateSystem.toCorrected(newRobot.lowestPx), theCameraMatrix, theCameraInfo, relPosOnField))
    return;
  const float angle = relPosOnField.angle();
  for(RLI robot(robots.begin()); robot != robots.end(); robot++)
  {
    if(robot->notUpdatedYet() && robot->teamRed == newRobot.teamRed
       && abs(robot->kf.mean.angle() - angle) < params.maxAngleDist
       && !shouldBeVisible(theCameraMatrix, theCameraInfo, theImage, theImageCoordinateSystem, robot->kf.mean))
    {
      ASSERT(robot->kf.covariance.det() != 0.0);
      // No measurement update:
      const Matrix2x2<> Q = Covariance::create(params.sensorNoiseStdDev);
      Vector2<> measurement = h(robot->kf.mean, theCameraMatrix, theCameraInfo, theImageCoordinateSystem);
      robot->kf.update(measurement, Q, theCameraMatrix, theCameraInfo, theImageCoordinateSystem);
      robot->cyclesSinceLastUpdate = 0;
      robot->timeStamp = theFrameInfo.time;
      robot->state = Robot::UPDATED;
      robot->confirmations++;
    }
  }
}

void RobotLocatorUKF::markRobotsToDelete()
{
  for(RLI r(robots.begin()); r != robots.end(); r++)
  {
    if(r->kf.approximateProbabilityAtMeanArea(params.robotWidth) < params.minMeanProbability)
    {
      r->state = Robot::DELETED;
    }
    else if(r->notUpdatedYet())
    {
      if(!theFieldDimensions.isInsideCarpet(Geometry::relative2FieldCoord(theRobotPose, r->kf.mean)))
      {
        r->state = Robot::DELETED;
      }
      else if(shouldBeVisible(theCameraMatrix, theCameraInfo, theImage, theImageCoordinateSystem, r->kf.mean))
      {
        r->kf.covariance += Covariance::create(params.noSensorNoiseStdDev, r->kf.mean.angle());
        r->state = Robot::UPDATED;
      }
    }
    else
    {
      const Vector2<> absPosOnField = Geometry::relative2FieldCoord(theRobotPose, r->kf.mean);
      const Vector2<> leftGoalPostOnField(
        (float) theFieldDimensions.xPosOpponentGroundline,
        (float) theFieldDimensions.yPosLeftGoal);
      LINE("module:RobotLocatorUKF:vectorToPost",
           absPosOnField.x, absPosOnField.y, leftGoalPostOnField.x, leftGoalPostOnField.y,
           20, Drawings::ps_solid, ColorClasses::red);
      if((leftGoalPostOnField - absPosOnField).abs() < params.minDistanceToOpponentGoalPost)
      {
        r->state = Robot::DELETED;
        continue;
      }
      const Vector2<> rightGoalPostOnField(
        (float) theFieldDimensions.xPosOpponentGroundline,
        (float) theFieldDimensions.yPosRightGoal);
      LINE("module:RobotLocatorUKF:vectorToPost",
           absPosOnField.x, absPosOnField.y, rightGoalPostOnField.x, rightGoalPostOnField.y,
           20, Drawings::ps_solid, ColorClasses::red);
      if((rightGoalPostOnField - absPosOnField).abs() < params.minDistanceToOpponentGoalPost)
        r->state = Robot::DELETED;
    }
  }
}

void RobotLocatorUKF::uniteNearRobots()
{
  unitedRobots.clear();
  const size_t robotsSize = robots.size();
  for(size_t i = 1; i < robotsSize; i++) for(size_t j = 0; j < i; j++)
    {
      if(!(robotIsDeleted(robots[i]) || robotIsDeleted(robots[j]))
         && robots[i].teamRed == robots[j].teamRed)
      {
        if(Geometry::distance(robots[i].kf.mean, robots[j].kf.mean) <= params.maxAllowedDistance)
        {
          // optimal fusion of two measurements
          const Matrix2x2<> covarianceFactor = (robots[i].kf.covariance + robots[j].kf.covariance).invert();
          const Vector2<> mean = robots[i].kf.covariance * covarianceFactor * robots[i].kf.mean
                                 + robots[j].kf.covariance * covarianceFactor * robots[j].kf.mean;
          const Matrix2x2<> covariance = (robots[i].kf.covariance.invert() + robots[j].kf.covariance.invert()).invert();

          const bool standing = robots[i] < robots[j] ? robots[j].standing : robots[i].standing;
          const unsigned timeStamp = std::min(robots[i].timeStamp, robots[j].timeStamp);
          Robot unitedRobot(mean, covariance, standing, robots[i].teamRed, timeStamp);
          unitedRobot.cyclesSinceLastUpdate = std::min(robots[i].cyclesSinceLastUpdate, robots[j].cyclesSinceLastUpdate);
          unitedRobot.state = Robot::UPDATED;
          unitedRobot.confirmations = robots[i].confirmations + robots[j].confirmations;
          unitedRobots.push_back(unitedRobot);

          robots[i].state = robots[j].state = Robot::DELETED;
        }
      }
    }
  robots.insert(robots.end(), unitedRobots.begin(), unitedRobots.end());
}

void RobotLocatorUKF::cycle(RobotsModel& robotsModel)
{
  COMPLEX_DRAWING("module:RobotLocatorUKF:projectOnImage",
  {
    for(RLI robot = robots.begin(); robot != robots.end(); robot++)
    {
      Vector2<int> pointInImage;
      const bool valid = Geometry::calculatePointInImage(
        Vector2<int>((int) robot->kf.mean.x, (int) robot->kf.mean.y),
        theCameraMatrix, theCameraInfo, pointInImage);
      if(!valid) continue;
      Vector2<> rawPointInImage = theImageCoordinateSystem.fromCorrectedApprox(pointInImage);
      if(!Geometry::isPointInsideRectangle(
        Vector2<>(0.0f, 0.0f),
        Vector2<>((float)(theCameraInfo.resolutionWidth - 1),
      (float)(theCameraInfo.resolutionHeight - 1)),
        rawPointInImage))
        continue;

      Matrix2x2<> L = Covariance::choleskyDecomposition(robot->kf.covariance);
      std::vector<Vector2<> > sigmaPoints(5);
      sigmaPoints[0] = h(robot->kf.mean, theCameraMatrix, theCameraInfo, theImageCoordinateSystem);
      for(int i = 0; i < 2; i++)
      {
        sigmaPoints[i + 1] = h(robot->kf.mean + L[i], theCameraMatrix, theCameraInfo, theImageCoordinateSystem);
        sigmaPoints[i + 3] = h(robot->kf.mean - L[i], theCameraMatrix, theCameraInfo, theImageCoordinateSystem);
      }
      for(int i = 0; i < 5; i++)
      {
        MID_DOT("module:RobotLocatorUKF:projectOnImage", sigmaPoints[i].x, sigmaPoints[i].y,
                ColorClasses::orange, ColorClasses::red);
      }
      Vector2<> meanH = (sigmaPoints[0] + sigmaPoints[1] + sigmaPoints[2] + sigmaPoints[3] + sigmaPoints[4]) / 5.0f;
      MID_DOT("module:RobotLocatorUKF:projectOnImage", meanH.x, meanH.y, ColorClasses::blue, ColorClasses::yellow);

      Matrix2x2<> covH;
      for(int i = 0; i < 5; i++)
      {
        Vector2<> meanDiff = sigmaPoints[i] - meanH;
        Matrix2x2<> add;
        add[0][0] = meanDiff[0] * meanDiff[0];
        add[1][1] = meanDiff[1] * meanDiff[1];
        add[0][1] = add[1][0] = meanDiff[0] * meanDiff[1];
        covH += add;
      }
      covH /= 2.0f;
      float axis1 = 0.0f, axis2 = 0.0f, angle = 0.0f;
      Covariance::errorEllipse(covH, axis1, axis2, angle);
      ELLIPSE("module:RobotLocatorUKF:projectOnImage", meanH, axis1, axis2, angle,
              10, Drawings::ps_solid, ColorRGBA(255, 100, 100, 100), Drawings::bs_solid, ColorRGBA(255, 100, 100, 100));
    }
  });

  for(RLI robot = robots.begin(); robot != robots.end(); robot++)
  {
    robot->cycle();
  }

  RLI newEnd = remove_if(robots.begin(), robots.end(), robotIsDeleted);
  const size_t newSize = newEnd - robots.begin();
  ASSERT_WITHIN(newSize, 0, robots.size());
  robots.resize(newSize);

  if(robots.size() > params.maxRobots)
  {
    sort(robots.rbegin(), robots.rend()); // descending order with respect to the probability
    robots.resize(params.maxRobots);
  }

  robotsModel.robots.clear();
  robotsModel.robots.reserve(params.maxRobots);
  for(RLI r = robots.begin(); r != robots.end(); r++)
  {
    if(r->confirmations > 2)
    {
      robotsModel.robots.push_back(RobotsModel::Robot(r->kf.mean, r->teamRed, r->standing, r->kf.covariance, r->timeStamp));
    }
  }
  ASSERT(robotsModel.robots.size() <= params.maxRobots);
}

bool RobotLocatorUKF::shouldBeVisible(
  const CameraMatrix& theCameraMatrix,
  const CameraInfo& theCameraInfo,
  const Image& theImage,
  const ImageCoordinateSystem& theImageCoordinateSystem,
  const Vector2<>& relPosOnField)
{
  static const float TOLERANCE = 2.0f;
  Vector2<int> pointInImage;
  const bool valid = Geometry::calculatePointInImage(
                       Vector2<int>((int) relPosOnField.x, (int) relPosOnField.y),
                       theCameraMatrix, theCameraInfo, pointInImage);
  if(!valid) return false;
  Vector2<> rawPointInImage = theImageCoordinateSystem.fromCorrectedApprox(pointInImage);
  CROSS("module:RobotLocator:projection", (int) rawPointInImage.x, (int) rawPointInImage.y,
        2, 2, Drawings::ps_solid, ColorClasses::red);
  const bool isInImage = Geometry::isPointInsideRectangle(
                           Vector2<>(-TOLERANCE,
                                     (float)(theCameraInfo.resolutionHeight / 2) - TOLERANCE),  // The team marker should be visible, too.
                           Vector2<>((float)(theCameraInfo.resolutionWidth - 1) + TOLERANCE,
                                     (float)(theCameraInfo.resolutionHeight - 1) + TOLERANCE),
                           rawPointInImage);
  return isInImage;
}

MAKE_MODULE(RobotLocatorUKF, Modeling)

