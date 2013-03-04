/**
 * @file ExpRobotLocator.cpp
 * The implementation of the RobotLocator with a Kalman filter.
 * @author Colin Graf
 */

#include "ExpRobotLocator.h"
#include "Tools/Math/Approx.h"

MAKE_MODULE(ExpRobotLocator, Modeling)

void ExpRobotLocator::init()
{
  p.processDeviation = Vector3<>(5.f, 5.f, 0.1f);
  p.robotRotationDeviation = Vector2<>(0.02f, 0.06f);
  p.odometryDeviation = Pose2D(0.5f, 0.5f, 0.5f);
  p.heightDeviation = 50.f; // 50 mm for robots with 1m distance

  p.robotFeetRadius = 100.f;
  p.robotWaistbandRadius = 50.f;
  p.robotMaxHorizontalHeight = 150.f;
  p.robotDefaultHeight = 260.f;

  p.angleTolerance = fromDegrees(20.f);
  p.distanceTolerance = 500.f; // 500 mm for robots with 1m distance

  p.robotNotSeenTimeout = 200;
  p.robotNotUpdatedTimeout = 15000;

  averageHeight[0] = averageHeight[1] = p.robotDefaultHeight;
  averageHeightVar[0] = averageHeightVar[1] = sqr(800.f);
}

void ExpRobotLocator::update(RobotsModel& robotsModel)
{
  MODIFY("module:ExpRobotLocator:parameters", p);

  motionUpdate();
  sensorUpdate();
  removeDatedFilters();

  // generate model
  robotsModel.robots.clear();
  for(std::vector<Filter>::iterator i = filters.begin(), end = filters.end(); i != end; ++i)
  {
    Filter& filter = *i;
    robotsModel.robots.push_back(RobotsModel::Robot(filter.pos, filter.teamRed, filter.height > p.robotMaxHorizontalHeight, filter.posCov, filter.timestamp));
  }
}

void ExpRobotLocator::motionUpdate()
{
  Pose2D odometryOffset = theOdometryData - lastOdometryData;
  lastOdometryData = theOdometryData;

  float odometryCos = cos(odometryOffset.rotation);
  float odometrySin = sin(odometryOffset.rotation);
  float odometryRotationDeviation = odometryOffset.rotation * p.odometryDeviation.rotation;
  float odometryDeviationCos = cos(odometryRotationDeviation);
  float odometryDeviationSin = sin(odometryRotationDeviation);
  Vector2<> odometryTranslationDeviation(odometryOffset.translation.x * p.odometryDeviation.translation.x, odometryOffset.translation.y * p.odometryDeviation.translation.y);
  Vector2<> odometryTranslationCov(sqr(odometryTranslationDeviation.x), sqr(odometryTranslationDeviation.y));

  Matrix2x2<> odometryRotation(Vector2<>(odometryCos, -odometrySin), Vector2<>(odometrySin, odometryCos)); // a
  Matrix2x2<> odometryRotationTransposed = odometryRotation.transpose();
  Matrix2x2<> odometryRotationDeviationRotation(Vector2<>(odometryDeviationCos, -odometryDeviationSin), Vector2<>(odometryDeviationSin, odometryDeviationCos));
  Vector2<> odometryTranslation(-odometryOffset.translation.x, -odometryOffset.translation.y); // u

  Vector3<> processCov(sqr(p.processDeviation[0]), sqr(p.processDeviation[1]), sqr(p.processDeviation[2]));

  for(std::vector<Filter>::iterator i = filters.begin(), end = filters.end(); i != end; ++i)
  {
    Filter& filter = *i;

    filter.pos = odometryRotation * filter.pos + odometryTranslation;
    filter.posCov = odometryRotation * filter.posCov * odometryRotationTransposed;

    // add process noise
    filter.posCov[0][0] += processCov[0];
    filter.posCov[1][1] += processCov[1];
    filter.heightVar += processCov[2];

    // add uncertainty for odometry translation
    filter.posCov[0][0] += odometryTranslationCov[0];
    filter.posCov[1][1] += odometryTranslationCov[1];

    // add uncertainty for odometry rotation (curde approximation)
    Vector2<> odometryRotationDeviationTranslation = odometryRotationDeviationRotation * filter.pos - filter.pos;
    filter.posCov[0][0] += sqr(odometryRotationDeviationTranslation.x);
    filter.posCov[1][1] += sqr(odometryRotationDeviationTranslation.y);

    // compute angle and distance
    filter.angle = approxAtan2(filter.pos.y, filter.pos.x);
    filter.distance = filter.pos.abs();
  }

  averageHeightVar[0] += processCov[2];
  averageHeightVar[1] += processCov[2];
}

void ExpRobotLocator::sensorUpdate()
{
  // find matching filter for fully seen robots
  for(std::vector<RobotPercept::Robot>::const_iterator i = theRobotPercept.robots.begin(), end = theRobotPercept.robots.end(); i != end; ++i)
  {
    const RobotPercept::Robot& robotPercept = *i;

    // compute team marker height
    const Vector2<> correctedRobotInImage = theImageCoordinateSystem.toCorrected(robotPercept.lowestPx);
    const Vector2<> correctedTeamMarkerInImage = theImageCoordinateSystem.toCorrected(robotPercept.centerOfMarker);
    const Vector3<> cameraToRobot(theCameraInfo.focalLength, theCameraInfo.opticalCenter.x - correctedRobotInImage.x, theCameraInfo.opticalCenter.y - correctedRobotInImage.y);
    const Vector3<> cameraToTeamMarker(theCameraInfo.focalLength, theCameraInfo.opticalCenter.x - correctedTeamMarkerInImage.x, theCameraInfo.opticalCenter.y - correctedTeamMarkerInImage.y);
    const Vector3<> rotatedCameraToRobot = theCameraMatrix.rotation * cameraToRobot;
    const Vector3<> rotatedCameraToTeamMarker = theCameraMatrix.rotation * cameraToTeamMarker;
    const Vector3<> scaledCameraToRobot = rotatedCameraToRobot * (theCameraMatrix.translation.z / rotatedCameraToRobot.z);
    float markerHeight, markerCameraDistance;
    {
      const Vector2<> d(sqrt(sqr(scaledCameraToRobot.x) + sqr(scaledCameraToRobot.y)) + p.robotFeetRadius - p.robotWaistbandRadius, 0.f);
      const Vector2<> c(0.f, theCameraMatrix.translation.z);
      const Vector2<> v(sqrt(sqr(rotatedCameraToTeamMarker.x) + sqr(rotatedCameraToTeamMarker.y)),  rotatedCameraToTeamMarker.z);
      const float b = d.x / v.x;
      markerCameraDistance = v.abs() * b;
      markerHeight = c.y + v.y * b;
    }
    const float markerHeightVar = sqr(markerCameraDistance * 0.001f * p.heightDeviation);

    // compute robot position, angle and distance on field
    const Vector3<> robotOnField3d = theCameraMatrix.translation - scaledCameraToRobot;
    Vector2<> robotOnField(robotOnField3d.x, robotOnField3d.y);
    float distanceToRobot = robotOnField.abs();
    robotOnField *= 1.f + p.robotFeetRadius / distanceToRobot;
    distanceToRobot += p.robotFeetRadius;
    const float angleToRobot = approxAtan2(robotOnField.y, robotOnField.x);

    // update average height estimate
    if(markerHeight > p.robotMaxHorizontalHeight)
    {
      float& averageHeight = this->averageHeight[(int) robotPercept.teamRed];
      float& averageHeightVar = this->averageHeightVar[(int) robotPercept.teamRed];
      const float k = averageHeightVar / (averageHeightVar + markerHeightVar);
      averageHeight += k * (markerHeight - averageHeight);
      averageHeightVar -= k * averageHeightVar;
    }

    // try to find a compatible filter 
    for(std::vector<Filter>::iterator i = filters.begin(), end = filters.end(); i != end; ++i)
    {
      Filter& filter = *i;

      if(filter.teamRed == robotPercept.teamRed &&
         abs(normalize(filter.angle - angleToRobot)) < p.angleTolerance && // TODO: use probability stuff to compute find matching filters
         abs(distanceToRobot - filter.distance) / std::min(filter.distance, distanceToRobot) * 1000.f < p.distanceTolerance)
      {
        filter.updateHeight(markerHeight, markerHeightVar, p.robotMaxHorizontalHeight);
        filter.updatePos(robotOnField, getCovOfPixelInWorld(correctedRobotInImage, 0.f));
        filter.timestamp = theFrameInfo.time;
        filter.timeSinceNotSeen = 0;
        goto nextRobot;
      }
    }

    // add a new filter
    {
      const Vector2<> robotOnFieldRelToWorld = theRobotPose * robotOnField;
      if(theFieldDimensions.isInsideCarpet(robotOnFieldRelToWorld))
        filters.push_back(Filter(robotPercept.teamRed, robotOnField, getCovOfPixelInWorld(correctedRobotInImage, 0.f), markerHeight, markerHeightVar, angleToRobot, distanceToRobot, theFrameInfo.time));
    }

  nextRobot:;
  }

  // find matching filter for team maker spots
  for(std::vector<TeamMarkerSpots::TeamMarkerSpot>::const_iterator i = theTeamMarkerSpots.teamMarkers.begin(), end = theTeamMarkerSpots.teamMarkers.end(); i != end; ++i)
  {
    const TeamMarkerSpots::TeamMarkerSpot& teamMarkerSpot = *i;

    // compute vector to team marker
    const Vector2<> correctedTeamMarkerInImage = theImageCoordinateSystem.toCorrected(teamMarkerSpot.centerOfGravity);
    const Vector3<> cameraToTeamMarker(theCameraInfo.focalLength, theCameraInfo.opticalCenter.x - correctedTeamMarkerInImage.x, theCameraInfo.opticalCenter.y - correctedTeamMarkerInImage.y);
    const Vector3<> rotatedCameraToTeamMarker = theCameraMatrix.rotation * cameraToTeamMarker;
    const float sketchyAngleToRobot = approxAtan2(rotatedCameraToTeamMarker.y, rotatedCameraToTeamMarker.x);
    bool teamRed = teamMarkerSpot.color == ColorClasses::red;

    // try to find a compatible filter 
    for(std::vector<Filter>::iterator i = filters.begin(), end = filters.end(); i != end; ++i)
    {
      Filter& filter = *i;

      if(filter.teamRed == teamRed && abs(normalize(filter.angle - sketchyAngleToRobot)) < p.angleTolerance * 1.1f)
      {
        // compute distance and angle to the robot using the team maker height from the filter
        const Vector3<> scaledCameraToTeamMarker = rotatedCameraToTeamMarker * ((theCameraMatrix.translation.z - filter.height) / rotatedCameraToTeamMarker.z);
        const Vector3<> teamMarkerOnField3d = theCameraMatrix.translation - scaledCameraToTeamMarker;
        Vector2<> robotOnField(teamMarkerOnField3d.x, teamMarkerOnField3d.y);
        float distanceToRobot = robotOnField.abs();
        robotOnField *= 1.f + p.robotWaistbandRadius / distanceToRobot;
        distanceToRobot += p.robotWaistbandRadius;
        const float angleToRobot = approxAtan2(robotOnField.y, robotOnField.x);

        if(abs(normalize(filter.angle - angleToRobot)) < p.angleTolerance && // TODO: use probability stuff to compute find matching filters
         abs(distanceToRobot - filter.distance) / std::min(filter.distance, distanceToRobot) * 1000.f < p.distanceTolerance)
        {
          if(filter.timestamp != theFrameInfo.time)
          {
            filter.updatePos(robotOnField, getCovOfPixelInWorld(correctedTeamMarkerInImage, filter.height));
            filter.timestamp = theFrameInfo.time;
            filter.timeSinceNotSeen = 0;
          }
          goto nextRobot2;
        }
      }
    }

    // add a new filter
    {
      // compute distance and angle to the robot using the team maker height from the filter
      const float averageHeight = this->averageHeight[(int) teamRed];
      const Vector3<> scaledCameraToTeamMarker = rotatedCameraToTeamMarker * ((theCameraMatrix.translation.z - averageHeight) / rotatedCameraToTeamMarker.z);
      const float markerCameraDistance = scaledCameraToTeamMarker.abs();
      const Vector3<> teamMarkerOnField3d = theCameraMatrix.translation - scaledCameraToTeamMarker;
      Vector2<> robotOnField(teamMarkerOnField3d.x, teamMarkerOnField3d.y);
      float distanceToRobot = robotOnField.abs();
      robotOnField *= 1.f + p.robotWaistbandRadius / distanceToRobot;
      distanceToRobot += p.robotWaistbandRadius;
      const Vector2<> robotOnFieldRelToWorld = theRobotPose * robotOnField;
      if(theFieldDimensions.isInsideCarpet(robotOnFieldRelToWorld))
      {
        const float angleToRobot = approxAtan2(robotOnField.y, robotOnField.x);
        const float markerHeightVar = sqr(markerCameraDistance * 0.001f * p.heightDeviation * 10.f); // increase variance by factor 10 since marker height was guessed

        // add filter
        filters.push_back(Filter(teamRed, robotOnField, getCovOfPixelInWorld(correctedTeamMarkerInImage, averageHeight), averageHeight, markerHeightVar, angleToRobot, distanceToRobot, theFrameInfo.time));
      }
    }

  nextRobot2:;
  }
}

void ExpRobotLocator::Filter::updateHeight(float markerHeight, float markerHeightVar, float robotMaxHorizontalHeight)
{
  if((markerHeight <= robotMaxHorizontalHeight) != (height <= robotMaxHorizontalHeight))
  { // reset height
    height = markerHeight;
    heightVar = markerHeightVar;
  }
  else
  { // update height
    const float k = heightVar / (heightVar + markerHeightVar);
    height += k * (markerHeight - height);
    heightVar -= k * heightVar;
  }
}

void ExpRobotLocator::Filter::updatePos(const Vector2<>& robotOnField, const Matrix2x2<>& sensorCov)
{
  Matrix2x2<> covPlusSensorCov = posCov;
  covPlusSensorCov += sensorCov;
  Matrix2x2<> k = posCov * covPlusSensorCov.invert();
  Vector2<> innovation = robotOnField - pos;
  Vector2<> correction = k * innovation;
  pos += correction;
  posCov -= k * posCov;
}

void ExpRobotLocator::removeDatedFilters()
{
  Pose3D CameraMatrixInv = theCameraMatrix.invert();
  unsigned cycleTimeMs = (unsigned) (theFrameInfo.cycleTime * 1000.f);
  for(std::vector<Filter>::iterator i = filters.begin(), end = filters.end(); i != end; ++i)
  {
    Filter& filter = *i;
    if(filter.timestamp != theFrameInfo.time)
    {
      const Vector3<> robotMakerInCamera = CameraMatrixInv * Vector3<>(filter.pos.x, filter.pos.y, filter.height);
      if(robotMakerInCamera.x > 1)
      {
        const float scale = -theCameraInfo.focalLength / robotMakerInCamera.x;
        Vector2<> image(theCameraInfo.opticalCenter.x + scale * robotMakerInCamera.y, theCameraInfo.opticalCenter.y + scale * robotMakerInCamera.z);
        image = theImageCoordinateSystem.fromCorrectedApprox(image);
        if((image - Vector2<>(theCameraInfo.opticalCenter.x, theCameraInfo.opticalCenter.y)).squareAbs() < sqr(theCameraInfo.resolutionHeight * 0.24f))
          filter.timeSinceNotSeen += cycleTimeMs;
      }
    }
  }

  for(std::vector<Filter>::iterator i = filters.begin(), end = filters.end(); i != end; ++i)
  {
    Filter& filter = *i;
    if(filter.timeSinceNotSeen > p.robotNotSeenTimeout || theFrameInfo.getTimeSince(filter.timestamp) > (int) p.robotNotUpdatedTimeout)
    {
      *i = filters.back();
      filters.pop_back();
      break; // remove only one filter per iteration
    }
  }
}

Matrix2x2<> ExpRobotLocator::getCovOfPixelInWorld(const Vector2<>& correctedPointInImage, float pointZInWorld) const
{
  const Vector3<> unscaledVectorToPoint(theCameraInfo.focalLength, theCameraInfo.opticalCenter.x - correctedPointInImage.x, theCameraInfo.opticalCenter.y - correctedPointInImage.y);
  const Vector3<> unscaledWorld = theCameraMatrix.rotation * unscaledVectorToPoint;
  const float h = theCameraMatrix.translation.z - pointZInWorld;
  const float scale = h / -unscaledWorld.z;
  Vector2<> pointInWorld(unscaledWorld.x * scale, unscaledWorld.y * scale);
  const float distance = pointInWorld.abs();
  Vector2<> cossin = distance == 0.f ? Vector2<>(1.f, 0.f) : pointInWorld * (1.f / distance);
  Matrix2x2<> rot(cossin, Vector2<>(-cossin.y, cossin.x));
  Matrix2x2<> cov(Vector2<>(sqr(h / tan((distance == 0.f ? pi_2 : atan(h / distance)) - p.robotRotationDeviation.x) - distance), 0.f),
                 Vector2<>(0.f, sqr(tan(p.robotRotationDeviation.y) * distance)));
  return rot * cov * rot.transpose();
}
