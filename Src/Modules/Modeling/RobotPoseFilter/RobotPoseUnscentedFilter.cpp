/**
* @file RobotPoseUnscentedFilter.cpp
* Implementation of module RobotPoseUnscentedFilter
* @author Colin Graf
*/

#include "RobotPoseUnscentedFilter.h"
#include "Tools/Range.h"
#include "Tools/Debugging/DebugDrawings.h"

#define COVARIANCE2D(id, cov, mean) \
  COMPLEX_DRAWING(id, \
  { \
    const float factor = 1.f; \
    const float cov012 = cov[0][1] * cov[0][1]; \
    const float varianceDiff = cov[0][0] - cov[1][1]; \
    const float varianceDiff2 = varianceDiff * varianceDiff; \
    const float varianceSum = cov[0][0] + cov[1][1]; \
    const float root = sqrt(varianceDiff2 + 4.0f * cov012); \
    const float eigenValue1 = 0.5f * (varianceSum + root); \
    const float eigenValue2 = 0.5f * (varianceSum - root); \
    \
    const float axis1 = 2.0f * sqrt(factor * eigenValue1); \
    const float axis2 = 2.0f * sqrt(factor * eigenValue2); \
    const float angle = 0.5f * atan2(2.0f * cov[0][1], varianceDiff); \
    \
    ELLIPSE(id, mean, sqrt(3.0f) * axis1, sqrt(3.0f) * axis2, angle, \
            10, Drawings::ps_solid, ColorRGBA(100,100,255,100), Drawings::bs_solid, ColorRGBA(100,100,255,100)); \
    ELLIPSE(id, mean, sqrt(2.0f) * axis1, sqrt(2.0f) * axis2, angle, \
            10, Drawings::ps_solid, ColorRGBA(150,150,100,100), Drawings::bs_solid, ColorRGBA(150,150,100,100)); \
    ELLIPSE(id, mean, axis1, axis2, angle, \
            10, Drawings::ps_solid, ColorRGBA(255,100,100,100), Drawings::bs_solid, ColorRGBA(255,100,100,100)); \
  });


MAKE_MODULE(RobotPoseUnscentedFilter, Modeling)

RobotPoseUnscentedFilter::RobotPoseUnscentedFilter() : lastResetTime(0)
{
  p.respawnMaxPoseDistance = Pose2D(0.6f, 1000.f, 1000.f);
  p.respawnPoseDeviation = Pose2D(0.6f, 1000.f, 1000.f);
  p.lineRelationCorridor = 300.f;
  p.odometryDeviation = Pose2D(0.3f, 0.2f, 0.2f);
  p.odometryRotationDeviation = Vector2<>(pi_2 / 1000.f, pi_2 / 1000.f);
  p.filterProcessDeviation = Pose2D(0.001f, 0.8f, 0.8f);
  p.robotRotationDeviation = Vector2<>(0.02f, 0.06f);
  p.robotRotationDeviationInStand = Vector2<>(0.02f, 0.04f);
}

void RobotPoseUnscentedFilter::init()
{
  // prepare relevant field line table
  countOfFieldLines = 0;
  for(unsigned int i = 0, count = theFieldDimensions.fieldLines.lines.size(); i < count; ++i)
  {
    const FieldDimensions::LinesTable::Line& fieldLine = theFieldDimensions.fieldLines.lines[i];
    if(!fieldLine.isPartOfCircle && fieldLine.length > 300.f)
    {
      ASSERT(countOfFieldLines < int(sizeof(fieldLines) / sizeof(*fieldLines)));
      FieldLine& relevantFieldLine = fieldLines[countOfFieldLines++];
      relevantFieldLine.start = fieldLine.corner.translation;
      relevantFieldLine.end = fieldLine.corner * Vector2<>(fieldLine.length, 0);
      relevantFieldLine.dir = relevantFieldLine.end - relevantFieldLine.start;
      relevantFieldLine.dir.normalize();
      relevantFieldLine.length = fieldLine.length;
      relevantFieldLine.vertical = abs(fieldLine.corner.rotation) < 0.001f || abs(normalize(fieldLine.corner.rotation - pi)) < 0.001f;
    }
  }

  // and goal posts
  goalPosts[GoalPercept::OPPONENT_TEAM][GoalPost::IS_LEFT] = Vector2<>(float(theFieldDimensions.xPosOpponentGoalpost), float(theFieldDimensions.yPosLeftGoal));
  goalPosts[GoalPercept::OPPONENT_TEAM][GoalPost::IS_RIGHT] = Vector2<>(float(theFieldDimensions.xPosOpponentGoalpost), float(theFieldDimensions.yPosRightGoal));
  goalPosts[GoalPercept::OWN_TEAM][GoalPost::IS_LEFT] = Vector2<>(float(theFieldDimensions.xPosOwnGoalpost), float(theFieldDimensions.yPosRightGoal));
  goalPosts[GoalPercept::OWN_TEAM][GoalPost::IS_RIGHT] = Vector2<>(float(theFieldDimensions.xPosOwnGoalpost), float(theFieldDimensions.yPosLeftGoal));
}

void RobotPoseUnscentedFilter::update(FilteredRobotPose& filteredRobotPose)
{
  if(theMotionInfo.motion == MotionRequest::specialAction
     && theMotionInfo.specialActionRequest.specialAction == SpecialActionRequest::sitDownKeeper)
  {
    return;
  }
  MODIFY("module:RobotPoseUnscentedFilter:parameters", p);

  DECLARE_DEBUG_DRAWING("module:RobotPoseUnscentedFilter:image", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:RobotPoseUnscentedFilter:field", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:RobotPoseUnscentedFilter:fieldRel", "drawingOnField");

  motionUpdate();

  // respawn?
  if(lastResetTime == 0 || lastPoseSetTime != thePotentialRobotPose.poseSetTime ||
     abs(mean.x - thePotentialRobotPose.translation.x) > p.respawnMaxPoseDistance.translation.x ||
     abs(mean.y - thePotentialRobotPose.translation.y) > p.respawnMaxPoseDistance.translation.y ||
     abs(normalize(mean.z - thePotentialRobotPose.rotation)) > p.respawnMaxPoseDistance.rotation ||
     !theGroundContactState.contact ||
     theFallDownState.state != theFallDownState.upright)
  {
    mean = Vector3f(thePotentialRobotPose.translation.x, thePotentialRobotPose.translation.y, thePotentialRobotPose.rotation);
    cov = Matrix3x3f();
    cov[0][0] = sqr(p.respawnPoseDeviation.translation.x);
    cov[1][1] = sqr(p.respawnPoseDeviation.translation.y);
    cov[2][2] = sqr(p.respawnPoseDeviation.rotation);
    lastResetTime = theFrameInfo.time;
    lastPoseSetTime = thePotentialRobotPose.poseSetTime;
  }

  // try to relate the line percepts
  int middleLine = -1;
  dedicatedLinePercepts.clear();
  Vector2<> intersection, orthogonalProjection;
  float sqrLineRelationCorridor = sqr(p.lineRelationCorridor);
  LinePercept linePercept;
  for(std::list< ::LinePercept::Line>::const_iterator it = theLinePercept.lines.begin(), lastIt = theLinePercept.lines.end(); it != lastIt; ++it)
  {
    const ::LinePercept::Line& line = *it;
    linePercept.start = Vector2<>(float(line.first.x), float(line.first.y));
    linePercept.end = Vector2<>(float(line.last.x), float(line.last.y));
    Vector2<> startOnField = thePotentialRobotPose * linePercept.start;
    Vector2<> endOnField = thePotentialRobotPose * linePercept.end;
    Vector2<> dirOnField = endOnField - startOnField;
    dirOnField.normalize();
    Vector2<> orthogonalOnField(dirOnField.y, -dirOnField.x);

    COMPLEX_DRAWING("module:RobotPoseUnscentedFilter:field",
    {
      Vector2<> checkLineStart1 = startOnField + orthogonalOnField* p.lineRelationCorridor;
      Vector2<> checkLineEnd1 = startOnField - orthogonalOnField* p.lineRelationCorridor;
      Vector2<> checkLineStart2 = endOnField + orthogonalOnField* p.lineRelationCorridor;
      Vector2<> checkLineEnd2 = endOnField - orthogonalOnField* p.lineRelationCorridor;
      LINE("module:RobotPoseUnscentedFilter:field", checkLineStart1.x, checkLineStart1.y, checkLineEnd1.x, checkLineEnd1.y, 0, Drawings::ps_solid, ColorRGBA(0xff, 0, 0));
      LINE("module:RobotPoseUnscentedFilter:field", checkLineStart2.x, checkLineStart2.y, checkLineEnd2.x, checkLineEnd2.y, 0, Drawings::ps_solid, ColorRGBA(0xff, 0, 0));
    });

    linePercept.index = -1;
    for(int i = 0; i < countOfFieldLines; ++i)
    {
      const FieldLine& fieldLine = fieldLines[i];

      if(getSqrDistanceToLine(fieldLine.start, fieldLine.dir, fieldLine.length, startOnField) > sqrLineRelationCorridor ||
         getSqrDistanceToLine(fieldLine.start, fieldLine.dir, fieldLine.length, endOnField) > sqrLineRelationCorridor)
        continue;
      if(!intersectLineWithLine(startOnField, orthogonalOnField, fieldLine.start, fieldLine.dir, intersection))
        continue;
      if(getSqrDistanceToLine(startOnField, dirOnField, intersection) > sqrLineRelationCorridor)
        continue;
      if(!intersectLineWithLine(endOnField, orthogonalOnField, fieldLine.start, fieldLine.dir, intersection))
        continue;
      if(getSqrDistanceToLine(startOnField, dirOnField, intersection) > sqrLineRelationCorridor)
        continue;
      if(linePercept.index != -1)
      {
        // ambiguous?
        linePercept.index = -1;
        break;
      }
      linePercept.index = i;
    }

    if(linePercept.index != -1) // success
    {
      if(line.midLine)
        middleLine = dedicatedLinePercepts.size();
      dedicatedLinePercepts.push_back(linePercept);
    }
  }

  // try to relate the goal posts
  dedicatedGoalPercepts.clear();
  GoalPercept goalPercept;
  for(int i = 0, count = theGoalPercept.goalPosts.size(); i < count; ++i)
  {
    const GoalPost& post = theGoalPercept.goalPosts[i];
    goalPercept.position = Vector2<>(float(post.positionOnField.x), float(post.positionOnField.y));
    Vector2<> positionOnField = thePotentialRobotPose * goalPercept.position;
    goalPercept.membership = positionOnField.x > 0.f ? GoalPercept::OPPONENT_TEAM : GoalPercept::OWN_TEAM;
    goalPercept.side = post.position;
    if(goalPercept.side != GoalPost::IS_UNKNOWN)
    {
      if((positionOnField - goalPosts[goalPercept.membership][goalPercept.side]).squareAbs() < sqrLineRelationCorridor)
        dedicatedGoalPercepts.push_back(goalPercept);
    }
    else
    {
      ASSERT(GoalPost::IS_LEFT == GoalPost::IS_RIGHT - 1);
      for(int j = GoalPost::IS_LEFT; j <= GoalPost::IS_RIGHT; ++j)
      {
        if((positionOnField - goalPosts[goalPercept.membership][j]).squareAbs() > sqrLineRelationCorridor)
          continue;
        if(goalPercept.side != GoalPost::IS_UNKNOWN)
        {
          // ambiguous?
          goalPercept.side = GoalPost::IS_UNKNOWN;
          break;
        }
        goalPercept.side = (GoalPost::Position) j;
      }
      if(goalPercept.side != GoalPost::IS_UNKNOWN)
        dedicatedGoalPercepts.push_back(goalPercept);
    }
  }

  // try to use the center circle
  bool centerCircle = false;
  if(theLinePercept.circle.found)
  {
    Vector2<> circlePos(float(theLinePercept.circle.pos.x), float(theLinePercept.circle.pos.y));
    Vector2<> circlePosOnField = thePotentialRobotPose * circlePos;
    if(circlePosOnField.squareAbs() < sqrLineRelationCorridor)
      centerCircle = true;
  }

  // use dedicated percepts
  if(middleLine >= 0 && centerCircle)
  {
    useCenterCircleWithMiddleLine(Vector2<>(float(theLinePercept.circle.pos.x), float(theLinePercept.circle.pos.y)), dedicatedLinePercepts[middleLine]);
    dedicatedLinePercepts[middleLine].index = -1;
    centerCircle = false;
  }
  for(int i = 0, count = dedicatedLinePercepts.size(); i < count; ++i)
  {
    const LinePercept& linePercept = dedicatedLinePercepts[i];
    if(linePercept.index != -1)
      useLinePercept(linePercept, fieldLines[linePercept.index]);
  }
  for(int i = 0, count = dedicatedGoalPercepts.size(); i < count; ++i)
  {
    const GoalPercept& goalPercept = dedicatedGoalPercepts[i];
    if(goalPercept.side != GoalPost::IS_UNKNOWN)
      useGoalPost(goalPercept.position, goalPosts[goalPercept.membership][goalPercept.side]);
  }
  if(centerCircle)
    useCenterCircle(Vector2<>(float(theLinePercept.circle.pos.x), float(theLinePercept.circle.pos.y)));

  // generate model
  filteredRobotPose.rotation = mean.z;
  filteredRobotPose.translation.x = mean.x;
  filteredRobotPose.translation.y = mean.y;
  filteredRobotPose.cov = Matrix3x3<>(Vector3<>(cov[0].x, cov[0].y, cov[0].z),
                                      Vector3<>(cov[1].x, cov[1].y, cov[1].z),
                                      Vector3<>(cov[2].x, cov[2].y, cov[2].z));
  filteredRobotPose.filterResetTime = lastResetTime;

#ifndef RELEASE
  Vector2<> translation(mean.x, mean.y);
  Matrix2x2<> translationCov(Vector2<>(cov[0].x, cov[0].y), Vector2<>(cov[1].x, cov[1].y));
  COVARIANCE2D("module:RobotPoseUnscentedFilter:field", translationCov, translation);
#endif
}

void RobotPoseUnscentedFilter::motionUpdate()
{
  generateSigmaPoints();
  addOdometryToSigmaPoints();
  computeMeanOfSigmaPoints();
  computeCovOfSigmaPoints();
  addProcessNoise();

  mean.z = normalize(mean.z);
}

void RobotPoseUnscentedFilter::computeCholeskyDecompositionOfCov()
{
  // improved symmetry input
  const float a11 = cov.c[0].x;
  const float a21 = (cov.c[0].y + cov.c[1].x) * 0.5f;
  const float a31 = (cov.c[0].z + cov.c[2].x) * 0.5f;

  const float a22 = cov.c[1].y;
  const float a32 = (cov.c[1].z + cov.c[2].y) * 0.5f;

  const float a33 = cov.c[2].z;

  // output
  float& l11(l.c[0].x);
  float& l21(l.c[0].y);
  float& l31(l.c[0].z);

  float& l22(l.c[1].y);
  float& l32(l.c[1].z);

  float& l33(l.c[2].z);

  // compute cholesky decomposition

  //ASSERT(a11 >= 0.f);
  l11 = sqrt(std::max<>(a11, 0.f));
  if(l11 == 0.f) l11 = 0.0000000001f;
  l21 = a21 / l11;
  l31 = a31 / l11;

  //ASSERT(a22 - l21 * l21 >= 0.f);
  l22 = sqrt(std::max<>(a22 - l21 * l21, 0.f));
  if(l22 == 0.f) l22 = 0.0000000001f;
  l32 = (a32 - l31 * l21) / l22;

  //ASSERT(a33 - l31 * l31 - l32 * l32 >= 0.f);
  l33 = sqrt(std::max<>(a33 - l31 * l31 - l32 * l32, 0.f));
}

void RobotPoseUnscentedFilter::generateSigmaPoints()
{
  computeCholeskyDecompositionOfCov();
  sigmaPoints[0] = mean;
  sigmaPoints[1] = mean + l.c[0];
  sigmaPoints[2] = mean - l.c[0];
  sigmaPoints[3] = mean + l.c[1];
  sigmaPoints[4] = mean - l.c[1];
  sigmaPoints[5] = mean + l.c[2];
  sigmaPoints[6] = mean - l.c[2];
}

void RobotPoseUnscentedFilter::addOdometryToSigmaPoints()
{
  odometryOffset = theOdometryData - lastOdometryData;
  for(int i = 0; i < 7; ++i)
  {
    Vector2<> odo(odometryOffset.translation);
    odo.rotate(sigmaPoints[i].z);
    sigmaPoints[i] += Vector3f(odo.x, odo.y, odometryOffset.rotation);
  }
  lastOdometryData = theOdometryData;
}

void RobotPoseUnscentedFilter::computeMeanOfSigmaPoints()
{
  mean = sigmaPoints[0];
  for(int i = 1; i < 7; ++i)
    mean += sigmaPoints[i];
  mean *= 1.f / 7.f;
}

void RobotPoseUnscentedFilter::computeCovOfSigmaPoints()
{
  Vector3f d = sigmaPoints[0] - mean;
  cov = Matrix3x3f(d * d.x, d * d.y, d * d.z);
  for(int i = 1; i < 7; ++i)
  {
    Vector3f d = sigmaPoints[i] - mean;
    cov += Matrix3x3f(d * d.x, d * d.y, d * d.z);
  }
  cov *= 0.5f;
}

void RobotPoseUnscentedFilter::addProcessNoise()
{
  cov[0][0] += sqr(p.filterProcessDeviation.translation.x);
  cov[1][1] += sqr(p.filterProcessDeviation.translation.y);
  cov[2][2] += sqr(p.filterProcessDeviation.rotation);

  Vector2<> odo(odometryOffset.translation);
  odo.rotate(mean.z);
  cov[0][0] += sqr(odo.x * p.odometryDeviation.translation.x);
  cov[1][1] += sqr(odo.y * p.odometryDeviation.translation.y);
  cov[2][2] += sqr(odometryOffset.rotation * p.odometryDeviation.rotation);
  cov[2][2] += sqr(odo.x * p.odometryRotationDeviation.x);
  cov[2][2] += sqr(odo.y * p.odometryRotationDeviation.y);
}

void RobotPoseUnscentedFilter::lineSensorUpdate(bool vertical, const Vector2f& reading, const Matrix2x2f& readingCov)
{
  COMPLEX_DRAWING("module:RobotPoseUnscentedFilter:field",
  {
    Vector2<> checkLineStart1 = vertical ? Vector2<>(3500.f, reading.x) : Vector2<>(reading.x, 2500.f);
    Vector2<> checkLineEnd1 = vertical ? Vector2<>(-3500.f, reading.x) : Vector2<>(reading.x, -2500.f);
    LINE("module:RobotPoseUnscentedFilter:field", checkLineStart1.x, checkLineStart1.y, checkLineEnd1.x, checkLineEnd1.y, 0, Drawings::ps_solid, ColorRGBA(0xff, 0xaa, 0));
    Vector2<> rot(1000.f, 0.f);
    rot.rotate(reading.y);
    Vector2<> base = vertical ? Vector2<>(mean.x, reading.x) : Vector2<>(reading.x, mean.y);
    rot += base;
    ARROW("module:RobotPoseUnscentedFilter:field", base.x, base.y, rot.x, rot.y, 0, Drawings::ps_solid, ColorRGBA(0xff, 0xaa, 0));
  });

  generateSigmaPoints();
  computeLineReadings(vertical);
  computeMeanOfLineReadings();
  computeCovOfLineReadingsAndSigmaPoints();
  computeCovOfLineReadingsReadings();
  
  lineReadingMean.y = normalize(lineReadingMean.y);
  const Matrix3x2f kalmanGain = lineReadingAndMeanCov.transpose() * (lineReadingCov + readingCov).invert();
  Vector2f innovation = reading - lineReadingMean;
  innovation.y = normalize(innovation.y);
  const Vector3f correction = kalmanGain * innovation;
  mean += correction;
  mean.z = normalize(mean.z);
  cov -= kalmanGain * lineReadingAndMeanCov;
}

void RobotPoseUnscentedFilter::computeLineReadings(bool vertical)
{
  if(vertical)
    for(int i = 0; i < 7; ++i)
      lineReadings[i] = Vector2f(sigmaPoints[i].y, sigmaPoints[i].z);
  else
    for(int i = 0; i < 7; ++i)
      lineReadings[i] = Vector2f(sigmaPoints[i].x, sigmaPoints[i].z);
}

void RobotPoseUnscentedFilter::computeMeanOfLineReadings()
{
  lineReadingMean = lineReadings[0];
  for(int i = 1; i < 7; ++i)
    lineReadingMean += lineReadings[i];
  lineReadingMean *= 1.f / 7.f;
}

void RobotPoseUnscentedFilter::computeCovOfLineReadingsAndSigmaPoints()
{
  lineReadingAndMeanCov = Matrix2x3f();
  for(int i = 0; i < 3; ++i)
  {
    Vector2f d1 = lineReadings[i * 2 + 1] - lineReadingMean;
    lineReadingAndMeanCov += Matrix2x3f(d1 * l[i].x, d1 * l[i].y, d1 * l[i].z);
    Vector2f d2 = lineReadings[i * 2 + 2] - lineReadingMean;
    lineReadingAndMeanCov += Matrix2x3f(d2 * -l[i].x, d2 * -l[i].y, d2 * -l[i].z);
  }
  lineReadingAndMeanCov *= 0.5f;
}

void RobotPoseUnscentedFilter::computeCovOfLineReadingsReadings()
{
  Vector2f d = lineReadings[0] - lineReadingMean;
  lineReadingCov = Matrix2x2f(d * d.x, d * d.y);
  for(int i = 1; i < 7; ++i)
  {
    Vector2f d = lineReadings[i] - lineReadingMean;
    lineReadingCov += Matrix2x2f(d * d.x, d * d.y);
  }
  lineReadingCov *= 0.5f;
}

void RobotPoseUnscentedFilter::landmarkSensorUpdate(const Vector2<>& landmarkPosition, const Vector2f& reading, const Matrix2x2f& readingCov)
{
  COMPLEX_DRAWING("module:RobotPoseUnscentedFilter:field",
  {
    CIRCLE("module:RobotPoseUnscentedFilter:field", landmarkPosition.x, landmarkPosition.y, reading.abs(), 0, Drawings::ps_solid, ColorRGBA(0xff, 0xaa, 0), Drawings::bs_null, ColorRGBA());
  });

  generateSigmaPoints();
  computeLandmarkReadings(landmarkPosition);
  computeMeanOfLandmarkReadings();
  computeCovOfLandmarkReadingsAndSigmaPoints();
  computeCovOfLandmarkReadingsReadings();
  
  const Matrix3x2f kalmanGain = landmarkReadingAndMeanCov.transpose() * (landmarkReadingCov + readingCov).invert();
  Vector2f innovation = reading - landmarkReadingMean;
  const Vector3f correction = kalmanGain * innovation;
  mean += correction;
  mean.z = normalize(mean.z);
  cov -= kalmanGain * landmarkReadingAndMeanCov;
}

void RobotPoseUnscentedFilter::computeLandmarkReadings(const Vector2<>& landmarkPosition)
{
  for(int i = 0; i < 7; ++i)
  {
    Pose2D pose(sigmaPoints[i].z, sigmaPoints[i].x, sigmaPoints[i].y);
    Vector2<> landmarkPosRel = pose.invert() * landmarkPosition; // TODO: optimize this
    landmarkReadings[i] = Vector2f(landmarkPosRel.x, landmarkPosRel.y);
  }
}

void RobotPoseUnscentedFilter::computeMeanOfLandmarkReadings()
{
  landmarkReadingMean = landmarkReadings[0];
  for(int i = 1; i < 7; ++i)
    landmarkReadingMean += landmarkReadings[i];
  landmarkReadingMean *= 1.f / 7.f;
}

void RobotPoseUnscentedFilter::computeCovOfLandmarkReadingsAndSigmaPoints()
{
  landmarkReadingAndMeanCov = Matrix2x3f();
  for(int i = 0; i < 3; ++i)
  {
    Vector2f d1 = landmarkReadings[i * 2 + 1] - landmarkReadingMean;
    landmarkReadingAndMeanCov += Matrix2x3f(d1 * l[i].x, d1 * l[i].y, d1 * l[i].z);
    Vector2f d2 = landmarkReadings[i * 2 + 2] - landmarkReadingMean;
    landmarkReadingAndMeanCov += Matrix2x3f(d2 * -l[i].x, d2 * -l[i].y, d2 * -l[i].z);
  }
  landmarkReadingAndMeanCov *= 0.5f;
}

void RobotPoseUnscentedFilter::computeCovOfLandmarkReadingsReadings()
{
  Vector2f d = landmarkReadings[0] - landmarkReadingMean;
  landmarkReadingCov = Matrix2x2f(d * d.x, d * d.y);
  for(int i = 1; i < 7; ++i)
  {
    Vector2f d = landmarkReadings[i] - landmarkReadingMean;
    landmarkReadingCov += Matrix2x2f(d * d.x, d * d.y);
  }
  landmarkReadingCov *= 0.5f;
}

void RobotPoseUnscentedFilter::poseSensorUpdate(const Vector3f& reading, const Matrix3x3f& readingCov)
{
  COMPLEX_DRAWING("module:RobotPoseUnscentedFilter:field",
  {
    Vector2<> checkLineStart1(3500.f, reading.y);
    Vector2<> checkLineEnd1(-3500.f, reading.y);
    LINE("module:RobotPoseUnscentedFilter:field", checkLineStart1.x, checkLineStart1.y, checkLineEnd1.x, checkLineEnd1.y, 0, Drawings::ps_solid, ColorRGBA(0xff, 0xaa, 0));
    Vector2<> checkLineStart2(reading.x, 2500.f);
    Vector2<> checkLineEnd2(reading.x, -2500.f);
    LINE("module:RobotPoseUnscentedFilter:field", checkLineStart2.x, checkLineStart2.y, checkLineEnd2.x, checkLineEnd2.y, 0, Drawings::ps_solid, ColorRGBA(0xff, 0xaa, 0));
    Vector2<> rot(1000.f, 0.f);
    rot.rotate(reading.z);
    rot += Vector2<>(reading.x, reading.y);
    ARROW("module:RobotPoseUnscentedFilter:field", reading.x, reading.y, rot.x, rot.y, 0, Drawings::ps_solid, ColorRGBA(0xff, 0xaa, 0));
  });

  generateSigmaPoints();
  computePoseReadings();
  computeMeanOfPoseReadings();
  computeCovOfPoseReadingsAndSigmaPoints();
  computeCovOfPoseReadingsReadings();
  
  poseReadingMean.z = normalize(poseReadingMean.z);
  const Matrix3x3f kalmanGain = poseReadingAndMeanCov.transpose() * (poseReadingCov + readingCov).invert();
  Vector3f innovation = reading - poseReadingMean;
  innovation.z = normalize(innovation.z);
  const Vector3f correction = kalmanGain * innovation;
  mean += correction;
  mean.z = normalize(mean.z);
  cov -= kalmanGain * poseReadingAndMeanCov;
}

void RobotPoseUnscentedFilter::computePoseReadings()
{
  for(int i = 0; i < 7; ++i)
    poseReadings[i] = Vector3f(sigmaPoints[i].x, sigmaPoints[i].y, sigmaPoints[i].z);
}

void RobotPoseUnscentedFilter::computeMeanOfPoseReadings()
{
  poseReadingMean = poseReadings[0];
  for(int i = 1; i < 7; ++i)
    poseReadingMean += poseReadings[i];
  poseReadingMean *= 1.f / 7.f;
}

void RobotPoseUnscentedFilter::computeCovOfPoseReadingsAndSigmaPoints()
{
  poseReadingAndMeanCov = Matrix3x3f();
  for(int i = 0; i < 3; ++i)
  {
    Vector3f d1 = poseReadings[i * 2 + 1] - poseReadingMean;
    poseReadingAndMeanCov += Matrix3x3f(d1 * l[i].x, d1 * l[i].y, d1 * l[i].z);
    Vector3f d2 = poseReadings[i * 2 + 2] - poseReadingMean;
    poseReadingAndMeanCov += Matrix3x3f(d2 * -l[i].x, d2 * -l[i].y, d2 * -l[i].z);
  }
  poseReadingAndMeanCov *= 0.5f;
}

void RobotPoseUnscentedFilter::computeCovOfPoseReadingsReadings()
{
  Vector3f d = poseReadings[0] - poseReadingMean;
  poseReadingCov = Matrix3x3f(d * d.x, d * d.y, d * d.z);
  for(int i = 1; i < 7; ++i)
  {
    Vector3f d = poseReadings[i] - poseReadingMean;
    poseReadingCov += Matrix3x3f(d * d.x, d * d.y, d * d.z);
  }
  poseReadingCov *= 0.5f;
}

void RobotPoseUnscentedFilter::useLinePercept(const LinePercept& linePercept, const FieldLine& fieldLine)
{
  Vector2<> center = (linePercept.start + linePercept.end) * 0.5f;
  Vector2<> dir = linePercept.end - linePercept.start;
  dir.normalize();
  Matrix2x2f cov = getCovOfPointInWorld(center, 0.f);
  /*
  COMPLEX_DRAWING("module:RobotPoseUnscentedFilter:field",
  {
    Vector2<> centerOnField = thePotentialRobotPose* center;
    float dist = (centerOnField.x - fieldLine.start.x) * fieldLine.dir.x + (centerOnField.y - fieldLine.start.y) * fieldLine.dir.y;
    Vector2<> foot = fieldLine.start + fieldLine.dir* dist;
    LINE("module:RobotPoseUnscentedFilter:field", centerOnField.x, centerOnField.y, foot.x, foot.y, 0, Drawings::ps_solid, ColorRGBA(0, 0, 0xff));
  });
  COVARIANCE2D("module:RobotPoseUnscentedFilter:fieldRel", cov, center);
  */

  Vector2<> orthogonalProjectiona = getOrthogonalProjection(linePercept.start, dir, Vector2<>());
  float measuredAngle = -atan2(orthogonalProjectiona.y, orthogonalProjectiona.x);
  measuredAngle = normalize(measuredAngle + (fieldLine.vertical ? pi_2 : 0));
  float possibleAngle2 = normalize(measuredAngle - pi);
  if(abs(normalize(possibleAngle2 - thePotentialRobotPose.rotation)) < abs(normalize(measuredAngle - thePotentialRobotPose.rotation)))
    measuredAngle = possibleAngle2;
  float c = cos(measuredAngle), s = sin(measuredAngle);
  Matrix2x2f angleRotationMatrix(Vector2f(c, s), Vector2f(-s, c));
  Vector2f orthogonalProjection = angleRotationMatrix * Vector2f(orthogonalProjectiona.x, orthogonalProjectiona.y);

  cov = angleRotationMatrix * cov * angleRotationMatrix.transpose();

  if(fieldLine.vertical)
  {
    const float measuredY = fieldLine.start.y - orthogonalProjection.y;
    const float yVariance = cov[1][1];
    const float angleVariance = sqr(atan(sqrt(4.f * yVariance / (linePercept.start - linePercept.end).squareAbs())));
    lineSensorUpdate(true, Vector2f(measuredY, measuredAngle), Matrix2x2f(Vector2f(yVariance, 0.f), Vector2f(0.f, angleVariance)));
  }
  else
  {
    const float measuredX = fieldLine.start.x - orthogonalProjection.x;
    const float xVariance = cov[0][0];
    const float angleVariance = sqr(atan(sqrt(4.f * xVariance / (linePercept.start - linePercept.end).squareAbs())));
    lineSensorUpdate(false, Vector2f(measuredX, measuredAngle), Matrix2x2f(Vector2f(xVariance, 0.f), Vector2f(0.f, angleVariance)));
  }
}

void RobotPoseUnscentedFilter::useGoalPost(const Vector2<>& postPos, const Vector2<>& postOnField)
{
  Matrix2x2f cov = getCovOfPointInWorld(postPos, 0.f);
  landmarkSensorUpdate(postOnField, Vector2f(postPos.x, postPos.y), cov);
}

void RobotPoseUnscentedFilter::useCenterCircle(const Vector2<>& circlePos)
{
  Matrix2x2f cov = getCovOfCircle(circlePos);
  landmarkSensorUpdate(Vector2<>(), Vector2f(circlePos.x, circlePos.y), cov);
}

void RobotPoseUnscentedFilter::useCenterCircleWithMiddleLine(const Vector2<>& circlePos, const LinePercept& linePercept)
{
  Matrix2x2f circleCov = getCovOfCircle(circlePos);

  Vector2<> lineCenter = (linePercept.start + linePercept.end) * 0.5f;
  Matrix2x2f lineCov = getCovOfPointInWorld(lineCenter, 0.f);

  Vector2<> dir = linePercept.end - linePercept.start;
  dir.normalize();
  Vector2<> orthogonalProjectiona = getOrthogonalProjection(linePercept.start, dir, Vector2<>());
  float measuredAngle = -atan2(orthogonalProjectiona.y, orthogonalProjectiona.x);
  measuredAngle = normalize(measuredAngle);
  float possibleAngle2 = normalize(measuredAngle - pi);
  if(abs(normalize(possibleAngle2 - thePotentialRobotPose.rotation)) < abs(normalize(measuredAngle - thePotentialRobotPose.rotation)))
    measuredAngle = possibleAngle2;
  float c = cos(measuredAngle), s = sin(measuredAngle);
  Matrix2x2f angleRotationMatrix(Vector2f(c, s), Vector2f(-s, c));
  Vector2f orthogonalProjection = angleRotationMatrix * Vector2f(orthogonalProjectiona.x, orthogonalProjectiona.y);

  Matrix2x2f covXR = angleRotationMatrix * lineCov * angleRotationMatrix.transpose();
  Matrix2x2f covY = angleRotationMatrix * circleCov * angleRotationMatrix.transpose();

  const float measuredX = -orthogonalProjection.x;
  const float xVariance = covXR[0][0];
  const float angleVariance = sqr(atan(sqrt(4.f * xVariance / (linePercept.start - linePercept.end).squareAbs())));

  const float measueredY = -(Pose2D(measuredAngle, measuredX, 0.f) * circlePos).y;

  poseSensorUpdate(Vector3f(measuredX, measueredY, measuredAngle), Matrix3x3f(
    Vector3f(xVariance, 0.f, 0.f), Vector3f(0.f, covY[1][1], 0.f), Vector3f(0.f, 0.f, angleVariance)));
}

bool RobotPoseUnscentedFilter::intersectLineWithLine(const Vector2<>& lineBase1, const Vector2<>& lineDir1,
    const Vector2<>& lineBase2, const Vector2<>& lineDir2, Vector2<>& intersection) const
{
  float h = lineDir1.x * lineDir2.y - lineDir1.y * lineDir2.x;
  if(h == 0.f)
    return false;
  float scale = ((lineBase2.x - lineBase1.x) * lineDir1.y - (lineBase2.y - lineBase1.y) * lineDir1.x) / h;
  intersection.x = lineBase2.x + lineDir2.x * scale;
  intersection.y = lineBase2.y + lineDir2.y * scale;
  return true;
}

float RobotPoseUnscentedFilter::getSqrDistanceToLine(const Vector2<>& base, const Vector2<>& dir, float length, const Vector2<>& point) const
{
  float l = (point.x - base.x) * dir.x + (point.y - base.y) * dir.y;
  if(l < 0)
    l = 0;
  if(l > length)
    l = length;
  return ((base + dir * l) - point).squareAbs();
}

float RobotPoseUnscentedFilter::getSqrDistanceToLine(const Vector2<>& base, const Vector2<>& dir, const Vector2<>& point) const
{
  float l = (point.x - base.x) * dir.x + (point.y - base.y) * dir.y;
  return ((base + dir * l) - point).squareAbs();
}

Vector2<> RobotPoseUnscentedFilter::getOrthogonalProjection(const Vector2<>& base, const Vector2<>& dir, const Vector2<>& point) const
{
  float l = (point.x - base.x) * dir.x + (point.y - base.y) * dir.y;
  return base + dir * l;
}

Matrix2x2f RobotPoseUnscentedFilter::getCovOfPointInWorld(const Vector2<>& pointInWorld2, float pointZInWorld) const
{
  Vector3<> unscaledVectorToPoint = theCameraMatrix.invert() * Vector3<>(pointInWorld2.x, pointInWorld2.y, pointZInWorld);
  const Vector3<> unscaledWorld = theCameraMatrix.rotation * unscaledVectorToPoint;
  const float h = theCameraMatrix.translation.z - pointZInWorld;
  const float scale = h / -unscaledWorld.z;
  Vector2f pointInWorld(unscaledWorld.x * scale, unscaledWorld.y * scale);
  const float distance = pointInWorld.abs();
  Vector2f cossin = distance == 0.f ? Vector2f(1.f, 0.f) : pointInWorld * (1.f / distance);
  Matrix2x2f rot(cossin, Vector2f(-cossin.y, cossin.x));
  const Vector2<>& robotRotationDeviation = theMotionInfo.motion == MotionRequest::stand ? p.robotRotationDeviationInStand : p.robotRotationDeviation;
  Matrix2x2f cov(Vector2f(sqr(h / tan((distance == 0.f ? pi_2 : atan(h / distance)) - robotRotationDeviation.x) - distance), 0.f),
                  Vector2f(0.f, sqr(tan(robotRotationDeviation.y) * distance)));
  return rot * cov * rot.transpose();
}

Matrix2x2f RobotPoseUnscentedFilter::getCovOfCircle(const Vector2<>& circlePos) const
{
  float circleDistance = circlePos.abs();
  Vector2<> increasedCirclePos = circlePos;
  if(circleDistance < theFieldDimensions.centerCircleRadius * 2.f)
  {
    if(circleDistance < 10.f)
      increasedCirclePos = Vector2<>((float) theFieldDimensions.centerCircleRadius * 2, 0.f);
    else
      increasedCirclePos *= theFieldDimensions.centerCircleRadius * 2.f / circleDistance;
  }
  return getCovOfPointInWorld(increasedCirclePos, 0.f);
}
