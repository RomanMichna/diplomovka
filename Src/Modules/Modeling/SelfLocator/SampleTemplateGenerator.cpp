/**
* @file SampleTemplateGenerator.cpp
*
* This file implements a submodule that generates robot positions from percepts.
*
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#include "SampleTemplateGenerator.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Probabilistics.h"

SampleTemplateGenerator::SampleTemplateGenerator(const SelfLocatorParameters& parameters,
                                                 const GoalPercept& goalPercept, const LinePercept& linePercept,
                                                 const FrameInfo& frameInfo,
                                                 const FieldDimensions& fieldDimensions, 
                                                 const OdometryData& odometryData) :
  parameters(parameters),
  theGoalPercept(goalPercept),
  theLinePercept(linePercept),
  theFrameInfo(frameInfo),
  theFieldDimensions(fieldDimensions),
  theOdometryData(odometryData)
{
}

void SampleTemplateGenerator::init()
{
  realPostPositions[GoalPost::IS_LEFT] =
    Vector2<>((float) theFieldDimensions.xPosOpponentGoalpost, (float) theFieldDimensions.yPosLeftGoal);
  realPostPositions[GoalPost::IS_RIGHT] =
    Vector2<>((float) theFieldDimensions.xPosOpponentGoalpost, (float) theFieldDimensions.yPosRightGoal);
  fullGoals.init();
  knownGoalposts.init();
  unknownGoalposts.init();
}

void SampleTemplateGenerator::bufferNewPerceptions()
{
  if(!theGoalPercept.goalPosts.empty())
  {
    Vector2<> positionOnField1((float) theGoalPercept.goalPosts[0].positionOnField.x, 
                               (float) theGoalPercept.goalPosts[0].positionOnField.y);
 
    // Buffer data generated from GoalPercept:
    if(theGoalPercept.goalPosts.size() == 2)
    {
      Vector2<> positionOnField2((float) theGoalPercept.goalPosts[1].positionOnField.x, 
                                 (float) theGoalPercept.goalPosts[1].positionOnField.y);
      FullGoal newFullGoal;
      newFullGoal.realLeftPosition = realPostPositions[GoalPost::IS_LEFT];
      newFullGoal.realRightPosition = realPostPositions[GoalPost::IS_RIGHT];
      newFullGoal.seenLeftPosition = positionOnField1;
      newFullGoal.seenRightPosition = positionOnField2;
      newFullGoal.timestamp = theFrameInfo.time;
      newFullGoal.odometry = theOdometryData;

      // Before adding, check if templates can be generated from this perception
      SampleTemplate checkTemplate = generateTemplateFromFullGoal(newFullGoal);
      if(checkTemplate.timestamp)
        fullGoals.add(newFullGoal);
    }
    else
    {
      const GoalPost& post = theGoalPercept.goalPosts[0];
      if(post.position != GoalPost::IS_UNKNOWN)
      {
        // We might currently see a single goal post with known side (but not a complete goal)
        KnownGoalpost newPost;
        newPost.realPosition = realPostPositions[post.position];
        newPost.seenPosition = positionOnField1;
        newPost.timestamp = theFrameInfo.time;
        newPost.odometry = theOdometryData;
        newPost.centerCircleSeen = theLinePercept.circle.found;
        if(newPost.centerCircleSeen)
          newPost.centerCircleSeenPosition = Vector2<>((float)(theLinePercept.circle.pos.x), (float)(theLinePercept.circle.pos.y));
        knownGoalposts.add(newPost);
      }
      else
      {
        // Maybe we have seen some goalpost of which we do not know the side:
        UnknownGoalpost newPost;
        newPost.realPositions[0] = realPostPositions[GoalPost::IS_LEFT];
        newPost.realPositions[1] = realPostPositions[GoalPost::IS_RIGHT];
        newPost.seenPosition = positionOnField1;
        newPost.timestamp = theFrameInfo.time;
        newPost.odometry = theOdometryData;
        newPost.centerCircleSeen = theLinePercept.circle.found;
        if(newPost.centerCircleSeen)
          newPost.centerCircleSeenPosition = Vector2<>((float)(theLinePercept.circle.pos.x), (float)(theLinePercept.circle.pos.y));
        unknownGoalposts.add(newPost);
      }
    }
  }
  
  // If there are still some too old percepts after adding new ones -> delete them:
  removeOldPercepts(fullGoals);
  removeOldPercepts(knownGoalposts);
  removeOldPercepts(unknownGoalposts);
}

template<typename T> void SampleTemplateGenerator::removeOldPercepts(RingBuffer<T, MAX_PERCEPTS>& buffer)
{
  while(buffer.getNumberOfEntries())
  {
    T& oldestElement = buffer[buffer.getNumberOfEntries() - 1];
    if(theFrameInfo.getTimeSince(oldestElement.timestamp) > parameters.templateMaxKeepTime)
      buffer.removeFirst();
    else
      break;
  }
}

SampleTemplate SampleTemplateGenerator::getNewTemplate()
{
  SampleTemplate newTemplate;
  // Current solution: Prefer to construct templates from full goals only:
  if(fullGoals.getNumberOfEntries())
  {
    FullGoal& goal = fullGoals[rand() % fullGoals.getNumberOfEntries()];
    newTemplate = generateTemplateFromFullGoal(goal);
  }
  else if(knownGoalposts.getNumberOfEntries())
  {
    KnownGoalpost& goalPost = knownGoalposts[rand() % knownGoalposts.getNumberOfEntries()];
    if(goalPost.centerCircleSeen)
      newTemplate = generateTemplateFromPositionAndCenterCircle(goalPost.seenPosition, goalPost.centerCircleSeenPosition, goalPost.realPosition, goalPost.odometry);
    if(newTemplate.timestamp == 0)
      newTemplate = generateTemplateFromPosition(goalPost.seenPosition, goalPost.realPosition, goalPost.odometry);
  }
  else if(unknownGoalposts.getNumberOfEntries())
  {
    UnknownGoalpost& goalPost = unknownGoalposts[rand() % unknownGoalposts.getNumberOfEntries()];
    if(goalPost.centerCircleSeen)
      newTemplate = generateTemplateFromPositionAndCenterCircle(goalPost.seenPosition, goalPost.centerCircleSeenPosition, goalPost.realPositions[rand() % 2], goalPost.odometry);
    if(newTemplate.timestamp == 0)
      newTemplate = generateTemplateFromPosition(goalPost.seenPosition, goalPost.realPositions[rand() % 2], goalPost.odometry);
  }
  if(newTemplate.timestamp == 0) // In some cases, no proper sample is generated, return a random sample
    newTemplate = generateRandomTemplate();
  return newTemplate;
}

bool SampleTemplateGenerator::templatesAvailable() const
{
  const int sumOfTemplates = fullGoals.getNumberOfEntries() +
                             knownGoalposts.getNumberOfEntries() + unknownGoalposts.getNumberOfEntries();
  return sumOfTemplates > 0;
}

SampleTemplate SampleTemplateGenerator::generateTemplateFromFullGoal(const FullGoal& goal) const
{
  SampleTemplate newTemplate;
  Pose2D odometryOffset = theOdometryData - goal.odometry;
  float leftPostDist = goal.seenLeftPosition.abs();
  float leftDistUncertainty = sampleTriangularDistribution(parameters.standardDeviationGoalpostSampleBearingDistance);
  if(leftPostDist + leftDistUncertainty > parameters.standardDeviationGoalpostSampleBearingDistance)
    leftPostDist += leftDistUncertainty;
  float rightPostDist = goal.seenRightPosition.abs();
  float rightDistUncertainty = sampleTriangularDistribution(parameters.standardDeviationGoalpostSampleBearingDistance);
  if(rightPostDist + rightDistUncertainty > parameters.standardDeviationGoalpostSampleBearingDistance)
    rightPostDist += rightDistUncertainty;
  Geometry::Circle c1(goal.realLeftPosition, leftPostDist + theFieldDimensions.goalPostRadius);
  Geometry::Circle c2(goal.realRightPosition, rightPostDist + theFieldDimensions.goalPostRadius);
  // If there are intersections, take the first one that is in the field:
  Vector2<> p1, p2;
  int result = Geometry::getIntersectionOfCircles(c1, c2, p1, p2);
  if(result)
  {
    if(theFieldDimensions.isInsideCarpet(p1) && checkTemplateClipping(p1))
    {
      float origAngle = (goal.realLeftPosition - p1).angle();
      float observedAngle = goal.seenLeftPosition.angle();
      Pose2D templatePose(origAngle - observedAngle, p1);
      templatePose += odometryOffset;
      newTemplate = templatePose;
      newTemplate.timestamp = theFrameInfo.time;
    }
    else if(theFieldDimensions.isInsideCarpet(p2) && checkTemplateClipping(p2))
    {
      float origAngle = (goal.realLeftPosition - p2).angle();
      float observedAngle = goal.seenLeftPosition.angle();
      Pose2D templatePose(origAngle - observedAngle, p2);
      templatePose += odometryOffset;
      newTemplate = templatePose;
      newTemplate.timestamp = theFrameInfo.time;
    }
  }
  // The else case is omitted, calling function has to check the timestamp of the generated sample
  return newTemplate;
}

SampleTemplate SampleTemplateGenerator::generateTemplateFromPositionAndCenterCircle(const Vector2<>& posSeen, const Vector2<>& circlePosSeen,
    const Vector2<>& posReal, const Pose2D& postOdometry) const
{
  SampleTemplate newTemplate;
  Pose2D odometryOffset = theOdometryData - postOdometry;
  float postDist = posSeen.abs();
  float postDistUncertainty = sampleTriangularDistribution(parameters.standardDeviationGoalpostSampleBearingDistance);
  if(postDist + postDistUncertainty > parameters.standardDeviationGoalpostSampleBearingDistance)
    postDist += postDistUncertainty;
  float circleDist = circlePosSeen.abs();
  float circleDistUncertainty = sampleTriangularDistribution(parameters.standardDeviationGoalpostSampleBearingDistance); //No special uncertainty for center circle available
  if(circleDist + circleDistUncertainty > parameters.standardDeviationGoalpostSampleBearingDistance)
    circleDist += circleDistUncertainty;
  Geometry::Circle c1(posReal, postDist + theFieldDimensions.goalPostRadius);
  Geometry::Circle c2(Vector2<>(0.0f, 0.0f), circleDist);
  // If there are intersections, take the first one that is in the field:
  Vector2<> p1, p2;
  int result = Geometry::getIntersectionOfCircles(c1, c2, p1, p2);
  if(result)
  {
    if(theFieldDimensions.isInsideCarpet(p1) && checkTemplateClipping(p1))
    {
      float origAngle = (posReal - p1).angle();
      float observedAngle = posSeen.angle();
      Pose2D templatePose(origAngle - observedAngle, p1);
      templatePose += odometryOffset;
      newTemplate = templatePose;
      newTemplate.timestamp = theFrameInfo.time;
    }
    else if(theFieldDimensions.isInsideCarpet(p2) && checkTemplateClipping(p2))
    {
      float origAngle = (posReal - p2).angle();
      float observedAngle = posSeen.angle();
      Pose2D templatePose(origAngle - observedAngle, p2);
      templatePose += odometryOffset;
      newTemplate = templatePose;
      newTemplate.timestamp = theFrameInfo.time;
    }
  }
  // The else case is omitted, calling function has to check the timestamp of the generated sample
  return newTemplate;
}

SampleTemplate SampleTemplateGenerator::generateTemplateFromPosition(
  const Vector2<>& posSeen, const Vector2<>& posReal,
  const Pose2D& postOdometry) const
{
  SampleTemplate newTemplate;
  float r = posSeen.abs() + theFieldDimensions.goalPostRadius;
  float distUncertainty = sampleTriangularDistribution(parameters.standardDeviationGoalpostSampleBearingDistance);
  if(r + distUncertainty > parameters.standardDeviationGoalpostSampleBearingDistance)
    r += distUncertainty;
  Vector2<> realPosition = posReal;
  float minY = std::max(posReal.y - r, static_cast<float>(theFieldDimensions.yPosRightFieldBorder));
  float maxY = std::min(posReal.y + r, static_cast<float>(theFieldDimensions.yPosLeftFieldBorder));
  Vector2<> p;
  p.y = minY + randomFloat() * (maxY - minY);
  float xOffset(sqrt(sqr(r) - sqr(p.y - posReal.y)));
  p.x = posReal.x;
  p.x += (p.x > 0) ? -xOffset : xOffset;
  if(theFieldDimensions.isInsideCarpet(p) && checkTemplateClipping(p))
  {
    float origAngle = (realPosition - p).angle();
    float observedAngle = posSeen.angle();
    Pose2D templatePose(origAngle - observedAngle, p);
    Pose2D odometryOffset = theOdometryData - postOdometry;
    templatePose += odometryOffset;
    newTemplate = templatePose;
    newTemplate.timestamp = theFrameInfo.time;
  }
  return newTemplate;
}

SampleTemplate SampleTemplateGenerator::generateRandomTemplate() const
{
  SampleTemplate newTemplate;
  if(parameters.clipTemplateGeneration)
    newTemplate = Pose2D::random(parameters.clipTemplateGenerationRangeX, parameters.clipTemplateGenerationRangeY, Range<>(-pi, pi));
  else
    newTemplate = theFieldDimensions.randomPoseOnField();
  newTemplate.timestamp = theFrameInfo.time;
  return newTemplate;
}

void SampleTemplateGenerator::draw()
{
  for(int i = 0; i < fullGoals.getNumberOfEntries(); ++i)
  {
    FullGoal& goal = fullGoals[i];
    Pose2D odometryOffset = goal.odometry - theOdometryData;
    Vector2<> leftPost = odometryOffset * goal.seenLeftPosition;
    Vector2<> rightPost = odometryOffset * goal.seenRightPosition;
    LINE("module:SelfLocator:templates", leftPost.x, leftPost.y,
         rightPost.x, rightPost.y, 50, Drawings::ps_solid, ColorRGBA(140, 140, 255));
    CIRCLE("module:SelfLocator:templates", leftPost.x, leftPost.y,
           100, 20, Drawings::ps_solid, ColorRGBA(0, 0, 0), Drawings::bs_solid, ColorRGBA(140, 140, 255));
    CIRCLE("module:SelfLocator:templates", rightPost.x, rightPost.y,
           100, 20, Drawings::ps_solid, ColorRGBA(0, 0, 0), Drawings::bs_solid, ColorRGBA(140, 140, 255));
  }
  for(int i = 0; i < knownGoalposts.getNumberOfEntries(); ++i)
  {
    KnownGoalpost& post = knownGoalposts[i];
    Pose2D odometryOffset = post.odometry - theOdometryData;
    Vector2<> postPos = odometryOffset * post.seenPosition;
    CIRCLE("module:SelfLocator:templates", postPos.x, postPos.y,
           100, 20, Drawings::ps_solid, ColorRGBA(140, 140, 255), Drawings::bs_solid, ColorRGBA(140, 140, 255));
    CIRCLE("module:SelfLocator:templates", postPos.x, postPos.y,
           200, 20, Drawings::ps_solid, ColorRGBA(0, 0, 0), Drawings::bs_null, ColorRGBA(140, 140, 255));
  }
}

bool SampleTemplateGenerator::checkTemplateClipping(const Vector2<> pos) const
{
  if(!parameters.clipTemplateGeneration)
    return true;
  else
    return (parameters.clipTemplateGenerationRangeX.isInside(pos.x) && parameters.clipTemplateGenerationRangeY.isInside(pos.y));
}
