/**
* @file RobotPoseValidator.cpp
 * Implements a module that validates poses from the self locator on a field
 * with yellow goals.
 * @author Colin Graf
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#include "RobotPoseValidator.h"
#include "Tools/Debugging/DebugDrawings.h"

MAKE_MODULE(RobotPoseValidator, Modeling)

RobotPoseValidator::RobotPoseValidator() :
  validated(false), validGoalSightingSinceLastReset(0), validUnknownGoalSightingSinceLastReset(0), lastFilterResetTime(0)
{
  p.validationMaxDeviation = Pose2D(0.1f, 100.f, 100.f);
  p.validationMaxGoalPerceptDistance = 0.1f;
  p.validationMinGoalSightings = 6;
  p.validationMinUnknownGoalSightings = 10;
}

  void RobotPoseValidator::init()
{
  // and goal posts
  goalPosts[OPPONENT_TEAM][GoalPost::IS_LEFT] = Vector2<>(float(theFieldDimensions.xPosOpponentGoalpost), float(theFieldDimensions.yPosLeftGoal));
  goalPosts[OPPONENT_TEAM][GoalPost::IS_RIGHT] = Vector2<>(float(theFieldDimensions.xPosOpponentGoalpost), float(theFieldDimensions.yPosRightGoal));
  goalPosts[OWN_TEAM][GoalPost::IS_LEFT] = Vector2<>(float(theFieldDimensions.xPosOwnGoalpost), float(theFieldDimensions.yPosRightGoal));
  goalPosts[OWN_TEAM][GoalPost::IS_RIGHT] = Vector2<>(float(theFieldDimensions.xPosOwnGoalpost), float(theFieldDimensions.yPosLeftGoal));
}

void RobotPoseValidator::update(RobotPose& robotPose)
{
  // reset validation?
  if(lastFilterResetTime != theFilteredRobotPose.filterResetTime ||
     !theGroundContactState.contact ||
     theFallDownState.state != theFallDownState.upright)
  {
    validated = false;
  }
  lastFilterResetTime = theFilteredRobotPose.filterResetTime;

  // try to validate the pose via goal posts
  if(!validated)
  {
    for(int i = 0, count = theGoalPercept.goalPosts.size(); i < count; ++i)
    {
      const GoalPost& post = theGoalPercept.goalPosts[i];
      Vector2<> seenPositionOnField = theFilteredRobotPose * Vector2<>(float(post.positionOnField.x), float(post.positionOnField.y));
      Membership side = seenPositionOnField.x > 0 ? OPPONENT_TEAM : OWN_TEAM;
      if(post.position != GoalPost::IS_UNKNOWN)
      {
        const Vector2<>& realPositionOnField = goalPosts[side][post.position];
        if(abs(realPositionOnField.x - theFilteredRobotPose.translation.x) > float(theFieldDimensions.xPosOpponentGroundline - theFieldDimensions.xPosOpponentPenaltyArea) &&
          (seenPositionOnField - realPositionOnField).squareAbs() / (realPositionOnField - theFilteredRobotPose.translation).squareAbs() < sqr(p.validationMaxGoalPerceptDistance))
          validGoalSightingSinceLastReset++;
      }
      else
      {
        const Vector2<>& realPositionOnField1 = goalPosts[side][GoalPost::IS_LEFT];
        const Vector2<>& realPositionOnField2 = goalPosts[side][GoalPost::IS_RIGHT];
        if(abs(realPositionOnField1.x - theFilteredRobotPose.translation.x) > float(theFieldDimensions.xPosOpponentGroundline - theFieldDimensions.xPosOpponentPenaltyArea) &&
           (seenPositionOnField - realPositionOnField1).squareAbs() / (realPositionOnField1 - theFilteredRobotPose.translation).squareAbs() < sqr(p.validationMaxGoalPerceptDistance))
          validUnknownGoalSightingSinceLastReset++;
        if(abs(realPositionOnField2.x - theFilteredRobotPose.translation.x) > float(theFieldDimensions.xPosOpponentGroundline - theFieldDimensions.xPosOpponentPenaltyArea) &&
           (seenPositionOnField - realPositionOnField2).squareAbs() / (realPositionOnField2 - theFilteredRobotPose.translation).squareAbs() < sqr(p.validationMaxGoalPerceptDistance))
          validUnknownGoalSightingSinceLastReset++;
      }
    }
    if((validGoalSightingSinceLastReset >= p.validationMinGoalSightings || validUnknownGoalSightingSinceLastReset >= p.validationMinUnknownGoalSightings) &&
       theFilteredRobotPose.cov[0][0] <= sqr(p.validationMaxDeviation.translation.x) &&
       theFilteredRobotPose.cov[1][1] <= sqr(p.validationMaxDeviation.translation.y) &&
       theFilteredRobotPose.cov[2][2] <= sqr(p.validationMaxDeviation.rotation))
      validated = true;
  }

  // generate model
  robotPose.rotation = theFilteredRobotPose.rotation;
  robotPose.translation = theFilteredRobotPose.translation;
  if(validated)
  {
    robotPose.validity = 1.f;
    robotPose.deviation = sqrt(std::max(theFilteredRobotPose.cov[0].x, theFilteredRobotPose.cov[1].y));
  }
  else
  {
    robotPose.validity = thePotentialRobotPose.validity;
    robotPose.deviation = RobotPose::unknownDeviation;
  }

  EXECUTE_ONLY_IN_DEBUG(robotPose.draw(theOwnTeamInfo.teamColor != TEAM_BLUE););
}
