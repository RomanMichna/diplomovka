/**
* @file GameStateHandler.cpp
*
* This file implements a submodule correcting localization based on GameController info.
*
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#include "GameStateHandler.h"
#include "Tools/Debugging/Debugging.h"
#include "Representations/Infrastructure/TeamMateData.h"
#include "Platform/SoundPlayer.h"

GameStateHandler::GameStateHandler(const SelfLocatorParameters& parameters,
                                   const FieldDimensions& fieldDimensions,
                                   const GameInfo& gameInfo,
                                   const OwnTeamInfo& ownTeamInfo,
                                   const RobotInfo& robotInfo,
                                   const RobotPose& robotPose,
                                   const OdometryData& odometryData,
                                   const SideConfidence& sideConfidence,
                                   const OwnSideModel& ownSideModel,
                                   const GroundContactState& groundContactState) :
  parameters(parameters),
  theFieldDimensions(fieldDimensions),
  theGameInfo(gameInfo),
  theOwnTeamInfo(ownTeamInfo),
  theRobotInfo(robotInfo),
  theRobotPose(robotPose),
  theOdometryData(odometryData),
  theSideConfidence(sideConfidence),
  theOwnSideModel(ownSideModel),
  theGroundContactState(groundContactState),
  lastPenalty(PENALTY_NONE),
  lastGameState(-1),
  lastGroundContactState(false),
  wasValidatedBefore(false)
{
}

void GameStateHandler::preProcess()
{
  if(theRobotPose.validity == 1.f || !wasValidatedBefore)
    lastRobotPose = theRobotPose;
  else
    lastRobotPose += theOdometryData - lastOdometryData;
  
  if(theSideConfidence.mirror) // side confidence is about last frame
  {
    lastRobotPose = Pose2D(pi) + lastRobotPose;
    SoundPlayer::play("themirrorcow.wav");
  }
}

bool GameStateHandler::shouldRedistribute(std::vector<Pose2D>& poses, std::vector<Pose2D>& standardDeviations) const
{
  poses.clear();
  standardDeviations.clear();
  
  if(theGameInfo.secondaryState == STATE2_PENALTYSHOOT)
  {
    // penalty shoot: if game state switched to playing reset samples to start pos
    if((lastGameState == STATE_SET && theGameInfo.state == STATE_PLAYING) ||
       (lastPenalty != PENALTY_NONE && theRobotInfo.penalty == PENALTY_NONE))
    {
      //this only works for the penalty shootout, but since there's no
      //way to detect a penalty shootout from within the game..... =|
      if(theGameInfo.kickOffTeam == theOwnTeamInfo.teamColour)
      {
        //striker pose (center of the pitch)
        poses.push_back(Pose2D(0.f, 0.f, 0.f));
        standardDeviations.push_back(Pose2D(0.2f, 200.f, 200.f));
      }
      else
      {
        //goalie pose (in the center of the goal)
        poses.push_back(Pose2D(0.f, (float) theFieldDimensions.xPosOwnGroundline, 0.f));
        standardDeviations.push_back(Pose2D(0.2f, 200.f, 200.f));
      }
      return true;
    }
  }
  else if(theOwnSideModel.returnFromGameControllerPenalty || theOwnSideModel.returnFromManualPenalty)
  {
    // normal game: if penalty is over reset samples to reenter positions
    poses.push_back(Pose2D(-pi_2, (float) theFieldDimensions.xPosOwnPenaltyMark, (float) theFieldDimensions.yPosLeftSideline));
    poses.push_back(Pose2D(pi_2, (float) theFieldDimensions.xPosOwnPenaltyMark, (float) theFieldDimensions.yPosRightSideline));
    standardDeviations.push_back(Pose2D(0.2f, 200.f, 200.f));
    standardDeviations.push_back(Pose2D(0.2f, 200.f, 200.f));
    return true;
  }
  else if(lastGameState != STATE_INITIAL && theGameInfo.state == STATE_INITIAL)
  {
    // normal game: we start on the sidelines looking at our goal
    for(int i = 0; i < 4; ++i)
    {
      poses.push_back(Pose2D(fromDegrees(-133.f), -700.f - (float) i * 500.f, (float) theFieldDimensions.yPosLeftSideline));
      poses.push_back(Pose2D(fromDegrees(133.f), -700.f - (float) i * 500.f, (float) theFieldDimensions.yPosRightSideline));
      standardDeviations.push_back(Pose2D(0.3f, 400, 200));
      standardDeviations.push_back(Pose2D(0.3f, 400, 200));
    }
    return true;
  }
  else if(lastGameState == STATE_SET && !lastGroundContactState &&
          (theGroundContactState.contact || theGameInfo.state == STATE_PLAYING))
  {
    // normal game: manual placement
    if(theRobotInfo.number == TeamMateData::firstPlayer)
      poses.push_back(Pose2D(0.f, (float) theFieldDimensions.xPosOwnGroundline, 0.f));
    else if(theGameInfo.kickOffTeam == theOwnTeamInfo.teamColor)
    {
      poses.push_back(Pose2D(0.f, (float) -theFieldDimensions.centerCircleRadius - 100, 0.f));
      poses.push_back(Pose2D(0.f, (float) theFieldDimensions.xPosOwnPenaltyMark, (float) theFieldDimensions.yPosLeftPenaltyArea));
      poses.push_back(Pose2D(0.f, (float) theFieldDimensions.xPosOwnPenaltyMark, (float) theFieldDimensions.yPosRightPenaltyArea));
      poses.push_back(Pose2D(0.f, (float) theFieldDimensions.xPosOwnPenaltyArea + 100, (float) theFieldDimensions.yPosLeftPenaltyArea));
      poses.push_back(Pose2D(0.f, (float) theFieldDimensions.xPosOwnPenaltyArea + 100, (float) theFieldDimensions.yPosRightPenaltyArea));
    }
    else
    {
      poses.push_back(Pose2D(0.f, (float) theFieldDimensions.xPosOwnPenaltyArea + 100, (float) (theFieldDimensions.yPosLeftSideline + theFieldDimensions.yPosLeftPenaltyArea) / 2.f));
      poses.push_back(Pose2D(0.f, (float) theFieldDimensions.xPosOwnPenaltyArea + 100, (float) theFieldDimensions.yPosLeftGoal / 2.f));
      poses.push_back(Pose2D(0.f, (float) theFieldDimensions.xPosOwnPenaltyArea + 100, (float) theFieldDimensions.yPosRightGoal / 2.f));
      poses.push_back(Pose2D(0.f, (float) theFieldDimensions.xPosOwnPenaltyArea + 100, (float) (theFieldDimensions.yPosRightSideline + theFieldDimensions.yPosRightPenaltyArea) / 2.f));
    }
    for(unsigned i = 0; i < poses.size(); ++i)
      standardDeviations.push_back(Pose2D(0.2f, 200, 200));
    return true;
  }
  return false;
}

bool GameStateHandler::isMirrorCloser(const Pose2D& currentPose, const Pose2D& lastPose) const
{
  const Vector2<>& translation = currentPose.translation;
  Vector2<> rotationWeight(std::max(parameters.useRotationThreshold - std::min(translation.abs(), lastPose.translation.abs()), 0.f), 0);
  Vector2<> rotation = Pose2D(currentPose.rotation) * rotationWeight;
  Vector2<> lastRotation = Pose2D(lastPose.rotation) * rotationWeight;
  
  return (lastPose.translation - translation).abs() + (lastRotation - rotation).abs() >
         (lastPose.translation + translation).abs() + (lastRotation + rotation).abs();
}

bool GameStateHandler::shouldMirror(const Pose2D& pose) const
{
  if(theGameInfo.secondaryState == STATE2_PENALTYSHOOT && theGameInfo.kickOffTeam == theOwnTeamInfo.teamColor &&
     (lastGameState == STATE_SET || lastPenalty == PENALTY_MANUAL))
    return std::abs(pose.rotation) > pi_2;
  else if(lastGameState == STATE_SET && theGameInfo.kickOffTeam == theOwnTeamInfo.teamColor)
    return (pose * Vector2<>(-parameters.ownHalfOffensiveKickOffTolerance, 0)).x > 0;
  else if(lastGameState == STATE_INITIAL || lastGameState == STATE_SET || theOwnSideModel.stillInOwnSide)
    return pose.translation.x > 0;
  else
    return isMirrorCloser(pose, lastRobotPose);
}

void GameStateHandler::postProcess(bool wasMirroredOrRedistributed)
{
  lastPenalty = theRobotInfo.penalty;
  lastGameState = theGameInfo.state;
  lastOdometryData = theOdometryData;
  lastGroundContactState = theGroundContactState.contact;

  if(wasMirroredOrRedistributed)
    wasValidatedBefore = false;
  else if(theRobotPose.validity == 1.f)
    wasValidatedBefore = true;
}
