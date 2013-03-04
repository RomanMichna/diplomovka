/**
 * @file OwnSideModelProvider.h
 * The file implements a module that determines whether the robot cannot have left its own
 * side since the last kick-off.
 *
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas R�fer</a>
 */

#include "OwnSideModelProvider.h"
#include "Representations/Infrastructure/TeamMateData.h"
#include "Tools/Settings.h"
#include "Tools/Debugging/Debugging.h"
#include "Tools/Streams/InStreams.h"

OwnSideModelProvider::OwnSideModelProvider() :
  lastPenalty(PENALTY_NONE),
  lastGameState(STATE_INITIAL),
  distanceWalkedAtKnownPosition(0),
  largestXPossibleAtKnownPosition(0),
  manuallyPlaced(false),
  timeWhenPenalized(0),
  gameStateWhenPenalized(STATE_INITIAL)
{
  InConfigMap stream(Global::getSettings().expandLocationFilename("ownSideModel.cfg"));
  ASSERT(stream.exists());
  stream >> parameters;
}

void OwnSideModelProvider::update(OwnSideModel& ownSideModel)
{
  if(theGameInfo.state == STATE_SET && !theGroundContactState.contact)
    manuallyPlaced = true;
  
  if(lastPenalty == PENALTY_NONE && theRobotInfo.penalty != PENALTY_NONE)
  {
    timeWhenPenalized = theFrameInfo.time;
    gameStateWhenPenalized = theGameInfo.state;
  }
  
  ownSideModel.returnFromGameControllerPenalty = false;
  ownSideModel.returnFromManualPenalty = false;

  if(theGameInfo.secondaryState != STATE2_PENALTYSHOOT)
  {
    if(lastGameState == STATE_INITIAL && theGameInfo.state != STATE_INITIAL)
    {
      distanceWalkedAtKnownPosition = theOdometer.distanceWalked;
      largestXPossibleAtKnownPosition = parameters.largestXInInitial;
    }
    else if(lastPenalty == PENALTY_MANUAL && theRobotInfo.penalty == PENALTY_NONE)
    {
      distanceWalkedAtKnownPosition = theOdometer.distanceWalked;
      largestXPossibleAtKnownPosition = -theFieldDimensions.centerCircleRadius - parameters.awayFromLineDistance;
      ownSideModel.returnFromManualPenalty = true;
    }
    else if(lastPenalty != PENALTY_NONE && theRobotInfo.penalty == PENALTY_NONE &&
            (theFrameInfo.getTimeSince(timeWhenPenalized) > parameters.minPenaltyTime || theGameInfo.state != gameStateWhenPenalized))
    {
      distanceWalkedAtKnownPosition = theOdometer.distanceWalked;
      largestXPossibleAtKnownPosition = (float) theFieldDimensions.xPosOwnPenaltyMark;
      ownSideModel.returnFromGameControllerPenalty = true;
    }
    else if(lastGameState == STATE_SET && theGameInfo.state == STATE_PLAYING)
    {
      distanceWalkedAtKnownPosition = theOdometer.distanceWalked;
      if(manuallyPlaced)
      {
        if(theRobotInfo.number == TeamMateData::firstPlayer)
          largestXPossibleAtKnownPosition = (float) theFieldDimensions.xPosOwnGroundline;
        else if(theGameInfo.kickOffTeam == theOwnTeamInfo.teamColor)
          largestXPossibleAtKnownPosition = (float) -theFieldDimensions.centerCircleRadius - parameters.awayFromLineDistance;
        else
          largestXPossibleAtKnownPosition = (float) theFieldDimensions.xPosOwnPenaltyArea + parameters.awayFromLineDistance;
      }
      else if(theRobotInfo.number == TeamMateData::firstPlayer)
        largestXPossibleAtKnownPosition = (float) theFieldDimensions.xPosOwnPenaltyArea;
      else if(theGameInfo.kickOffTeam == theOwnTeamInfo.teamColor)
        largestXPossibleAtKnownPosition = -parameters.awayFromLineDistance;
      else
        largestXPossibleAtKnownPosition = (float) theFieldDimensions.xPosOwnPenaltyMark - parameters.awayFromLineDistance;
    }
  }
  else if(lastGameState == STATE_SET)
  {
    distanceWalkedAtKnownPosition = theOdometer.distanceWalked;
    if(theGameInfo.kickOffTeam == theOwnTeamInfo.teamColor)
      largestXPossibleAtKnownPosition = 0;
    else
      largestXPossibleAtKnownPosition = (float) theFieldDimensions.xPosOwnGroundline;
  }
  ownSideModel.largestXPossible = largestXPossibleAtKnownPosition + parameters.distanceUncertaintyOffset + 
                                  (theOdometer.distanceWalked - distanceWalkedAtKnownPosition) * parameters.distanceUncertaintyFactor;
  ownSideModel.stillInOwnSide = ownSideModel.largestXPossible < 0;
  lastPenalty = theRobotInfo.penalty;
  lastGameState = theGameInfo.state;
  
  if(theGameInfo.state != STATE_SET)
    manuallyPlaced = false;
}

MAKE_MODULE(OwnSideModelProvider, Modeling)
