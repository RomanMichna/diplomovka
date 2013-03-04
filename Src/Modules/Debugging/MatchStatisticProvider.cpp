/**
* @file MatchStatisticProvider.cpp
* Implements a class that collects match information and statistics
* @author Tobias Kastner
*/

#include "MatchStatisticProvider.h"
#include "Tools/Streams/OutStreams.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Settings.h"
#include "Platform/File.h"
#include "Tools/Math/Geometry.h"
#include "Platform/SoundPlayer.h"
#include <string>

void MatchStatisticProvider::init()
{
  InConfigMap stream(Global::getSettings().expandLocationFilename("matchStatistic.cfg"));
  if(stream.exists())
    stream >> p;
  else
  {
    InConfigMap stream("matchStatistic.cfg");
    if(stream.exists())
      stream >> p;
  }
}

// load representation from file
void MatchStatisticProvider::load(MatchStatistic& ms, std::string robot)
{
  viewMode = true; // activate view mode -> do not modify/update this representation any more
  InConfigMap stream(File::getBHDir() + std::string("/Config/Statistics/") + robot);
  if(stream.exists())
  {
    stream >> ms;
    OUTPUT(idText, text, "Loaded MatchStatistic for " << robot.c_str());
    return;
  }
  else
  {
    viewMode = false;
    OUTPUT(idText, text, "Failed to load MatchStatistic for " << robot.c_str());
  }
}

// save representation to file
void MatchStatisticProvider::save(MatchStatistic& ms)
{
  char clicks[8];
  char htBuf[8];
  sprintf(clicks, "%u", timesSaved);
  sprintf(htBuf, "%u", ms.ht);

  OutConfigMap stream(File::getBHDir() + std::string("/Config/Statistics/") + Global::getSettings().robot +
                      std::string(htBuf) + std::string("_") + std::string(clicks) + std::string(".stx"));
  if(stream.exists())
  {
    stream << ms;
    OUTPUT(idText, text, "Saved MatchStatistic");
    ++timesSaved;
  }
  else
    OUTPUT(idText, text, "Failed to save MatchStatistic");
}

void MatchStatisticProvider::update(MatchStatistic& ms)
{
  std::string file("bla");
  MODIFY("module:MatchStatisticProvider:file", file);
  DEBUG_RESPONSE_ONCE("module:MatchStatisticProvider:load", load(ms, file););

  DEBUG_RESPONSE_ONCE("module:MatchStatisticProvider:save", save(ms););

  // only for analysis purposes.
  if(viewMode)
    return;

  // refresh some data
  // time elapsed from last update run in ms
  timePassed = theFrameInfo.time - lastTimeWhenUpdated;
  state = (int) theGameInfo.state;
  lastState = (int) lastGameInfo.state;

  ms.robotName = Global::getSettings().robot;
  ms.robotNumber = theRobotInfo.number;
  ms.data.teamColor = theOwnTeamInfo.teamColour;

  // set timestamp when this halftime ended
  if(lastState != STATE_FINISHED && state == STATE_FINISHED)
  {
    ms.data.timeWhenMatchFinishedThisHalfTime = theFrameInfo.time;
    ms.ht = theGameInfo.firstHalf ? 0 : 1;
  }

  if(p.saveStatistics && state == STATE_FINISHED && lastChestButton && !theKeyStates.pressed[KeyStates::chest] && theBehaviorControlOutput.behaviorData.relax)
  {
    save(ms);
    OUTPUT(idText, text, "Saved MatchStatistics stimulated by ChestButton");
    SoundPlayer::play("matchStatistics.wav");
    ms.reset();
  }
  lastChestButton = theKeyStates.pressed[KeyStates::chest];

  // do not collect data while initial statte
  if(state == STATE_INITIAL || state == STATE_FINISHED)
  {
    lastTimeWhenUpdated = theFrameInfo.time;
    ms.data.longestTimeWhenNoBallWasSeen = 0;
    return;
  }

  // collect data in ready, set and playing (and finished -> one frame) state
  updateCountableStuff(ms);
  lastOdometryData = theOdometryData;

  if(state != STATE_PLAYING && lastState != STATE_PLAYING)
  {
    lastTimeWhenUpdated = theFrameInfo.time;
    ms.data.longestTimeWhenNoBallWasSeen = 0;
    return;
  }

  // the following is only updated in playing state

  timePlayingNoGoalRobot += timePassed;
  timePlayingNoGoalTeam += timePassed;

  updateVisitedGrid(ms);
  updateBallStuff(ms);
  updateScoreInformation(ms);
  updateFallDownInformation(ms);
  updateSearchingInformation(ms);
  updatePenaltyInformation(ms);
  updateBehaviorInformation(ms);

  lastTimeWhenUpdated = theFrameInfo.time;
  lastGameInfo = theGameInfo;
  lastOwnTeamInfo = theOwnTeamInfo;
  lastMotionInfo = theMotionInfo;
  lastRobotInfo = theRobotInfo;
  lastBallWasSeen = theBallPercept.ballWasSeen;
  lastBehaviorData = theBehaviorControlOutput.behaviorData;
  lastSideConfidence = theSideConfidence;
}

void MatchStatisticProvider::updateCountableStuff(MatchStatistic& ms)
{
  // set timestamp when this halftime started
  if(lastState == STATE_SET && state == STATE_PLAYING && theGameInfo.secsRemaining >= 599)
    ms.data.timeWhenMatchStartedThisHalfTime = theFrameInfo.time;

  // add time wasted/being in ready or set state
  if(theGameInfo.secsRemaining < 600 && ((lastState == STATE_SET && state == STATE_SET) || (lastState == STATE_READY && state == STATE_READY)))
    ms.data.timeWastedNotBeingInPlayingState += timePassed;

  // add time elapsed without gameController packages
  if(lastTimeWhenUpdated - lastGameInfo.timeLastPackageReceived > p.timeSinceLastPackageReceiveWaitTime
    && theFrameInfo.getTimeSince(theGameInfo.timeLastPackageReceived) > (int) p.timeSinceLastPackageReceiveWaitTime)
    ms.data.timeWithoutGameController += timePassed;

  // update total walked distance in this halftime
  if((lastGroundContactState.contact && theGroundContactState.contact)
     || (lastRobotInfo.penalty == PENALTY_NONE && theRobotInfo.penalty == PENALTY_NONE))
  {
    float dist = (lastOdometryData - theOdometryData).translation.abs();
    ms.data.totalDistanceTraveled += dist;
    if(state == STATE_PLAYING && lastState == STATE_PLAYING)
      ms.data.distanceTraveledPlaying += dist;
  }

  if(theBehaviorControlOutput.behaviorData.role == lastBehaviorData.role && lastBehaviorData.role != BehaviorData::undefined)
    ms.data.roleTimes[(int) lastBehaviorData.role - 1] += timePassed;

  if(lastGroundContactState.contact && !theGroundContactState.contact)
    ms.data.timeWithoutGroundContact += timePassed;

  if(lastSideConfidence.confidenceState != SideConfidence::CONFUSED && theSideConfidence.confidenceState == SideConfidence::CONFUSED)
    ++ms.data.numOfLosts;
}

void MatchStatisticProvider::updateScoreInformation(MatchStatistic& ms)
{
  // find out wether a kick was performed this frame
  if(theMotionInfo.motion == MotionInfo::bike || (theMotionInfo.motion == MotionInfo::walk && theMotionInfo.walkRequest.kickType != WalkRequest::none))
    if(lastMotionInfo.motion == MotionRequest::stand || (lastMotionInfo.motion == MotionRequest::walk && lastMotionInfo.walkRequest.kickType == WalkRequest::none))
    {
      // backup kick information
      kickMightScore = true;
      timeWhenKickWasPerformed = theFrameInfo.time;
      poseWhenKicked = theRobotPose;
      motion = MotionRequest::stand;
      mirror = false;

      duelKick = theBehaviorControlOutput.behaviorData.activity == BehaviorData::duel;
      // count up total number of kicks except those performed while duelling.
      // but if this duel kick results in a goal, count it up later
      if(!duelKick)
        ms.data.numOfKicks++;
    }

  ScoreInfo::Effect effect = ScoreInfo::numOfEffects;

  // check for lasts kick effect -> goal, no goal, went out of the field ...
  bool goal = theOwnTeamInfo.score - lastOwnTeamInfo.score == 1;
  if(goal)
    effect = ScoreInfo::hit;
  else if(theFieldCoverage.throwIn && theGameInfo.dropInTeam == theOwnTeamInfo.teamColour)
    effect = ScoreInfo::wentOut;

  bool othersKicked = false;
  for(int i = theTeamMateData.firstPlayer; i < theTeamMateData.numOfPlayers; i++)
  {
    if(i == theRobotInfo.number)
      continue;
    if(!theTeamMateData.isPenalized[i] && theTeamMateData.isUpright[i] && theTeamMateData.hasGroundContact[i]
       && theTeamMateData.behaviorData[i].activity == BehaviorData::kick)
    {
      othersKicked = true;
      break;
    }
  }

  if(kickMightScore && othersKicked)
    kickMightScore = false;

  // just store information on succesful kicks or on those kicks causing the ball to leave the field (at the moment)
  if(theFrameInfo.getTimeSince(timeWhenKickWasPerformed) < p.maxTimeSinceKick && effect != ScoreInfo::numOfEffects && kickMightScore)
  {
    // update mean time to score of this robot
    if(effect == ScoreInfo::hit)
    {
      ms.data.robotScore++;
      ms.data.meanTimeToScoreRobot += timePlayingNoGoalRobot;
      timePlayingNoGoalRobot = 0;
      if(duelKick)
        ms.data.numOfKicks++;
    }

    ScoreInfo si;
    switch(motion)
    {
    case MotionRequest::bike:
      si = ScoreInfo(poseWhenKicked, timeWhenKickWasPerformed, theGameInfo.secsRemaining, motion, bikeID, effect, mirror);
      ms.data.scoreInformation.push_back(si);
      break;
    case MotionRequest::walk:
      si = ScoreInfo(poseWhenKicked, timeWhenKickWasPerformed, theGameInfo.secsRemaining, motion, walkKick, effect, mirror);
      ms.data.scoreInformation.push_back(si);
      break;
    default:
      break;
    }

    kickMightScore = false;
  }
  else if(theFrameInfo.getTimeSince(timeWhenKickWasPerformed) >= p.maxTimeSinceKick && kickMightScore)
    kickMightScore = false;


  // update mean time to score of this robots team even in case this goal was not scored by this robot
  if(goal)
  {
    ms.data.teamScore = theOwnTeamInfo.score;
    ms.data.meanTimeToScoreTeam += timePlayingNoGoalTeam;
    timePlayingNoGoalTeam = 0;
  }

}

void MatchStatisticProvider::updateFallDownInformation(MatchStatistic& ms)
{
  // are we going to perform a standup attempt
  if(lastMotionInfo.motion != MotionRequest::specialAction && theMotionInfo.motion == MotionRequest::specialAction)
    if(theMotionInfo.specialActionRequest.specialAction == SpecialActionRequest::standUpFrontNao
       || theMotionInfo.specialActionRequest.specialAction == SpecialActionRequest::standUpBackNao)
    {
      // was this fall down maybe caused by a kick
      if(theFrameInfo.getTimeSince(timeWhenKickWasPerformed) < p.maxTimeKickCauseFall)
      {
        FallDownInfo fdi(theRobotPose.translation, true, theMotionInfo.specialActionRequest.specialAction, motion, bikeID, walkKick, mirror);
        ms.data.fallDownInformation.push_back(fdi);
      }
      else
      {
        FallDownInfo fdi(theRobotPose.translation, false, theMotionInfo.specialActionRequest.specialAction, motion, bikeID, walkKick, mirror);
        ms.data.fallDownInformation.push_back(fdi);
      }
    }
}

void MatchStatisticProvider::updateVisitedGrid(MatchStatistic& ms)
{
  // update only if the desired amount of time has passed
  int timeSinceLastCycle = theFrameInfo.getTimeSince(gridUpdateCycle);

  // add new trace if this robot loses ground contact, was penalized
  // or is not in playing state any more
  if((lastGroundContactState.contact && !theGroundContactState.contact)
     || (lastRobotInfo.penalty == PENALTY_NONE && theRobotInfo.penalty != PENALTY_NONE)
     || (state != STATE_PLAYING && lastState == STATE_PLAYING))
    if(!ms.data.traceOnField.empty() && !ms.data.traceOnField.back().trace.empty())
    {
      ms.data.traceOnField.back().trace.push_back(TracePart(theRobotPose.translation, theBehaviorControlOutput.behaviorData.role));
      ms.data.traceOnField.push_back(Trace());
    }

  // criteria not to update the current trace
  if(!theGroundContactState.contact || theRobotInfo.penalty != PENALTY_NONE
     || state != STATE_PLAYING || theRobotPose.deviation > p.poseDeviationThreshold || timeSinceLastCycle < p.minCycleTime)
    return;

  gridUpdateCycle = theFrameInfo.time;

  // update grid/heatMap
  Vector2<> robotPos = theRobotPose.translation + Vector2<>((float) theFieldDimensions.xPosOpponentGroundline, (float) theFieldDimensions.yPosLeftSideline);
  int x = int(robotPos.x / HalfTimeInfo::gridCell);
  int y = int(robotPos.y / HalfTimeInfo::gridCell);
  x = x < 0 ? 0 : x;
  x = x > HalfTimeInfo::xSteps ? HalfTimeInfo::xSteps : x;
  y = y < 0 ? 0 : y;
  y = y > HalfTimeInfo::ySteps ? HalfTimeInfo::ySteps : y;

  if(ms.data.visitedFieldGrid[x * HalfTimeInfo::ySteps + y] < 255)
    ms.data.visitedFieldGrid[x * HalfTimeInfo::ySteps + y]++;

  // add new trace part to trace
  if(ms.data.traceOnField.empty())
  {
    ms.data.traceOnField.push_back(Trace());
    ms.data.traceOnField.back().trace.push_back(TracePart(theRobotPose.translation, theBehaviorControlOutput.behaviorData.role));
    return;
  }

  if(ms.data.traceOnField.back().trace.empty()
     || (theRobotPose.translation - ms.data.traceOnField.back().trace.back().position).abs() > p.minPositionChangeThreshold)
  {
    if(ms.data.traceOnField.back().trace.size() > 0)
      ms.data.traceOnField.back().length += (ms.data.traceOnField.back().trace.back().position - theRobotPose.translation).abs();

    ms.data.traceOnField.back().trace.push_back(TracePart(theRobotPose.translation, theBehaviorControlOutput.behaviorData.role));
  }
}

void MatchStatisticProvider::updateBallStuff(MatchStatistic& ms)
{
  // do not update ball information while "flying" or being penalized
  if(theRobotInfo.penalty != PENALTY_NONE || !theGroundContactState.contact)
  {
    ballNotSeenTime = 0;
    return;
  }

  // update time the ball was not seen
  if(!lastBallWasSeen && !theBallPercept.ballWasSeen)
    ballNotSeenTime += timePassed;

  if(!lastBallWasSeen && theBallPercept.ballWasSeen)
    if(ms.data.longestTimeWhenNoBallWasSeen < ballNotSeenTime)
    {
      ms.data.longestTimeWhenNoBallWasSeen = ballNotSeenTime;
      ballNotSeenTime = 0;
    }
}

void MatchStatisticProvider::updateSearchingInformation(MatchStatistic& ms)
{
  // find out wether this robot starts searching for the ball
  if(theBehaviorControlOutput.behaviorData.activity == BehaviorData::searchForBall && lastBehaviorData.activity != BehaviorData::searchForBall)
  {
    // backup information
    searchTime = theFrameInfo.time;
    poseWhenSearching = theRobotPose;
    return;
  }

  // this robot has stopped searching ... find out why (not necessarily because the ball was observed by this robot)
  if(theBehaviorControlOutput.behaviorData.activity != BehaviorData::patrol && theBehaviorControlOutput.behaviorData.activity != BehaviorData::searchForBall
     && (lastBehaviorData.activity == BehaviorData::patrol || lastBehaviorData.activity == BehaviorData::searchForBall))
  {
    // search stopped while patroling
    bool patrol = lastBehaviorData.activity == BehaviorData::patrol;

    if(ms.data.longestSearchForBallTime < (unsigned) theFrameInfo.getTimeSince(searchTime))
      ms.data.longestSearchForBallTime = (unsigned) theFrameInfo.getTimeSince(searchTime);

    // only store those searches that took more than given value
    if(theFrameInfo.getTimeSince(searchTime) > p.tolerableSearchTime)
    {
      SearchForBallInfo sfbi(poseWhenSearching, theFrameInfo.getTimeSince(searchTime), theBallPercept.ballWasSeen, patrol, Geometry::relative2FieldCoord(theRobotPose, theBallPercept.relativePositionOnField), theRobotPose);
      ms.data.searchForBallInformation.push_back(sfbi);
    }
  }
}

void MatchStatisticProvider::updatePenaltyInformation(MatchStatistic& ms)
{
  if(theRobotInfo.penalty != PENALTY_NONE && lastRobotInfo.penalty == PENALTY_NONE)
    ms.data.penaltyInformation.push_back(PenaltyInfo(theRobotInfo.penalty, theGameInfo.secsRemaining));

  if(theRobotInfo.penalty != PENALTY_NONE && lastRobotInfo.penalty != PENALTY_NONE)
    ms.data.timeBeingPenalized += timePassed;
}

void MatchStatisticProvider::updateBehaviorInformation(MatchStatistic& ms)
{
  if(lastBehaviorData.activity == BehaviorData::duel && theBehaviorControlOutput.behaviorData.activity == BehaviorData::duel)
    ms.data.duelTime += timePassed;
}

MAKE_MODULE(MatchStatisticProvider, Debugging)
