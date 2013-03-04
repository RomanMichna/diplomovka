/**
 * @file Controller/GameController.h
 * This file implements a class that simulates a console-based GameController.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "GameController.h"
#include "Oracle.h"
#include "Platform/SystemCall.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/BallModel.h"
#include "Tools/Streams/InStreams.h"
#include "Platform/BHAssert.h"
#include <limits>

FieldDimensions GameController::fieldDimensions;

GameController::GameController() :
timeOfLastDropIn(0),
timeWhenLastRobotMoved(0),
timeWhenStateBegan(0),
automatic(true)
{
  gameInfo.playersPerTeam = numOfRobots / 2;
  gameInfo.firstHalf = 1;
  gameInfo.dropInTime = -1;
  gameInfo.secsRemaining = 600;
  teamInfos[TEAM_BLUE].teamNumber = 1;
  teamInfos[TEAM_RED].teamNumber = 2;
  teamInfos[TEAM_RED].teamColor = TEAM_RED;
}

void GameController::registerOracle(int robot, Oracle &oracle)
{
  ASSERT(!robots[robot].oracle);
  robots[robot].oracle = &oracle;
  if(!fieldDimensions.xPosOwnPenaltyMark)
    fieldDimensions.load();
}

bool GameController::handleGlobalCommand(const std::string& command)
{
  if(command == "initial")
  {
    gameInfo.state = STATE_INITIAL;
    timeOfLastDropIn = 0;
    gameInfo.secsRemaining = 600;
    return true;
  }
  else if(command == "ready")
  {
    gameInfo.state = STATE_READY;
    for(int i = 0; i < numOfRobots; ++i)
      if(robots[i].info.penalty)
        handleRobotCommand(i, "none");
    timeWhenStateBegan = SystemCall::getCurrentSystemTime();
    return true;
  } 
  else if(command == "set")
  {
    gameInfo.state = STATE_SET;
    for(int i = 0; i < numOfRobots; ++i)
      robots[i].info.penalty = none;

    if(automatic)
    {
      placeGoalie(0);
      placeGoalie(numOfRobots / 2);
      placeDefensivePlayers(gameInfo.kickOffTeam == TEAM_BLUE ? numOfRobots / 2 + 1 : 1);
      placeOffensivePlayers(gameInfo.kickOffTeam == TEAM_BLUE ? 1 : numOfRobots / 2 + 1);
      executePlacement();
    }

    timeWhenStateBegan = SystemCall::getCurrentSystemTime();
    Oracle::moveBall(Vector3<>(0.f, 0.f, 50.f), true);
    return true;
  }
  else if(command == "playing")
  {
    gameInfo.state = STATE_PLAYING;
    timeWhenHalfStarted = SystemCall::getCurrentSystemTime() - (600 - gameInfo.secsRemaining) * 1000;
    return true;
  }
  else if(command == "finished")
  {
    gameInfo.state = STATE_FINISHED;
    return true;
  }
  else if(command == "kickOffBlue")
  {
    gameInfo.kickOffTeam = TEAM_BLUE;
    return true;
  }
  else if(command == "kickOffRed")
  {
    gameInfo.kickOffTeam = TEAM_RED;
    return true;
  }
  else if(command == "outByBlue")
  {
    gameInfo.dropInTeam = TEAM_BLUE;
    timeOfLastDropIn = SystemCall::getCurrentSystemTime();
    return true;
  }
  else if(command == "outByRed")
  {
    gameInfo.dropInTeam = TEAM_RED;
    timeOfLastDropIn = SystemCall::getCurrentSystemTime();
    return true;
  }
  return false;
}

bool GameController::handleGlobalConsole(In& stream)
{
  SYNC;
  std::string command;
  stream >> command;
  return handleGlobalCommand(command);
}
  
bool GameController::handleRobotCommand(int robot, const std::string& command)
{
  Robot& r = robots[robot];
  for(int i = 0; i < numOfPenalties; ++i)
    if(command == getName((Penalty) i))
    {
      r.info.penalty = i == manual ? PENALTY_MANUAL : i;
      if(i)
      {
        r.timeWhenPenalized = SystemCall::getCurrentSystemTime();
        if(automatic)
          placeForPenalty(robot, (float) fieldDimensions.xPosOpponentPenaltyMark,
                          (float) (fieldDimensions.yPosRightFieldBorder + 100), -pi_2);
      }
      else if(automatic)
      {
        BallModel ballModel;
        ASSERT(r.oracle);
        r.oracle->getBallModel(RobotPose(), ballModel);
        Vector2<> ballPos = ballModel.lastPerception;
        if(robot < numOfRobots / 2)
          ballPos = -ballPos;
        placeForPenalty(robot, (float) fieldDimensions.xPosOpponentPenaltyMark,
                        ballPos.y >= 0 ? (float) fieldDimensions.yPosRightSideline : (float) fieldDimensions.yPosLeftSideline,
                        ballPos.y >= 0 ? pi_2 : -pi_2);
      }
      return true;
    }
  return false;
}

bool GameController::handleRobotConsole(int robot, In& stream)
{
  SYNC;
  std::string command;
  stream >> command;
  return handleRobotCommand(robot, command);
}  

void GameController::placeForPenalty(int robot, float x, float y, float rotation)
{
  Robot& r = robots[robot];
  ASSERT(r.oracle);
  Vector2<> newPos(robot < numOfRobots / 2 ? x : -x, y);
  for(;;)
  {
    int j = 0;
    while(j < numOfRobots && 
          (j == robot || !robots[j].oracle || (robots[j].lastPose.translation - newPos).abs() >= 300))
      ++j;
    if(j == numOfRobots)
    {
      r.lastPose = Pose2D(rotation, newPos.x, newPos.y);
      r.oracle->moveRobot(Vector3<>(newPos.x, newPos.y, 350), Vector3<>(0, 0, rotation), true);
      break;
    }
    else
      newPos.x += newPos.x < 0 ? -400 : 400;
  }
}

bool GameController::inOwnPenaltyArea(int robot) const
{
  const Robot& r = robots[robot];
  if(r.lastPose.translation.y < fieldDimensions.yPosRightPenaltyArea ||
     r.lastPose.translation.y > fieldDimensions.yPosLeftPenaltyArea)
    return false;
  else if(robot < numOfRobots / 2)
    return r.lastPose.translation.x >= fieldDimensions.xPosOpponentPenaltyArea &&
           (r.lastPose.translation.x <= fieldDimensions.xPosOpponentGroundline || 
            (r.lastPose.translation.x <= fieldDimensions.xPosOpponentGoal &&
             r.lastPose.translation.y >= fieldDimensions.yPosRightGoal &&
             r.lastPose.translation.y <= fieldDimensions.yPosLeftGoal));
  else
    return r.lastPose.translation.x <= fieldDimensions.xPosOwnPenaltyArea &&
           (r.lastPose.translation.x >= fieldDimensions.xPosOwnGroundline || 
            (r.lastPose.translation.x >= fieldDimensions.xPosOwnGoal &&
             r.lastPose.translation.y >= fieldDimensions.yPosRightGoal &&
             r.lastPose.translation.y <= fieldDimensions.yPosLeftGoal));
}

void GameController::placeGoalie(int robot)
{
  Robot& r = robots[robot];
  r.manuallyPlaced = r.oracle && !inOwnPenaltyArea(robot);
  if(r.manuallyPlaced)
    r.lastPose = robot < numOfRobots / 2 ? Pose2D(-pi, (float) fieldDimensions.xPosOpponentGroundline, 0) 
                                         : Pose2D(0, (float) fieldDimensions.xPosOwnGroundline, 0);
}

void GameController::placeFromSet(int robot, int minRobot, const Pose2D* poses, int numOfPoses)
{
  ASSERT(numOfPoses <= numOfFieldPlayers);
  
  // For finding a manual placement pose, it is determined which
  // of the positions would be chosen by our teammates. 
  bool occupied[numOfFieldPlayers] = {false};
  for(int i = minRobot; i < minRobot + numOfFieldPlayers; ++i)
    if(i != robot && robots[i].oracle)
    {
      const Robot& r2 = robots[i];
      float minDistance = numeric_limits<float>::max();
      int bestPoseIndex = 0;
      for(int j = 0; j < numOfPoses; ++j)
      {
        const Pose2D& pose = poses[j];
        float distance = (pose.translation - r2.lastPose.translation).abs();
        if(!occupied[j] && distance < minDistance)
        {
          minDistance = distance;
          bestPoseIndex = j;
        }
      }
      occupied[bestPoseIndex] = true;
    }
  
  // The position that would not be chosen is suitable for this robot.
  int i = 0;
  while(i < numOfPoses && occupied[i])
    ++i;
  ASSERT(i < numOfFieldPlayers);
  robots[robot].lastPose = poses[i];
}

void GameController::placeOffensivePlayers(int minRobot)
{
  static const Pose2D poses[2][numOfFieldPlayers] =
  {
    {
      Pose2D(0, (float) -fieldDimensions.centerCircleRadius - 120, 0),
      Pose2D(0, (float) fieldDimensions.xPosOwnPenaltyMark + 150, (float) fieldDimensions.yPosLeftPenaltyArea),
      Pose2D(0, (float) fieldDimensions.xPosOwnPenaltyArea + 150, (float) fieldDimensions.yPosRightPenaltyArea)
    },
    {
      Pose2D(-pi, (float) fieldDimensions.centerCircleRadius + 120, 0),
      Pose2D(-pi, (float) fieldDimensions.xPosOpponentPenaltyMark, (float) fieldDimensions.yPosRightPenaltyArea),
      Pose2D(-pi, (float) fieldDimensions.xPosOpponentPenaltyArea - 150, (float) fieldDimensions.yPosLeftPenaltyArea)
    }
  };
  
  // Move all field players that are not in their own half or in their penalty area.
  // Count robots in center circle
  int numOfRobotsInCenterCircle = 0;
  float minDistance = numeric_limits<float>::max();
  int indexOfMinDistance = 0;
  for(int i = minRobot; i < minRobot + 3; ++i)
  {
    Robot& r = robots[i];
    r.manuallyPlaced = false;
    r.manuallyPlaced = r.oracle && 
    (inOwnPenaltyArea(i) ||
     r.lastPose.translation.y < fieldDimensions.yPosRightSideline ||
     r.lastPose.translation.y > fieldDimensions.yPosLeftSideline ||
     (i < numOfRobots / 2 && (r.lastPose.translation.x < 120 ||
                              r.lastPose.translation.x > fieldDimensions.xPosOpponentGroundline)) ||
     (i >= numOfRobots / 2 && (r.lastPose.translation.x > -120 ||
                               r.lastPose.translation.x < fieldDimensions.xPosOwnGroundline)));
    if(r.manuallyPlaced)
      placeFromSet(i, minRobot, poses[i < numOfRobots / 2 ? 1 : 0], numOfFieldPlayers);
    else
    {
      float distance = r.lastPose.translation.abs();
      if(distance < fieldDimensions.centerCircleRadius + 120)
      {
        ++numOfRobotsInCenterCircle;
        if(distance < minDistance)
        {
          minDistance = distance;
          indexOfMinDistance = i;
        }
      }
    }
  }
  
  // If there is more than one robot in center circle, keep the closest
  // to the field center and move the others away. 
  if(numOfRobotsInCenterCircle > 1)
    for(int i = minRobot; i < minRobot + 3; ++i)
    {
      Robot& r = robots[i];
      if(i != indexOfMinDistance && r.lastPose.translation.abs() < fieldDimensions.centerCircleRadius + 120)
        placeFromSet(i, minRobot, poses[i < numOfRobots / 2 ? 1 : 0] + 1, numOfFieldPlayers - 1);
    }
}

void GameController::placeDefensivePlayers(int minRobot)
{
  static const Pose2D poses[2][numOfFieldPlayers] =
  {
    {
      Pose2D(0, (float) fieldDimensions.xPosOwnPenaltyArea + 150, 350),
      Pose2D(0, (float) fieldDimensions.xPosOwnPenaltyArea + 150, 1600),
      Pose2D(0, (float) fieldDimensions.xPosOwnPenaltyArea + 150, -1600)
    },
    {
      Pose2D(-pi, (float) fieldDimensions.xPosOpponentPenaltyArea - 150, -350),
      Pose2D(-pi, (float) fieldDimensions.xPosOpponentPenaltyArea - 150, -1600),
      Pose2D(-pi, (float) fieldDimensions.xPosOpponentPenaltyArea - 150, 1600)
    }
  };
  for(int i = minRobot; i < minRobot + numOfFieldPlayers; ++i)
  {
    Robot& r = robots[i];
    r.manuallyPlaced = r.oracle && 
                       (inOwnPenaltyArea(i) ||
                        r.lastPose.translation.y < fieldDimensions.yPosRightSideline ||
                        r.lastPose.translation.y > fieldDimensions.yPosLeftSideline ||
                        (i < numOfRobots / 2 && (r.lastPose.translation.x < fieldDimensions.xPosOpponentPenaltyMark ||
                                                 r.lastPose.translation.x > fieldDimensions.xPosOpponentGroundline)) ||
                        (i >= numOfRobots / 2 && (r.lastPose.translation.x > fieldDimensions.xPosOwnPenaltyMark ||
                                                  r.lastPose.translation.x < fieldDimensions.xPosOwnGroundline)));
    if(r.manuallyPlaced)
      placeFromSet(i, minRobot, poses[i < numOfRobots / 2 ? 1 : 0], numOfFieldPlayers);
  }
}

void GameController::executePlacement()
{
  for(int i = 0; i < numOfRobots; ++i)
  {
    const Robot& r = robots[i];
    if(r.manuallyPlaced)
      r.oracle->moveRobot(Vector3<>(r.lastPose.translation.x, r.lastPose.translation.y, 350),
                          Vector3<>(0, 0, r.lastPose.rotation), true);
  }
}

void GameController::referee()
{
  if(automatic)
  {
    SYNC;
    switch(gameInfo.state)
    {
    case STATE_READY:
      if(SystemCall::getTimeSince(timeWhenStateBegan) < 2000)
        timeWhenLastRobotMoved = 0;
      if(SystemCall::getTimeSince(timeWhenStateBegan) >= 45000 ||
         (timeWhenLastRobotMoved && SystemCall::getTimeSince(timeWhenLastRobotMoved) > 2000))
        handleGlobalCommand("set");
      break;
    
    case STATE_SET:
      if(SystemCall::getTimeSince(timeWhenStateBegan) >= 5000)
        handleGlobalCommand("playing");
      break;
      
    case STATE_PLAYING:
      switch(Oracle::updateBall())
      {
      case Oracle::GOAL_BY_BLUE:
        ++teamInfos[TEAM_BLUE].score;
        VERIFY(handleGlobalCommand("kickOffRed"));
        VERIFY(handleGlobalCommand("ready"));
        break;
      case Oracle::GOAL_BY_RED:
        ++teamInfos[TEAM_RED].score;
        VERIFY(handleGlobalCommand("kickOffBlue"));
        VERIFY(handleGlobalCommand("ready"));
        break;
      case Oracle::OUT_BY_BLUE:
        VERIFY(handleGlobalCommand("outByBlue"));
        break;
      case Oracle::OUT_BY_RED:
        VERIFY(handleGlobalCommand("outByRed"));
        break;
      case Oracle::NONE:
        break;
      }
    }
  }
}

void GameController::writeGameInfo(Out& stream)
{
  SYNC;
  if(timeOfLastDropIn)
    gameInfo.dropInTime = SystemCall::getTimeSince(timeOfLastDropIn) / 1000;
  else
    gameInfo.dropInTime = -1;
  if(gameInfo.state == STATE_PLAYING)
    gameInfo.secsRemaining = 600 - SystemCall::getTimeSince(timeWhenHalfStarted) / 1000;
  stream << gameInfo;
}

void GameController::writeOwnTeamInfo(int robot, Out& stream)
{
  SYNC;
  stream << teamInfos[robot * 2 / numOfRobots];
}

void GameController::writeOpponentTeamInfo(int robot, Out& stream)
{
  SYNC;
  stream << teamInfos[1 - robot * 2 / numOfRobots];
}

void GameController::writeRobotInfo(int robot, Out& stream)
{
  SYNC;
  Robot& r = robots[robot];
  if(r.info.penalty)
    r.info.secsTillUnpenalised = std::max(int(30 - SystemCall::getTimeSince(r.timeWhenPenalized) / 1000), 0);
  RobotPose pose;
  ASSERT(r.oracle);
  r.oracle->getRobotPose(pose);
  if(robot < numOfRobots / 2)
    (Pose2D&) pose = Pose2D(pi) + pose;
  if((pose.translation - r.lastPose.translation).abs() > 5 ||
     normalize(pose.rotation - r.lastPose.rotation) > 0.05)
  {
    timeWhenLastRobotMoved = SystemCall::getCurrentSystemTime();
    r.lastPose = pose;
  }
  stream << r.info;
}

void GameController::addCompletion(std::set<std::string>& completion) const
{
  static const char* commands[] =
  {
    "initial",
    "ready",
    "set",
    "playing",
    "finished",
    "kickOffBlue",
    "kickOffRed",
    "outByBlue",
    "outByRed"
  };
  const int num = sizeof(commands) / sizeof(commands[0]);
  for(int i = 0; i < num; ++i)
    completion.insert(std::string("gc ") + commands[i]);
  for(int i = 0; i < numOfPenalties; ++i)
    if(i != obstruction)
      completion.insert(std::string("pr ") + getName((Penalty) i));
}
