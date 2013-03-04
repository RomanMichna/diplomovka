/**
* @file TeamMateData.h
* Declaration of a class representing information about the teammates.
* @author Colin Graf
*/

#pragma once

#include "Representations/Modeling/RobotsModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/BallAfterKickPose.h"
#include "Representations/Modeling/FieldCoverage.h"
#include "Representations/Modeling/SideConfidence.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include <sstream>

/**
* @class TeamMateData
* A class representing information about the teammates.
*/
class TeamMateData : public Streamable
{
private:
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(numOfConnectedPlayers);
    STREAM(sendThisFrame);
    STREAM(wasConnected);
    STREAM(timeStamps);
    STREAM(isConnected);
    STREAM(ballAfterKickPose);
    STREAM(passTarget);
    STREAM(ballModels);
    STREAM(obstacleModels);
    STREAM(robotsModels);
    STREAM(robotPoses);
    STREAM(robotsSideConfidence)
    STREAM(behaviorData);
    STREAM(isPenalized);
    STREAM(hasGroundContact);
    STREAM(isUpright);
    STREAM(teamHeadControlStates);
    STREAM(fieldCoverages);
    STREAM(timeLastGroundContact);
    STREAM(cameraHeights);
    STREAM_REGISTER_FINISH;
  }

public:
  ENUM(Player,
    noPlayer,
    firstPlayer,
    player1 = firstPlayer,
    player2,
    player3,
    player4
  );
  unsigned numOfConnectedPlayers; /**< The number of robots of which messages were received recently. _Not_ including this robot itself. */
  unsigned firstTeamMate;  /**< player number of first team mate */
  unsigned secondTeamMate; /**< player number of second team mate */
  unsigned thirdTeamMate;  /**< player number of third team mate */
  bool sendThisFrame; /**< The team communication will be sent in this frame. */
  bool wasConnected; /**< Whether we have been connected to a team mate. */
  unsigned int timeStamps[numOfPlayers]; /**< The times when messages from different robots arrived. */
  bool isConnected[numOfPlayers]; /**< Whether messages from each team mate were recently received. */

  BallAfterKickPose ballAfterKickPose;
  PassTarget passTarget;
  BallModel ballModels[numOfPlayers]; /**< The last received ball model of each team mate. */
  ObstacleModel obstacleModels[numOfPlayers];/**< The last received obstacle model of each team mate. */
  RobotsModel robotsModels[numOfPlayers]; /**< The last received robots model of each team mate. */
  RobotPose robotPoses[numOfPlayers]; /**< The last received robot pose of each team mate. */
  SideConfidence robotsSideConfidence[numOfPlayers]; /**< The last received sideConfidence of each team mate. */
  BehaviorData behaviorData[numOfPlayers]; /**< The last received behavior data of each team mate. */
  TeamHeadControlState teamHeadControlStates[numOfPlayers];
  FieldCoverage::GridInterval fieldCoverages[numOfPlayers]; /**< The last received field coverage grid of each team mate. */
  bool isPenalized[numOfPlayers]; /**< Tells us if a teammate is penalized. */
  bool hasGroundContact[numOfPlayers]; /**< Tells us if a teammate has ground contact. */
  bool isUpright[numOfPlayers]; /**< Tells us if a teammate is fallen down. */
  unsigned int timeLastGroundContact[numOfPlayers]; /**< The time since last ground contact of a team mate. */
  float cameraHeights[numOfPlayers]; /**<camera heights of team mates> */
  
  /**
  * Default constructor.
  */
  TeamMateData() :
    numOfConnectedPlayers(0),
    sendThisFrame(false),
    wasConnected(false)
  {
    for(int i = 0; i < numOfPlayers; ++i)
    {
      timeStamps[i] = 0;
      timeLastGroundContact[i] = 0;
    }
  }

  /** drawing function for representation*/
  void draw()
  {
    DECLARE_DEBUG_DRAWING("representation:TeamMateData", "drawingOnField");
    COMPLEX_DRAWING("representation:TeamMateData",
    {
      for(int i = 1; i < numOfPlayers; ++i)
      {
        if(timeStamps[i])
        {
          Vector2<>& rPos = robotPoses[i].translation;
          Vector2<> dirPos = robotPoses[i] * Vector2<>(200, 0);
          std::stringstream numStream;
          numStream << i;
          CIRCLE("representation:TeamMateData", rPos.x, rPos.y, 200, 20, Drawings::ps_solid,
                 ColorRGBA(255, 0, 0), Drawings::bs_null, ColorClasses::white);
          LINE("representation:TeamMateData", rPos.x, rPos.y, dirPos.x, dirPos.y, 20,
               Drawings::ps_solid, ColorRGBA(255, 0, 0));
          DRAWTEXT("representation:TeamMateData", rPos.x, rPos.y, 150, ColorRGBA(255, 0, 0), numStream.str().c_str());
          DRAWTEXT("representation:TeamMateData", rPos.x + 200, rPos.y - 200, 150,
                   ColorRGBA(255, 0, 0), BehaviorData::getName(behaviorData[i].role));
          Vector2<> bPos = robotPoses[i] * ballModels[i].estimate.position;
          CIRCLE("representation:TeamMateData", bPos.x, bPos.y, 50, 20, Drawings::ps_solid,
                 ColorRGBA(255, 0, 0), Drawings::bs_solid, ColorRGBA(255, 0, 0));
          LINE("representation:TeamMateData", rPos.x, rPos.y, bPos.x, bPos.y, 20, Drawings::ps_dash, ColorRGBA(255, 0, 0));
        }
      }
      Vector2<>bAPos = ballAfterKickPose.position;
      Vector2<>bAPosDev = ballAfterKickPose.dev;
      CIRCLE("representation:TeamMateData", bAPos.x, bAPos.y, bAPosDev.x, 3, Drawings::ps_solid,
             ColorRGBA(50, 200, 200, 100), Drawings::bs_solid, ColorRGBA(50, 200, 200, 100));
    });
  }
};

/**
* @class TeamDataSenderOutput
* An empty dummy representation for the TeamDataSender module
*/
class TeamDataSenderOutput : public Streamable
{
 /**
  * The method makes the object streamable
  * @param in The stream from which the object is read
  * @param out The stream to which the object is written
  */
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM_REGISTER_FINISH;
  }
};
