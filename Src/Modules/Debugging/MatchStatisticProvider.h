/**
* @file MatchStatisticProvider.h
* Declares a class that collects match information and statistics
* @author Tobias Kastner
*/

#pragma once

#include "Tools/Module/Module.h"

#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Debugging/MatchStatistic.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/BehaviorControl/BehaviorControlOutput.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Perception/BallPercept.h"
#include "Representations/Modeling/FieldCoverage.h"
#include "Representations/Infrastructure/TeamMateData.h"
#include "Representations/Infrastructure/KeyStates.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Modeling/SideConfidence.h"

MODULE(MatchStatisticProvider)
  REQUIRES(RobotPose)
  REQUIRES(BallModel)
  REQUIRES(BallPercept)
  REQUIRES(FrameInfo)
  REQUIRES(OwnTeamInfo)
  REQUIRES(GameInfo)
  REQUIRES(MotionInfo)
  REQUIRES(FallDownState)
  REQUIRES(BehaviorControlOutput)
  REQUIRES(GroundContactState)
  REQUIRES(RobotInfo)
  REQUIRES(FieldDimensions)
  REQUIRES(FieldCoverage)
  REQUIRES(TeamMateData)
  REQUIRES(KeyStates)
  REQUIRES(OdometryData)
  REQUIRES(SideConfidence)
  PROVIDES_WITH_MODIFY_AND_DRAW(MatchStatistic)
END_MODULE

class MatchStatisticProvider : public MatchStatisticProviderBase
{
public:
  /** Constructor */
  MatchStatisticProvider() : lastState(0), state(0), lastTimeWhenUpdated(0),
    timeWhenKickWasPerformed(0), timePassed(0), gridUpdateCycle(0), ballNotSeenTime(0) ,
    timePlayingNoGoalRobot(0), timePlayingNoGoalTeam(0), viewMode(false), lastBallWasSeen(false),
    mirror(false), duelKick(false), kickMightScore(false), testKickoff(false), lastChestButton(false),
    timesSaved(0)
  {};

private:

  class MatchStatisticProviderParameters : public Streamable
  {
    void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN;
      STREAM(timeSinceLastPackageReceiveWaitTime);
      STREAM(minPositionChangeThreshold);
      STREAM(maxTimeSinceKick);
      STREAM(maxTimeKickCauseFall);
      STREAM(minCycleTime);
      STREAM(poseDeviationThreshold);
      STREAM(tolerableSearchTime);
      STREAM(saveStatistics);
      STREAM_REGISTER_FINISH;
    }

  public:
    MatchStatisticProviderParameters() : timeSinceLastPackageReceiveWaitTime(2000),
      maxTimeSinceKick(9000), maxTimeKickCauseFall(5000), minCycleTime(3000),
      tolerableSearchTime(3000), minPositionChangeThreshold(300.f), poseDeviationThreshold(100.f),
      saveStatistics(false) {};

    unsigned timeSinceLastPackageReceiveWaitTime;
    int maxTimeSinceKick;
    int maxTimeKickCauseFall;
    int minCycleTime;
    int tolerableSearchTime;
    float minPositionChangeThreshold;
    float poseDeviationThreshold;
    bool saveStatistics;
  };

  void init();
  void update(MatchStatistic& theMatchStatistics); /**< update the representation to provide */

  void updateCountableStuff(MatchStatistic& ms); /**< update information that can be count ... like time elapsing concerns */
  void updateScoreInformation(MatchStatistic& ms); /**< update information wherefrom, when ...  a kick was performed */
  void updateFallDownInformation(MatchStatistic& ms); /**< update fall down information */
  void updateVisitedGrid(MatchStatistic& ms); /**< update grid and trace information */
  void updateBallStuff(MatchStatistic& ms); /**< update ball related stuff */
  void updateSearchingInformation(MatchStatistic& ms); /**< update information on searching a ball */
  void updatePenaltyInformation(MatchStatistic& ms); /**< update information on penalties */
  void updateBehaviorInformation(MatchStatistic& ms); /**< update behavior specific stuff */

  void save(MatchStatistic& theMatchStatistics); /**< save the provided representation ... for later analysis */
  void load(MatchStatistic& theMatchStatistics, std::string robot); /**< load the provided representation ... for analysis */

  MatchStatisticProviderParameters p;
  int lastState; /**< last gameControler/match state */
  int state; /**< current gameController/match state */
  unsigned lastTimeWhenUpdated; /**< timestamp when the last update was cycled */
  unsigned timeWhenKickWasPerformed; /**< timestamp when the last kick was performed */
  unsigned timePassed; /**< time elapsed from last update until now */
  unsigned gridUpdateCycle; /**< update grid every [?] seconds */
  unsigned ballNotSeenTime; /**< current time elapsed not seeing the ball */
  unsigned searchTime; /**< current time spent searching */
  unsigned timePlayingNoGoalRobot; /**< time elapsed in playing state when no goal was scored by this robot*/
  unsigned timePlayingNoGoalTeam; /**< time elapsed in playing state when no goal was scored by this robots team */

  bool viewMode; /**< needed for analysis or "are we in view mode?" */
  bool lastBallWasSeen; /**< was the ball seen in the past */
  bool mirror; /**< was the last kick mirrored */
  bool duelKick; /**< was the last kick caused by duelling */
  bool kickMightScore; /**< indicates wether this robot still believes that its last kick might hit the goal */
  bool testKickoff; /**< perform a test kickoff if true, so noting will be logged */
  Pose2D poseWhenKicked; /**< as the name says */
  Pose2D poseWhenSearching; /**< as the name says */
  bool lastChestButton; /** was the chestButton pressed? */
  unsigned timesSaved; /** how often the save function was called*/

  GameInfo lastGameInfo;
  MotionInfo lastMotionInfo;
  MotionRequest::Motion motion;
  BikeRequest::BMotionID bikeID;
  WalkRequest::KickType walkKick;
  OwnTeamInfo lastOwnTeamInfo;
  GroundContactState lastGroundContactState;
  RobotInfo lastRobotInfo;
  BehaviorData lastBehaviorData;
  OdometryData lastOdometryData;
  SideConfidence lastSideConfidence;
};
