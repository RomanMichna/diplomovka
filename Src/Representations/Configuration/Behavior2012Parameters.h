/**
* @file Representations/Configuration/Behavior2012Parameters.h
*
* The file declares a class that containts frequently used parameters of the behavior which can be modified.
*
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
*/

#pragma once

#include "Tools/Configuration/ConfigMap.h"
#include "Tools/Math/Pose2D.h"
#include "Tools/Enum.h"



class BH2012KickoffDescription : public Streamable
{
private:
  /**
  * The method makes the object streamable.
  * @param in The stream from which the object is read (if in != 0).
  * @param out The stream to which the object is written (if out != 0).
  */
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(id);
    STREAM(name);
    STREAM(weighting);
    STREAM(isDefault);
    STREAM(action);
    STREAM(actionParameters);
    STREAM(goaliePose);
    STREAM(playerPoses);
    STREAM_REGISTER_FINISH;
  }

public:
  int id;
  string name;
  float weighting;
  bool isDefault;
  string action;
  vector<float> actionParameters;
  Pose2D goaliePose;
  vector<Pose2D> playerPoses;

  /** Default constructor. */
  BH2012KickoffDescription() : id(-1), name(""), weighting(0.0f), isDefault(true)
  {
    actionParameters.push_back(0);
    actionParameters.push_back(0);
  }
};

class Behavior2012Parameters : public Streamable
{
private:
  /**
  * The method makes the object streamable.
  * @param in The stream from which the object is read (if in != 0).
  * @param out The stream to which the object is written (if out != 0).
  */
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(offensiveKickoffs);
    STREAM(defensiveKickoffs);
    STREAM(kickoffStrategyTimeout);
    STREAM(kickoffExecutionTimeout);
    STREAM(roleValuationFunction);
    STREAM(roleStrikerBonus);
    STREAM(roleSyncTicks);
    STREAM(roleFactorBallLastSeen);
    STREAM(roleFactorBallDisappeared);
    STREAM(roleTactic);
    STREAM(kickPoseKickInaccuracy);
    STREAM(kickPoseAssumedWalkingSpeed);
    STREAM(kickPoseSidewardsKickRange);
    STREAM(kickPoseSidewardsWalkKickRange);
    STREAM(kickPoseBackwardsKickRange);
    STREAM(kickPoseKeeperOnlyForwardKick);
    STREAM(supporterTranslationX);
    STREAM(supporterTranslationY);
    STREAM(supporterTranslationDefensiveX);
    STREAM(supporterTranslationDefensiveY);
    STREAM(supporterTranslationKeeperX);
    STREAM(supporterTranslationKeeperY);
    STREAM(supporterMinimumDistanceBetweenSupportedPositionAndSideLine);
    STREAM(supporterMaximumExtraDistance);
    STREAM(supporterOrientationRefersToBall);
    STREAM(supporterMinimumDistanceBetweenSupportedPositionAndOpponentsGroundLine);
    STREAM(supporterMinimumXPositionIfDefenderExists);
    STREAM(supporterWaitForStriker);
    STREAM(useNativeRole);
    STREAM(keeperDivingOn);
    STREAM(keeperAngleTolerance);
    STREAM(keeperPositionToleranceY);
    STREAM(maxGPChangeThreshold);
    STREAM(minGPChangeThreshold);
    STREAM(keeperAngleSmall);
    STREAM(keeperAngleBig);
    STREAM(keeperAngleSmallFactor);
    STREAM(keeperAngleBigFactor);
    STREAM(enableKeeperSitLocalisation);
    STREAM(enableMiddle);
    STREAM(penaltyKickXTarget);
    STREAM(kickProbabilityOwnHalfBike);
    STREAM(kickProbabilityOwnHalfWalk);
    STREAM(kickProbabilityOpponentHalfBike);
    STREAM(kickProbabilityOpponentHalfWalk);
    STREAM(leftCornerY);
    STREAM(rightCornerY);
    STREAM(halfLeftY);
    STREAM(halfRightY);
    STREAM(enableName);
    STREAM(exhaustedKickThreshold);
    STREAM(obstacleAvoidanceMethod);
    STREAM(enableConfusedBehavior);
    STREAM(enableConfusedAndAloneBehavior);
    STREAM(playingWithBrokenKeeper);
    STREAM(bteamEnabled);
    STREAM(bteamSpeedPercentage);
    STREAM(enableAttackingKeeper);
    STREAM_REGISTER_FINISH;
  }

public:
  /** A list of offensive kickoff definitions */
  vector<BH2012KickoffDescription> offensiveKickoffs;
  /** A list of defensive kickoff definitions */
  vector<BH2012KickoffDescription> defensiveKickoffs;
  /** Maximum time (in ms) for waiting for the team strategy to be determined */
  int kickoffStrategyTimeout;
  /** Maximum time (in ms) for kickoff behavior. Afterwards, behavior switches to "playing"*/
  float kickoffExecutionTimeout;

  /** Valuation functions for dynamic role changes */
  ENUM(RoleValuationFunction,
    kickPose,
    xabsl
  );
  RoleValuationFunction roleValuationFunction;
  
  /** Bonus for the striker [ms] */
  float roleStrikerBonus;
  /** Desired synchronization ticks */
  int roleSyncTicks;
  /** Factor for time since ball last seen */
  float roleFactorBallLastSeen;
  /** Factor for time since ball disappeared */
  float roleFactorBallDisappeared;
  /** Selected tactic (offenive => 2 supporter, normal => 1 defender, 1 supporter, defensive => 2 defender) */
  ENUM(RoleTactic,
    normal,
    defensive,
    offensive
  );
  RoleTactic roleTactic;

  /** The inaccuracy of the kick direction angle according to the intended direction */
  float kickPoseKickInaccuracy;
  /** The walking speed assumed for estimating the time to reach the ball for shooting a goal */
  Pose2D kickPoseAssumedWalkingSpeed;
  /** The range of sidewards kicks */
  float kickPoseSidewardsKickRange;
  /** The range of backwards kicks */
  float kickPoseBackwardsKickRange;
  /** The range of sidewards walking engine kicks */
  float kickPoseSidewardsWalkKickRange;
  /** Whether the keeper only be able to use forward kicks */
  bool kickPoseKeeperOnlyForwardKick;

  /** How far behind the supported object our (offensive) supporters are placed? */
  float supporterTranslationX;
  /** How far beside the supported object our (offensive) supporters are placed? */
  float supporterTranslationY;
  /** How far behind the supported object our (defensive) supporters are placed? */
  float supporterTranslationDefensiveX;
  /** How far beside the supported object our (defensive) supporters are placed? */
  float supporterTranslationDefensiveY;
  /** How far before the keeper our supporters are placed? */
  float supporterTranslationKeeperX;
  /** How far beside the keeper our supporters are placed? */
  float supporterTranslationKeeperY;
  /** From which position beside the side line the supporters are placed on the same side */
  float supporterMinimumDistanceBetweenSupportedPositionAndSideLine;
  /** How far the (offensive) supporter should be located more to the middle for side located positions */
  float supporterMaximumExtraDistance;
  /** Should the ball be the point of interest for orientation? If not it is the striker. */
  bool supporterOrientationRefersToBall;
  /** From which position before the opponents ground line the supporters are placed on the same side */
  float supporterMinimumDistanceBetweenSupportedPositionAndOpponentsGroundLine;
  /** If there is a defender, the supporter should not have a smaller x position. */
  float supporterMinimumXPositionIfDefenderExists;
  /** Should the supporter wait for the striker if he is near to the side and it would be difficult to walk around*/
  bool supporterWaitForStriker;

  /** use native role given by settings */
  bool useNativeRole;

  /** whether the keeper uses diving motion or just signs jump direction */
  bool keeperDivingOn;
  /** angle threshold to determine if the goalie has reached its target pose */
  float keeperAngleTolerance;
  /** y-coord threshold to determine if the goalie has reached its target pose */
  float keeperPositionToleranceY;
  /** max goalie position change threshold */
  float maxGPChangeThreshold;
  /** min goalie position change threshold */
  float minGPChangeThreshold;
  /** fixed angle for goalie-pose small (>0) */
  float keeperAngleSmall;
  /** fixed angle for goalie-pose big (>keeperAngleSmall) */
  float keeperAngleBig;
  /** threshold/factor when to use keeperAngleSmall */
  float keeperAngleSmallFactor;
  /** threshold/factor when to use keeperAngleBig */
  float keeperAngleBigFactor;
  /** wether the keeper uses the ParticleFilterSelfLocator while sitting */
  bool enableKeeperSitLocalisation;

  /** whether the penalty striker is allowed to shoot in the middle of the goal */
  bool enableMiddle;
  /** x component of the kick target for penalty kicks */
  float penaltyKickXTarget;

  /** probabilities for different forward kicks */
  float kickProbabilityOwnHalfBike, kickProbabilityOwnHalfWalk,
        kickProbabilityOpponentHalfBike, kickProbabilityOpponentHalfWalk;

  /** y components of the kick targets */
  float leftCornerY;
  float rightCornerY;
  float halfLeftY;
  float halfRightY;

  bool enableName;

  float exhaustedKickThreshold;

  ENUM(ObstacleAvoidanceMethod,
    pathFinderAndWalkTo,
    walkTo,
    improvedWalk);
  ObstacleAvoidanceMethod obstacleAvoidanceMethod; /**< Obstacle avoidance method used */

  bool enableConfusedBehavior;
  bool enableConfusedAndAloneBehavior;

  bool playingWithBrokenKeeper;

  bool bteamEnabled;
  float bteamSpeedPercentage;

  bool enableAttackingKeeper;

  /** Default constructor. */
  Behavior2012Parameters():
    kickoffStrategyTimeout(0),
    kickoffExecutionTimeout(0.f),
    roleValuationFunction(kickPose),
    roleStrikerBonus(0),
    roleSyncTicks(0),
    roleFactorBallLastSeen(0),
    roleFactorBallDisappeared(0),
    roleTactic(normal),
    kickPoseKickInaccuracy(0),
    kickPoseSidewardsKickRange(999999.f),
    kickPoseBackwardsKickRange(999999.f),
    kickPoseKeeperOnlyForwardKick(true),
    supporterTranslationX(0),
    supporterTranslationY(0),
    supporterTranslationDefensiveX(0),
    supporterTranslationDefensiveY(0),
    supporterTranslationKeeperX(0),
    supporterTranslationKeeperY(0),
    supporterMinimumDistanceBetweenSupportedPositionAndSideLine(0),
    supporterMaximumExtraDistance(0),
    supporterOrientationRefersToBall(false),
    supporterMinimumDistanceBetweenSupportedPositionAndOpponentsGroundLine(0),
    supporterMinimumXPositionIfDefenderExists(-750.0f),
    supporterWaitForStriker(false),
    useNativeRole(false),
    keeperDivingOn(false),
    keeperAngleTolerance(3.f),
    keeperPositionToleranceY(20.f),
    maxGPChangeThreshold(400.f),
    minGPChangeThreshold(50.f),
    keeperAngleSmall(12.f),
    keeperAngleBig(25.f),
    keeperAngleSmallFactor(0.5f),
    keeperAngleBigFactor(0.8f),
    enableKeeperSitLocalisation(true),
    enableMiddle(false),
    penaltyKickXTarget(4500.f),
    kickProbabilityOwnHalfBike(0.34f),
    kickProbabilityOwnHalfWalk(0.66f),
    kickProbabilityOpponentHalfBike(1.0f),
    kickProbabilityOpponentHalfWalk(0.0f),
    enableName(false),
    exhaustedKickThreshold(70.f),
    obstacleAvoidanceMethod(pathFinderAndWalkTo),
    enableConfusedBehavior(false),
    enableConfusedAndAloneBehavior(false),
    playingWithBrokenKeeper(false),
    bteamEnabled(false),
    bteamSpeedPercentage(1.f),
    enableAttackingKeeper(false)
  {}
};
