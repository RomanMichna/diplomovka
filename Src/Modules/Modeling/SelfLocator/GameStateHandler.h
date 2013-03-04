/**
* @file GameStateHandler.h
*
* This file declares a submodule correcting localization based on GameController info.
*
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#pragma once

#include "SelfLocatorParameters.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Modeling/SideConfidence.h"
#include "Representations/Modeling/OwnSideModel.h"
#include "Representations/Sensing/GroundContactState.h"

/**
* @class GameStateHandler
*
* A submodule correcting localization based on GameController info.
*/
class GameStateHandler
{
private:
  const SelfLocatorParameters& parameters;
  const FieldDimensions& theFieldDimensions;
  const GameInfo& theGameInfo;
  const OwnTeamInfo& theOwnTeamInfo;
  const RobotInfo& theRobotInfo;
  const RobotPose& theRobotPose;
  const OdometryData& theOdometryData;
  const SideConfidence& theSideConfidence;
  const OwnSideModel& theOwnSideModel;
  const GroundContactState& theGroundContactState;
  
  int lastPenalty; /**< Was the robot penalised in the last frame? */
  int lastGameState; /**< The game state in the last frame. */
  Pose2D lastRobotPose; /**< The validated pose of the robot calculated in the previous frame (or propageted by odometry). */
  Pose2D lastOdometryData; /**< The odometry data of the previous frame. */
  bool lastGroundContactState; /**< Had the robot ground contact in the previous frame? */
  bool wasValidatedBefore; /**< Was the pose validated after the robot was manually placed? */
  
  /**
  * Checks whether the mirrored version of the current pose would be closer
  * to the last pose than the original current pose.
  * @param currentPose The current pose.
  * @param lastPose The previous pose that is tested against.
  * @return Is mirroring better?
  */
  bool isMirrorCloser(const Pose2D& currentPose, const Pose2D& lastPose) const;

public:
  GameStateHandler(const SelfLocatorParameters& parameters,
                   const FieldDimensions& fieldDimensions,
                   const GameInfo& gameInfo,
                   const OwnTeamInfo& ownTeamInfo,
                   const RobotInfo& robotInfo,
                   const RobotPose& robotPose,
                   const OdometryData& odometryData,
                   const SideConfidence& sideConfidence,
                   const OwnSideModel& ownSideModel,
                   const GroundContactState& groundContactState);

  /**
  * This method must be called before the other methods to initialize processing the 
  * current frame.
  */
  void preProcess();
  
  /**
  * Should the samples be redistributed and if yes, how?
  * @param poses The centers of the new sample distribution.
  * @param standardDeviations The standard distributions around the new centers.
  * @return Should the samples be redistributed?
  */
  bool shouldRedistribute(std::vector<Pose2D>& poses, std::vector<Pose2D>& standardDeviations) const;
  
  /**
  * Should the pose be mirrored around the center of the field?
  * @param pose The pose to be checked.
  * @return Should it?
  */
  bool shouldMirror(const Pose2D& pose) const;
  
  /**
  * This method must be called after the other methods to finish processing the 
  * current frame.
  * @param Was the pose mirrored or were the samples redistributed?
  */
  void postProcess(bool wasMirroredOrRedistributed);
};
