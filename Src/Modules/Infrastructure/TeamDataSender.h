/**
* @file TeamDataSender.h
* Declaration of module TeamDataSender
* @author Colin Graf
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/TeamMateData.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Infrastructure/RobotHealth.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Modeling/CombinedWorldModel.h"
#include "Representations/Modeling/FreePartOfOpponentGoalModel.h"
#include "Representations/Modeling/SideConfidence.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/LinePercept.h"


MODULE(TeamDataSender)
  REQUIRES(FrameInfo)
  REQUIRES(TeamMateData)

  REQUIRES(RobotPose)
  REQUIRES(SideConfidence)
  REQUIRES(BallModel)
  REQUIRES(RobotsModel)
  REQUIRES(ObstacleModel)
  REQUIRES(MotionRequest)
  REQUIRES(FilteredSensorData)
  REQUIRES(JointRequest)
  REQUIRES(RobotHealth)
  REQUIRES(RobotInfo)
  REQUIRES(GroundContactState)
  REQUIRES(FallDownState)
  REQUIRES(CombinedWorldModel)
  REQUIRES(FreePartOfOpponentGoalModel)
  REQUIRES(CameraMatrix)
  REQUIRES(LinePercept)

  PROVIDES(TeamDataSenderOutput)
END_MODULE

/**
* @class TeamDataSender
* A modules for sending some representation to teammates
*/
class TeamDataSender : public TeamDataSenderBase
{
public:

  /** Default constructor */
  TeamDataSender() : sendFrames(0) {}

private:
  unsigned int sendFrames; /** Quantity of frames in which team data was sent */

  /**
  * The update function called in each cognition process cycle
  * @param teamDataSenderOutput An empty output representation
  */
  virtual void update(TeamDataSenderOutput& teamDataSenderOutput);
};
