/**
* @file TeamDataSender.cpp
* Implementation of module TeamDataSender
* @author Colin Graf
*/

#include "TeamDataSender.h"
#include "Tools/Team.h"
#include "Tools/Debugging/ReleaseOptions.h"

MAKE_MODULE(TeamDataSender, Infrastructure)

void TeamDataSender::update(TeamDataSenderOutput& teamDataSenderOutput)
{
  if(theTeamMateData.sendThisFrame)
  {
    ++sendFrames;

    TEAM_OUTPUT(idTeamMateRobotPose, bin, RobotPoseCompressed(theRobotPose));
    TEAM_OUTPUT(idTeamMateSideConfidence, bin, theSideConfidence);
    TEAM_OUTPUT(idTeamMateBallModel, bin, BallModelCompressed(theBallModel));
    TEAM_OUTPUT(idTeamMateRobotsModel, bin, RobotsModelCompressed(theRobotsModel));
    TEAM_OUTPUT(idTeamMateObstacleModel, bin,ObstacleModelCompressed(theObstacleModel));

    TEAM_OUTPUT(idTeamMateIsPenalized, bin, (theRobotInfo.penalty != PENALTY_NONE));
    TEAM_OUTPUT(idTeamMateHasGroundContact, bin, theGroundContactState.contact);
    TEAM_OUTPUT(idTeamMateIsUpright, bin, (theFallDownState.state == theFallDownState.upright));
    if(theGroundContactState.contact)
      TEAM_OUTPUT(idTeamMateTimeSinceLastGroundContact, bin, theFrameInfo.time);
    TEAM_OUTPUT(idTeamCameraHeight, bin, theCameraMatrix.translation.z);

    if(Global::getReleaseOptions().motionRequest)
      TEAM_OUTPUT(idMotionRequest, bin, theMotionRequest);

    if(Global::getReleaseOptions().sensorData)
    {
      TEAM_OUTPUT(idSensorData, bin, theFilteredSensorData);
      TEAM_OUTPUT(idJointData, bin, theJointRequest);
    }

    if(Global::getReleaseOptions().robotHealth || sendFrames % 20 == 0)
      TEAM_OUTPUT(idRobotHealth, bin, theRobotHealth);

    if(Global::getReleaseOptions().combinedWorldModel)
      TEAM_OUTPUT(idTeamMateCombinedWorldModel, bin, theCombinedWorldModel);

    if(Global::getReleaseOptions().freePartOfOpponentGoal)
      TEAM_OUTPUT(idTeamMateFreePartOfOpponentGoalModel, bin, theFreePartOfOpponentGoalModel);

    if(Global::getReleaseOptions().linePercept)
      TEAM_OUTPUT(idLinePercept, bin, theLinePercept);
  }
}
