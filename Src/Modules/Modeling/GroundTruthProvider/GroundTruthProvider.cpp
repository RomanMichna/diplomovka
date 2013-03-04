#include "GroundTruthProvider.h"
#include "Tools/Debugging/Modify.h"

void GroundTruthProvider::init()
{
  lastUpdateFrameTime = 0;
  InConfigMap stream("sslVision.cfg");
  if(stream.exists())
    stream >> params;
  else
    params.robotId = 7;
}

void GroundTruthProvider::setCurrentGroundTruth()
{
  MODIFY("parameters:GroundTruthProvider", params);
  if(theSSLVisionData.recentData.size() == 0) return;
  const SSLVisionData::SSLVisionFrame& currentFrame = theSSLVisionData.recentData.top();

  groundTruthTimestamp = currentFrame.receiveTimestamp;

  bool robotPoseSet = false;
  for(size_t i = 0; !robotPoseSet && i < currentFrame.blueRobots.size(); i++)
  {
    if(currentFrame.blueRobotIds[i] == params.robotId)
    {
      (Pose2D&) currentRobotPose = currentFrame.blueRobots[i];
      robotPoseSet = true;
    }
  }
  for(size_t i = 0; !robotPoseSet && i < currentFrame.yellowRobots.size(); i++)
  {
    if(currentFrame.yellowRobotIds[i] == params.robotId)
    {
      (Pose2D&) currentRobotPose = currentFrame.yellowRobots[i];
      robotPoseSet = true;
    }
  }
  // Actually, the validity and deviation depend on the accuracy of the SSL
  // vision and on the delay. But how could this be ground truth, if we admitted
  // that here?
  currentRobotPose.validity = 1.0f;
  currentRobotPose.deviation = 0.0f;

  Vector2<> absBallPosition = currentFrame.ball;
  DEBUG_RESPONSE("module:GroundTruthProvider:rotateField", { absBallPosition.rotate(pi); });
  const Vector2<> relBallPosition = (absBallPosition - currentRobotPose.translation).rotate(-currentRobotPose.rotation);
  currentBallModel.timeWhenLastSeen = groundTruthTimestamp;
  currentBallModel.estimate.position = relBallPosition;
  currentBallModel.lastPerception = relBallPosition;
  // TODO ball model velocity

  currentRobotsModel.robots.clear();
  currentRobotsModel.robots.reserve(currentFrame.yellowRobots.size() + currentFrame.blueRobots.size());
  for(size_t i = 0; i < currentFrame.yellowRobots.size(); i++)
  {
    if(currentFrame.yellowRobotIds[i] != params.robotId)
    {
      Pose2D absRobotPose = currentFrame.yellowRobots[i];
      DEBUG_RESPONSE("module:GroundTruthProvider:rotateField", { absRobotPose.translation.rotate(pi); });
      const Vector2<> relRobotPosition = (absRobotPose.translation - currentRobotPose.translation).rotate(-currentRobotPose.rotation);
      currentRobotsModel.robots.push_back(RobotsModel::Robot(relRobotPosition, true, true, Matrix2x2<>(1.0, 0.0, 0.0, 1.0), theFrameInfo.time));
    }
  }
  for(size_t i = 0; i < currentFrame.blueRobots.size(); i++)
  {
    if(currentFrame.blueRobotIds[i] != params.robotId)
    {
      Pose2D absRobotPose = currentFrame.blueRobots[i];
      DEBUG_RESPONSE("module:GroundTruthProvider:rotateField", { absRobotPose.translation.rotate(pi); });
      const Vector2<> relRobotPosition = (absRobotPose.translation - currentRobotPose.translation).rotate(-currentRobotPose.rotation);
      currentRobotsModel.robots.push_back(RobotsModel::Robot(relRobotPosition, true, true, Matrix2x2<>(1.0, 0.0, 0.0, 1.0), theFrameInfo.time));
    }
  }

  lastUpdateFrameTime = theFrameInfo.time;
}

void GroundTruthProvider::update(BallModel& ballModel)
{
  if(theFrameInfo.time > lastUpdateFrameTime)
    setCurrentGroundTruth();
  ballModel = currentBallModel;
}

void GroundTruthProvider::update(GroundTruthBallModel& groundTruthBallModel)
{
  if(theFrameInfo.time > lastUpdateFrameTime)
    setCurrentGroundTruth();
  groundTruthBallModel = (GroundTruthBallModel&) currentBallModel;
}

void GroundTruthProvider::update(RobotPose& robotPose)
{
  if(theFrameInfo.time > lastUpdateFrameTime)
    setCurrentGroundTruth();
  robotPose = currentRobotPose;
}

void GroundTruthProvider::update(GroundTruthRobotPose& groundTruthRobotPose)
{
  if(theFrameInfo.time > lastUpdateFrameTime)
    setCurrentGroundTruth();
  groundTruthRobotPose = (GroundTruthRobotPose&) currentRobotPose;
  groundTruthRobotPose.timestamp = groundTruthTimestamp;
}

void GroundTruthProvider::update(RobotsModel& robotsModel)
{
  if(theFrameInfo.time > lastUpdateFrameTime)
    setCurrentGroundTruth();
  robotsModel = currentRobotsModel;
}

void GroundTruthProvider::update(GroundTruthRobotsModel& groundTruthRobotsModel)
{
  if(theFrameInfo.time > lastUpdateFrameTime)
    setCurrentGroundTruth();
  groundTruthRobotsModel = (GroundTruthRobotsModel&) currentRobotsModel;
}

MAKE_MODULE(GroundTruthProvider, Infrastructure);
