#include "GroundTruthEvaluator.h"
#include "Tools/Math/Common.h"
#include "Tools/Debugging/DebugDrawings.h"
#include <cmath>

void GroundTruthEvaluator::init()
{
  const size_t bufferedSeconds = 2;
  const size_t cognitionFramesPerSecond = 30;
  size_t bufferSize = bufferedSeconds * cognitionFramesPerSecond;
  robotPoses.setMaxEntries(bufferSize);
  ballModels.setMaxEntries(bufferSize);
  const size_t teamCommMessagesPerSecond = 10;
  bufferSize = bufferedSeconds * teamCommMessagesPerSecond;
  groundTruthRobotPoses.setMaxEntries(bufferSize);
  groundTruthBallModels.setMaxEntries(bufferSize);
}

void GroundTruthEvaluator::update(GroundTruthResult& groundTruthResult)
{
  DECLARE_PLOT("module:GroundTruthEvaluator:translationalError");
  DECLARE_PLOT("module:GroundTruthEvaluator:rotationalError");
  DECLARE_PLOT("module:GroundTruthEvaluator:ballPositionError");
  DECLARE_PLOT("module:GroundTruthEvaluator:evaluationDelay");

  robotPoses.add(RepresentationWithTimestamp<RobotPose>(theRobotPose,
                 theFrameInfo.time));
  if(groundTruthRobotPoses.getNumberOfEntries() == 0 ||
     groundTruthRobotPoses.top().timestamp != theGroundTruthRobotPose.timestamp)
  {
    groundTruthRobotPoses.add(RepresentationWithTimestamp<RobotPose>(
                                theGroundTruthRobotPose, theGroundTruthRobotPose.timestamp));
  }

  RobotPose groundTruthRobotPose = currentGroundTruthRobotPose();
  RobotPose estimatedRobotPose = averageEstimatedRobotPose();

  float translationalError = (groundTruthRobotPose.translation
                              - estimatedRobotPose.translation).abs();
  PLOT("module:GroundTruthEvaluator:translationalError", translationalError);
  float rotationalError = fabs(groundTruthRobotPose.rotation
                               - estimatedRobotPose.rotation);
  if(rotationalError > pi) // Occures around +/-pi.
    rotationalError = pi2 - rotationalError;
  PLOT("module:GroundTruthEvaluator:rotationalError", rotationalError);

  ballModels.add(RepresentationWithTimestamp<BallModel>(theBallModel,
                 theFrameInfo.time));
  if(groundTruthBallModels.getNumberOfEntries() == 0 ||
     groundTruthBallModels.top().timestamp != theGroundTruthBallModel.timeWhenLastSeen)
  {
    groundTruthBallModels.add(RepresentationWithTimestamp<BallModel>(
                                theGroundTruthBallModel, theGroundTruthBallModel.timeWhenLastSeen));
  }

  BallModel groundTruthBallModel = currentGroundTruthBallModel();
  BallModel estimatedBallModel = averageEstimatedBallModel();

  float ballPositionError = (groundTruthBallModel.estimate.position
                             - estimatedBallModel.estimate.position).abs();
  PLOT("module:GroundTruthEvaluator:ballPositionError", ballPositionError);
  // TODO ball model velocity

  // TODO RobotsModel

  int evaluationDelay = 0;
  if(groundTruthRobotPoses.getNumberOfEntries() >= 2)
  {
    evaluationDelay = (int) theFrameInfo.time - (int) groundTruthRobotPoses[1].timestamp;
  }
  PLOT("module:GroundTruthEvaluator:evaluationDelay", evaluationDelay);
}

RobotPose GroundTruthEvaluator::currentGroundTruthRobotPose()
{
  RobotPose groundTruthRobotPose;
  if(groundTruthRobotPoses.getNumberOfEntries() >= 3)
    groundTruthRobotPose = groundTruthRobotPoses[1];
  return groundTruthRobotPose;
}

RobotPose GroundTruthEvaluator::averageEstimatedRobotPose()
{
  RobotPose averagedEstimatedRobotPose;
  unsigned aggregatedPoses = 0;

  if(groundTruthRobotPoses.getNumberOfEntries() >= 3)
  {
    unsigned previousTimestamp = groundTruthRobotPoses[2].timestamp;
    unsigned currentTimestamp = groundTruthRobotPoses[1].timestamp;
    unsigned nextTimestamp = groundTruthRobotPoses[0].timestamp;

    for(size_t i = 0; i < robotPoses.getNumberOfEntries(); i++)
    {
      int currentDistance = abs((int) robotPoses[i].timestamp - (int) currentTimestamp);
      int previousDistance = abs((int) robotPoses[i].timestamp - (int) previousTimestamp);
      if(currentDistance > previousDistance)
        continue;
      int nextDistance = abs((int) robotPoses[i].timestamp - (int) nextTimestamp);
      if(currentDistance > nextDistance)
        continue;

      // TODO weight by validity and time difference
      averagedEstimatedRobotPose.translation += robotPoses[i].translation;
      averagedEstimatedRobotPose.rotation += robotPoses[i].rotation;
      averagedEstimatedRobotPose.validity += robotPoses[i].validity;
      averagedEstimatedRobotPose.deviation += robotPoses[i].deviation;
      aggregatedPoses++;
    }

    if(aggregatedPoses)
    {
      averagedEstimatedRobotPose.translation /= (float) aggregatedPoses;
      averagedEstimatedRobotPose.rotation /= (float) aggregatedPoses;
      averagedEstimatedRobotPose.validity /= (float) aggregatedPoses;
      averagedEstimatedRobotPose.deviation /= (float) aggregatedPoses;
    }
  }

  return averagedEstimatedRobotPose;
}

BallModel GroundTruthEvaluator::currentGroundTruthBallModel()
{
  BallModel groundTruthBallModel;
  if(groundTruthBallModels.getNumberOfEntries() >= 3)
    groundTruthBallModel = groundTruthBallModels[1];
  return groundTruthBallModel;
}

BallModel GroundTruthEvaluator::averageEstimatedBallModel()
{
  BallModel averagedEstimatedBallModel;
  unsigned aggregatedModels = 0;

  if(groundTruthBallModels.getNumberOfEntries() >= 3)
  {
    unsigned previousTimestamp = groundTruthBallModels[2].timestamp;
    unsigned currentTimestamp = groundTruthBallModels[1].timestamp;
    unsigned nextTimestamp = groundTruthBallModels[0].timestamp;

    for(size_t i = 0; i < ballModels.getNumberOfEntries(); i++)
    {
      int currentDistance = abs((int) ballModels[i].timestamp - (int) currentTimestamp);
      int previousDistance = abs((int) ballModels[i].timestamp - (int) previousTimestamp);
      if(currentDistance > previousDistance)
        continue;
      int nextDistance = abs((int) ballModels[i].timestamp - (int) nextTimestamp);
      if(currentDistance > nextDistance)
        continue;

      // TODO weight by time difference
      averagedEstimatedBallModel.timeWhenLastSeen += ballModels[i].timeWhenLastSeen;
      averagedEstimatedBallModel.estimate.position += ballModels[i].estimate.position;
      averagedEstimatedBallModel.lastPerception += ballModels[i].lastPerception;
      aggregatedModels++;
    }

    if(aggregatedModels)
    {
      averagedEstimatedBallModel.timeWhenLastSeen /= aggregatedModels;
      averagedEstimatedBallModel.estimate.position /= (float) aggregatedModels;
      averagedEstimatedBallModel.lastPerception /= (float) aggregatedModels;
    }
  }

  return averagedEstimatedBallModel;
}

MAKE_MODULE(GroundTruthEvaluator, Infrastructure)
