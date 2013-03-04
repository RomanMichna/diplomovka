/**
 * @file GroundTruthProvider.h
 * @author <a href="mailto:afabisch@tzi.de">Alexander Fabisch</a>
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/SSLVisionData.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/RobotsModel.h"

#include "Tools/Streams/InStreams.h"
#include "Tools/Streams/Streamable.h"

MODULE(GroundTruthProvider)
  REQUIRES(FrameInfo)
  REQUIRES(SSLVisionData)
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(BallModel)
  PROVIDES_WITH_MODIFY_AND_OUTPUT(RobotPose)
  PROVIDES_WITH_DRAW(RobotsModel)
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(GroundTruthBallModel)
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(GroundTruthRobotPose)
  PROVIDES_WITH_DRAW(GroundTruthRobotsModel)
END_MODULE

/**
 * @class GroundTruthProvider
 * This class provides the ground truth data based on the SSLVisionData. Note
 * that these data are delayed because they have to be sent via network.
 */
class GroundTruthProvider : public GroundTruthProviderBase
{
  class Parameters : public Streamable
  {
    void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN;
      STREAM(robotId);
      STREAM_REGISTER_FINISH;
    }

  public:
    int robotId; /**< The id of the SSL pattern. */
  };

  Parameters params;
  unsigned lastUpdateFrameTime;
  unsigned groundTruthTimestamp;
  BallModel currentBallModel;
  RobotPose currentRobotPose;
  RobotsModel currentRobotsModel;

  void init();
  void setCurrentGroundTruth();
  void update(BallModel& ballModel);
  void update(GroundTruthBallModel& groundTruthBallModel);
  void update(RobotPose& robotPose);
  void update(GroundTruthRobotPose& groundTruthRobotPose);
  void update(RobotsModel& robotsModel);
  void update(GroundTruthRobotsModel& groundTruthRobotsModel);
};
