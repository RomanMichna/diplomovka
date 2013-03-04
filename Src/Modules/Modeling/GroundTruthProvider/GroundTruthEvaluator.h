/**
 * @file GroundTruthEvaluator.h
 * Compares estimated models with ground truth. The ground truth can either be
 * provided by the small size vision or by the simulator. The compared
 * representations will be synchronized.
 * @author <a href="mailto:afabisch@tzi.de">Alexander Fabisch</a>
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/GroundTruthResult.h"
#include "Tools/LimitedPriorityQueue.h"

MODULE(GroundTruthEvaluator)
  REQUIRES(FrameInfo)
  REQUIRES(RobotPose)
  REQUIRES(GroundTruthRobotPose)
  REQUIRES(BallModel)
  REQUIRES(GroundTruthBallModel)
  PROVIDES_WITH_MODIFY(GroundTruthResult)
END_MODULE

template<class R>
class RepresentationWithTimestamp : public R
{
public:
  unsigned timestamp;

  RepresentationWithTimestamp()
    : timestamp(0)
  {}

  RepresentationWithTimestamp(const R& representation, unsigned timestamp)
    : R(representation), timestamp(timestamp)
  {
  }

  bool operator<(const RepresentationWithTimestamp<R>& b) const
  {
    return timestamp < b.timestamp;
  }
};

class GroundTruthEvaluator : public GroundTruthEvaluatorBase
{
  void init();

  void update(GroundTruthResult& groundTruthResult);
  RobotPose currentGroundTruthRobotPose();
  RobotPose averageEstimatedRobotPose();
  BallModel currentGroundTruthBallModel();
  BallModel averageEstimatedBallModel();

  LimitedPriorityQueue<RepresentationWithTimestamp<RobotPose> > robotPoses;
  LimitedPriorityQueue<RepresentationWithTimestamp<RobotPose> > groundTruthRobotPoses;
  LimitedPriorityQueue<RepresentationWithTimestamp<BallModel> > ballModels;
  LimitedPriorityQueue<RepresentationWithTimestamp<BallModel> > groundTruthBallModels;
};
