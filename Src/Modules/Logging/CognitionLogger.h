/*
 * CognitionLogger.h
 *
 *  Created on: Feb 10, 2012
 *      Author: arne
 */

#ifndef COGNITIONLOGGER_H_
#define COGNITIONLOGGER_H_

#include "Logger.h"
#include "Representations/Logging/CognitionLoggerOutput.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Perception/BallPercept.h"
#include "Representations/Modeling/ArmContactModel.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/CombinedWorldModel.h"
#include "Representations/Modeling/FieldCoverage.h"
#include "Representations/Modeling/FreePartOfOpponentGoalModel.h"
#include "Representations/Modeling/GlobalFieldCoverage.h"
#include "Representations/Modeling/GroundTruthResult.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/RobotsModel.h"
#include "Representations/Modeling/USObstacleGrid.h"
#include "Representations/Perception/BodyContour.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/FootPercept.h"
#include "Representations/Perception/GoalPercept.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Perception/LinePercept.h"
#include "Representations/Perception/RegionPercept.h"
#include "Representations/Perception/RobotPercept.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/InertiaSensorData.h"
#include "Representations/Sensing/OrientationData.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/BehaviorControl/BehaviorControlOutput.h"
#include "Representations/BehaviorControl/BehaviorDebugOutput.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Modeling/SideConfidence.h"
#include "Representations/MotionControl/MotionInfo.h"
/**
 * The Logger requires all representations.
 * If you create a new representation and want to log it, the Logger has to(!!!) require it.
 *
 */



MODULE(CognitionLogger)
REQUIRES(GameInfo)
REQUIRES(RobotModel)
//REQUIRES(OrientationData)
//REQUIRES(InertiaSensorData)
REQUIRES(FallDownState)
REQUIRES(RobotPercept)
REQUIRES(RegionPercept)
REQUIRES(LinePercept)
REQUIRES(ImageCoordinateSystem)
REQUIRES(GoalPercept)
REQUIRES(FootPercept)
REQUIRES(BodyContour)
REQUIRES(RobotsModel)
REQUIRES(SideConfidence)
REQUIRES(USObstacleGrid)
REQUIRES(BallModel)
REQUIRES(CombinedWorldModel)
REQUIRES(CameraMatrix)
REQUIRES(ObstacleModel)
REQUIRES(RobotPose)
REQUIRES(BallPercept)
REQUIRES(BehaviorControlOutput)
REQUIRES(GlobalFieldCoverage)
REQUIRES(MotionRequest)
REQUIRES(HeadMotionRequest)
REQUIRES(ArmContactModel)
REQUIRES(FieldCoverage)
REQUIRES(FreePartOfOpponentGoalModel)
REQUIRES(FilteredJointData)
REQUIRES(FilteredSensorData)
REQUIRES(FrameInfo)
REQUIRES(OwnTeamInfo)
REQUIRES(PotentialRobotPose)
REQUIRES(MotionInfo)
REQUIRES(BehaviorDebugOutput)
REQUIRES(RobotInfo) //do not remove, theRobotInfo is used to determine the penalty state
PROVIDES(CognitionLoggerOutput) //The output is not used at all, but it has to be there to get the update method
END_MODULE

using namespace std;

class CognitionLogger : public Logger, public CognitionLoggerBase
{

private:

  /**
   * Contains pointers to all representations that should  be logged.
   */
  list<pair<MessageID,const Streamable*> > m_representations;

  /**
   * Mapping from representation names to the actual representation.
   */
  map<RepresentationName,const Streamable*> theRepresentationRefs;

public:
  CognitionLogger();
  virtual
  ~CognitionLogger();

  /**
   * This method should provide the representations that should be logged.
   */
  virtual list<pair<MessageID,const Streamable*> >& provideRepresentations();

  /**
   * This method should provide the process name.
   * It is called every frame.
   */
  virtual ProcessName getProcessName();

  void update(CognitionLoggerOutput& output);


private:
  /**
   * Name of the Process to which all representations logged by this logger belong
   */
  Logger::ProcessName theProcessName;
  void initRepresentations();

};

#endif /* COGNITIONLOGGER_H_ */
