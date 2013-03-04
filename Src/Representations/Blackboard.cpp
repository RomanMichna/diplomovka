/**
* @file Blackboard.cpp
* Implementation of a class representing the blackboard.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas R�fer</a>
*/

#include "Blackboard.h"
#include <cstring>
#include <cstdlib>

Blackboard::Blackboard() :
// Initialize all representations by themselves:
// Infrastructure
  theJointData(theJointData),
  theJointRequest(theJointRequest),
  theSensorData(theSensorData),
  theKeyStates(theKeyStates),
  theLEDRequest(theLEDRequest),
  theImage(theImage),
  theImageOther(theImageOther),
  theCameraInfo(theCameraInfo),
  theFrameInfo(theFrameInfo),
  theCognitionFrameInfo(theCognitionFrameInfo),
  theRobotInfo(theRobotInfo),
  theOwnTeamInfo(theOwnTeamInfo),
  theOpponentTeamInfo(theOpponentTeamInfo),
  theGameInfo(theGameInfo),
  theSoundRequest(theSoundRequest),
  theSoundOutput(theSoundOutput),
  theTeamMateData(theTeamMateData),
  theMotionRobotHealth(theMotionRobotHealth),
  theRobotHealth(theRobotHealth),
  theTeamDataSenderOutput(theTeamDataSenderOutput),
  theUSRequest(theUSRequest),

// Configuration
  theColorTable64(theColorTable64),
  theCameraSettings(theCameraSettings),
  theFieldDimensions(theFieldDimensions),
  theRobotDimensions(theRobotDimensions),
  theJointCalibration(theJointCalibration),
  theSensorCalibration(theSensorCalibration),
  theCameraCalibration(theCameraCalibration),
  theMassCalibration(theMassCalibration),
  theHardnessSettings(theHardnessSettings),
  thePassParameters(thePassParameters),
  theDamageConfiguration(theDamageConfiguration),
  theHeadLimits(theHeadLimits),
  theColorConfiguration(theColorConfiguration),

// Perception
  theCameraMatrix(theCameraMatrix),
  theCameraMatrixOther(theCameraMatrixOther),
  theRobotCameraMatrix(theRobotCameraMatrix),
  theRobotCameraMatrixOther(theRobotCameraMatrixOther),
  theImageCoordinateSystem(theImageCoordinateSystem),
  theBallSpots(theBallSpots),
  theLineSpots(theLineSpots),
  theBallPercept(theBallPercept),
  theLinePercept(theLinePercept),
  theRegionPercept(theRegionPercept),
  theGoalPercept(theGoalPercept),
  theGroundContactState(theGroundContactState),
  theBodyContour(theBodyContour),
  theTeamMarkerSpots(theTeamMarkerSpots),
  theRobotPercept(theRobotPercept),
  theImageInfo(theImageInfo),
  theImageRequest(theImageRequest),
  theFootPercept(theFootPercept),
  theRunLengthImage(theRunLengthImage),
  theImageGrid(theImageGrid),

// Modeling
  theArmContactModel(theArmContactModel),
  theFallDownState(theFallDownState),
  theBallModel(theBallModel),
  theCombinedWorldModel(theCombinedWorldModel),
  theGroundTruthBallModel(theGroundTruthBallModel),
  theObstacleModel(theObstacleModel),
  theUSObstacleGrid(theUSObstacleGrid),
  theRobotPose(theRobotPose),
  thePotentialRobotPose(thePotentialRobotPose),
  theFilteredRobotPose(theFilteredRobotPose),
  theFootContactModel(theFootContactModel),
  theGroundTruthRobotPose(theGroundTruthRobotPose),
  theRobotsModel(theRobotsModel),
  theGroundTruthRobotsModel(theGroundTruthRobotsModel),
  theFreePartOfOpponentGoalModel(theFreePartOfOpponentGoalModel),
  theSSLVisionData(theSSLVisionData),
  theGroundTruthResult(theGroundTruthResult),
  theFieldCoverage(theFieldCoverage),
  theGlobalFieldCoverage(theGlobalFieldCoverage),
  theSimpleFootModel(theSimpleFootModel),
  theSideConfidence(theSideConfidence),
  theOdometer(theOdometer),
  theOwnSideModel(theOwnSideModel),


// BehaviorControl
  theBehaviorControlOutput(theBehaviorControlOutput),
  theBehaviorLEDRequest(theBehaviorLEDRequest),
  theBehaviorDebugOutput(theBehaviorDebugOutput),

// Sensing
  theFilteredJointData(theFilteredJointData),
  theFilteredSensorData(theFilteredSensorData),
  theInertiaSensorData(theInertiaSensorData),
  theInspectedInertiaSensorData(theInspectedInertiaSensorData),
  theOrientationData(theOrientationData),
  theGroundTruthOrientationData(theGroundTruthOrientationData),
  theTorsoMatrix(theTorsoMatrix),
  theRobotModel(theRobotModel),

// MotionControl
  theOdometryData(theOdometryData),
  theGroundTruthOdometryData(theGroundTruthOdometryData),
  theMotionRequest(theMotionRequest),
  theHeadAngleRequest(theHeadAngleRequest),
  theHeadMotionRequest(theHeadMotionRequest),
  theHeadJointRequest(theHeadJointRequest),
  theMotionSelection(theMotionSelection),
  theSpecialActionsOutput(theSpecialActionsOutput),
  theWalkingEngineOutput(theWalkingEngineOutput),
  theWalkingEngineStandOutput(theWalkingEngineStandOutput),
  theBikeEngineOutput(theBikeEngineOutput),
  theMotionInfo(theMotionInfo),

// Debugging
  theMatchStatistic(theMatchStatistic),

//Logging
  theCognitionLoggerOutput(theCognitionLoggerOutput)
{
}

void Blackboard::operator=(const Blackboard& other)
{
  memcpy(this, &other, sizeof(Blackboard));
}

void* Blackboard::operator new(std::size_t size)
{
  return calloc(1, size);
}

void Blackboard::operator delete(void* p)
{
  return free(p);
}

void Blackboard::distract()
{
}

PROCESS_WIDE_STORAGE(Blackboard) Blackboard::theInstance = 0;
