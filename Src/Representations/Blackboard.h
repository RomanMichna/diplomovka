/**
* @file Blackboard.h
* Declaration of a class representing the blackboard.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#pragma once

#include <cstddef>
#include "Platform/SystemCall.h"

// Declare prototypes of all representations here:

// Infrastructure
class JointData;
class JointRequest;
class SensorData;
class KeyStates;
class LEDRequest;
class Image;
class ImageOther;
class CameraInfo;
class FrameInfo;
class CognitionFrameInfo;
class RobotInfo;
class OwnTeamInfo;
class OpponentTeamInfo;
class GameInfo;
class SoundRequest;
class SoundOutput;
class TeamMateData;
class MotionRobotHealth;
class RobotHealth;
class TeamDataSenderOutput;
class USRequest;

// Configuration
class ColorTable64;
class CameraSettings;
class FieldDimensions;
class RobotDimensions;
class JointCalibration;
class SensorCalibration;
class CameraCalibration;
class MassCalibration;
class HardnessSettings;
class PassParameters;
class DamageConfiguration;
class HeadLimits;
class ColorConfiguration;

// Perception
class CameraMatrix;
class CameraMatrixOther;
class RobotCameraMatrix;
class RobotCameraMatrixOther;
class ImageCoordinateSystem;
class BallSpots;
class LineSpots;
class BallPercept;
class LinePercept;
class RegionPercept;
class GoalPercept;
class GroundContactState;
class BodyContour;
class TeamMarkerSpots;
class RobotPercept;
class ImageInfo;
class ImageRequest;
class FootPercept;
class RunLengthImage;
class ImageGrid;

// Modeling
class ArmContactModel;
class FallDownState;
class BallModel;
class CombinedWorldModel;
class GroundTruthBallModel;
class ObstacleModel;
class USObstacleGrid;
class RobotPose;
class PotentialRobotPose;
class FilteredRobotPose;
class FootContactModel;
class GroundTruthRobotPose;
class RobotsModel;
class GroundTruthRobotsModel;
class FreePartOfOpponentGoalModel;
class SSLVisionData;
class GroundTruthResult;
class FieldCoverage;
class GlobalFieldCoverage;
class SimpleFootModel;
class SideConfidence;
class Odometer;
class OwnSideModel;

// BehaviorControl
class BehaviorControlOutput;
class BehaviorLEDRequest;
class BehaviorDebugOutput;

// Sensing
class FilteredJointData;
class FilteredSensorData;
class InertiaSensorData;
class InspectedInertiaSensorData;
class OrientationData;
class GroundTruthOrientationData;
class TorsoMatrix;
class RobotModel;

// MotionControl
class OdometryData;
class GroundTruthOdometryData;
class MotionRequest;
class HeadMotionRequest;
class HeadAngleRequest;
class HeadJointRequest;
class MotionSelection;
class SpecialActionsOutput;
class WalkingEngineOutput;
class WalkingEngineStandOutput;
class BikeEngineOutput;
class MotionInfo;

// Debugging
class MatchStatistic;

//Logging
class CognitionLoggerOutput;

// friends
class Process;
class Cognition;
class Motion;
class Framework;

/**
* @class Blackboard
* The class represents the blackboard that contains all representation.
* Note: The blackboard only contains references to the objects as attributes.
* The actual representations are constructed on the heap, because many copies of
* of the blackboard exist but only a single set of the representations shared
* by all instances.
*/
class Blackboard
{
protected:
  // Add all representations as constant references here:
  // Infrastructure
  const JointData& theJointData;
  const JointRequest& theJointRequest;
  const SensorData& theSensorData;
  const KeyStates& theKeyStates;
  const LEDRequest& theLEDRequest;
  const Image& theImage;
  const ImageOther& theImageOther;
  const CameraInfo& theCameraInfo;
  const FrameInfo& theFrameInfo;
  const CognitionFrameInfo& theCognitionFrameInfo;
  const RobotInfo& theRobotInfo;
  const OwnTeamInfo& theOwnTeamInfo;
  const OpponentTeamInfo& theOpponentTeamInfo;
  const GameInfo& theGameInfo;
  const SoundRequest& theSoundRequest;
  const SoundOutput& theSoundOutput;
  const TeamMateData& theTeamMateData;
  const MotionRobotHealth& theMotionRobotHealth;
  const RobotHealth& theRobotHealth;
  const TeamDataSenderOutput& theTeamDataSenderOutput;
  const USRequest& theUSRequest;

  // Configuration
  const ColorTable64& theColorTable64;
  const CameraSettings& theCameraSettings;
  const FieldDimensions& theFieldDimensions;
  const RobotDimensions& theRobotDimensions;
  const JointCalibration& theJointCalibration;
  const SensorCalibration& theSensorCalibration;
  const CameraCalibration& theCameraCalibration;
  const MassCalibration& theMassCalibration;
  const HardnessSettings& theHardnessSettings;
  const PassParameters& thePassParameters;
  const DamageConfiguration& theDamageConfiguration;
  const HeadLimits& theHeadLimits;
  const ColorConfiguration& theColorConfiguration;

  // Perception
  const CameraMatrix& theCameraMatrix;
  const CameraMatrixOther& theCameraMatrixOther;
  const RobotCameraMatrix& theRobotCameraMatrix;
  const RobotCameraMatrixOther& theRobotCameraMatrixOther;
  const ImageCoordinateSystem& theImageCoordinateSystem;
  const BallSpots& theBallSpots;
  const LineSpots& theLineSpots;
  const BallPercept& theBallPercept;
  const LinePercept& theLinePercept;
  const RegionPercept& theRegionPercept;
  const GoalPercept& theGoalPercept;
  const GroundContactState& theGroundContactState;
  const BodyContour& theBodyContour;
  const TeamMarkerSpots& theTeamMarkerSpots;
  const RobotPercept& theRobotPercept;
  const ImageInfo& theImageInfo;
  const ImageRequest& theImageRequest;
  const FootPercept& theFootPercept;
  const RunLengthImage& theRunLengthImage;
  const ImageGrid& theImageGrid;

  // Modeling
  const ArmContactModel& theArmContactModel;
  const FallDownState& theFallDownState;
  const BallModel& theBallModel;
  const CombinedWorldModel& theCombinedWorldModel;
  const GroundTruthBallModel& theGroundTruthBallModel;
  const ObstacleModel& theObstacleModel;
  const USObstacleGrid& theUSObstacleGrid;
  const RobotPose& theRobotPose;
  const PotentialRobotPose& thePotentialRobotPose;
  const FilteredRobotPose& theFilteredRobotPose;
  const FootContactModel& theFootContactModel;
  const GroundTruthRobotPose& theGroundTruthRobotPose;
  const RobotsModel& theRobotsModel;
  const GroundTruthRobotsModel& theGroundTruthRobotsModel;
  const FreePartOfOpponentGoalModel& theFreePartOfOpponentGoalModel;
  const SSLVisionData& theSSLVisionData;
  const GroundTruthResult& theGroundTruthResult;
  const FieldCoverage& theFieldCoverage;
  const GlobalFieldCoverage& theGlobalFieldCoverage;
  const SimpleFootModel& theSimpleFootModel;
  const SideConfidence& theSideConfidence;
  const Odometer& theOdometer;
  const OwnSideModel& theOwnSideModel;


  // BehaviorControl
  const BehaviorControlOutput& theBehaviorControlOutput;
  const BehaviorLEDRequest& theBehaviorLEDRequest;
  const BehaviorDebugOutput& theBehaviorDebugOutput;

  // Sensing
  const FilteredJointData& theFilteredJointData;
  const FilteredSensorData& theFilteredSensorData;
  const InertiaSensorData& theInertiaSensorData;
  const InspectedInertiaSensorData& theInspectedInertiaSensorData;
  const OrientationData& theOrientationData;
  const GroundTruthOrientationData& theGroundTruthOrientationData;
  const TorsoMatrix& theTorsoMatrix;
  const RobotModel& theRobotModel;

  // MotionControl
  const OdometryData& theOdometryData;
  const GroundTruthOdometryData& theGroundTruthOdometryData;
  const MotionRequest& theMotionRequest;
  const HeadAngleRequest& theHeadAngleRequest;
  const HeadMotionRequest& theHeadMotionRequest;
  const HeadJointRequest& theHeadJointRequest;
  const MotionSelection& theMotionSelection;
  const SpecialActionsOutput& theSpecialActionsOutput;
  const WalkingEngineOutput& theWalkingEngineOutput;
  const WalkingEngineStandOutput& theWalkingEngineStandOutput;
  const BikeEngineOutput& theBikeEngineOutput;
  const MotionInfo& theMotionInfo;

  // Debugging
  const MatchStatistic& theMatchStatistic;

  //Logging
  const CognitionLoggerOutput& theCognitionLoggerOutput;

  PROCESS_WIDE_STORAGE_STATIC(Blackboard) theInstance; /**< The only real instance in the current process. */

  /**
  * The method is a dummy that is called to prevent the compiler from certain
  * optimizations in a method generated in Module.h.
  * It is empty, but important, not defined inline.
  */
  static void distract();

private:
  /**
  * Default constructor.
  */
  Blackboard();

public:
  /**
  * Virtual destructor.
  * Required for derivations of this class.
  */
  virtual ~Blackboard() {}

  /**
  * Assignment operator.
  * Note: copies will share all representations.
  * @param other The instance that is cloned.
  */
  void operator=(const Blackboard& other);

  /**
  * The operator allocates a memory block that is zeroed.
  * Therefore, all members of this class are initialized with 0.
  * @attention This operator is only called if this class is instantiated by
  * a separate call to new, i.e. it cannot be created as a part of another class.
  * @param size The size of the block in bytes.
  * @return A pointer to the block.
  */
  static void* operator new(std::size_t);

  /**
  * The operator frees a memory block.
  * @param p The address of the block to free.
  */
  static void operator delete(void* p);

  friend class Process; /**< The class Process can set theInstance. */
  friend class Cognition; /**< The class Cognition can read theInstance. */
  friend class Motion; /**< The class Motion can read theInstance. */
  friend class Framework; /**< The class Framework can set theInstance. */
};
