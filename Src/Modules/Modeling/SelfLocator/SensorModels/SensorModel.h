/**
* @file SensorModel.h
*
* Declares an abstract base class for dífferent sensor models.
*
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
*/

#pragma once

#include "Tools/Math/Geometry.h"

class SelfLocatorParameter;

/**
* @class SensorModel
* Abstract base class
*/
class SensorModel
{
protected:
  /** Parameters of the self locator. Might become changed towards local parameters.*/
  const SelfLocatorParameters& theSelfLocatorParameters;
  /** Reference to frame information */
  const FrameInfo& theFrameInfo;
  /** Reference to field information */
  const FieldDimensions& theFieldDimensions;
  /** Reference to camera matrix */
  const CameraMatrix& theCameraMatrix;
  /** Module for checking perceptions before their use. */
  const PerceptValidityChecker& thePerceptValidityChecker;

  /** Computes a weighting for the angle difference between model and observation
  * @param measuredAngle The measured value
  * @param modelPosition The position of the sensed object in the global frame of reference
  * @param robotPose The pose of the robot (or the sample)
  * @param standardDeviation The standard deviation to become applied
  * @param bestPossibleWeighting Used for scaling to [0..1]
  * @return A weighting [0..1]
  */
  float computeAngleWeighting(float measuredAngle, const Vector2<>& modelPosition,
                              const Pose2D& robotPose, float standardDeviation, float bestPossibleWeighting)
  {
    const float modelAngle = Geometry::angleTo(robotPose, modelPosition);
    return gaussianProbability(abs(modelAngle - measuredAngle), standardDeviation) / bestPossibleWeighting;
  }

  /** Computes a weighting for the distance difference (described as angle) between model and observation
  * @param measuredDistanceAsAngle The angle between the vertical axis through the robot and the ray to the object
  * @param modelPosition The position of the sensed object in the global frame of reference
  * @param robotPose The pose of the robot (or the sample)
  * @param cameraZ The height of the camera
  * @param standardDeviation The standard deviation to become applied
  * @param bestPossibleWeighting Used for scaling to [0..1]
  * @return A weighting [0..1]
  */
  float computeDistanceWeighting(float measuredDistanceAsAngle, const Vector2<>& modelPosition,
                                 const Pose2D& robotPose, float cameraZ, float standardDeviation, float bestPossibleWeighting)
  {
    const float modelDistance = (robotPose.translation - modelPosition).abs();
    const float modelDistanceAsAngle = (pi_2 - atan2(cameraZ, modelDistance));
    return gaussianProbability(abs(modelDistanceAsAngle - measuredDistanceAsAngle),
                               standardDeviation) / bestPossibleWeighting;
  }

public:
  /** Possible observations. */
  class Observation
  {
  public:
    ENUM(Type,
      POINT,
      CORNER,
      GOAL_POST,
      CENTER_CIRCLE
    );
    Type type; /**< The type of the observation. */
    int index; /**< A hint how to find the observation in the corresponding data structure. */

    /** Default constructor. */
    Observation() {}

    /**
    * Constructor.
    * @param type The type of the observation.
    * @param index A hint how to find the observation in the corresponding data structure.
    */
    Observation(Type type, int index) : type(type), index(index) {}
  };

  const Observation::Type type; /**< The observation type processed by this sensor model. */

  /** Different result of computeWeightings function*/
  ENUM(SensorModelResult,
    NO_SENSOR_UPDATE,         // No weighting for any sample
    PARTIAL_SENSOR_UPDATE,        // Weightings for some samples
    FULL_SENSOR_UPDATE            // Weightings for all samples
  );

  /** Constructor. */
  SensorModel(const SelfLocatorParameters& selfLocatorParameters,
              const FrameInfo& frameInfo, const FieldDimensions& fieldDimensions,
              const CameraMatrix& cameraMatrix, const PerceptValidityChecker& perceptValidityChecker,
              Observation::Type type) :
    theSelfLocatorParameters(selfLocatorParameters), theFrameInfo(frameInfo),
    theFieldDimensions(fieldDimensions), theCameraMatrix(cameraMatrix),
    thePerceptValidityChecker(perceptValidityChecker), type(type)
  {}

  /** Destructor */
  virtual ~SensorModel() {}

  /** Function for computing weightings for a sample set.
  * @param samples The samples (not changed by this function
  * @param selectedIndices The indices of the selected observations.
  * @param weightings List of weightings. -1 means: no update
  * @return An overall result of the computation
  */
  virtual SensorModelResult computeWeightings(const SampleSet<SelfLocatorSample>& samples,
      const vector<int>& selectedIndices, vector<float>& weightings) = 0;
};
