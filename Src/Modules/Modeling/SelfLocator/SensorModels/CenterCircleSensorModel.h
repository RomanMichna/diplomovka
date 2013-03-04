/**
* @file CenterCircleSensorModel.h
*
* Sensor model for updating samples by perceived center circles
*
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
*/

#pragma once


/**
* @class CenterCircleSensorModel
*/
class CenterCircleSensorModel: public SensorModel
{
private:
  /** Reference to line percept which contains center circle information */
  const LinePercept& theLinePercept;

public:
  /** Constructor. */
  CenterCircleSensorModel(const SelfLocatorParameters& selfLocatorParameters,
                          const LinePercept& linePercept, const FrameInfo& frameInfo,
                          const FieldDimensions& fieldDimensions, const CameraMatrix& cameraMatrix,
                          const PerceptValidityChecker& perceptValidityChecker):
    SensorModel(selfLocatorParameters, frameInfo, fieldDimensions, cameraMatrix,
                perceptValidityChecker, Observation::CENTER_CIRCLE),
    theLinePercept(linePercept) {}

  /** Function for computing weightings for a sample set.
  * @param samples The samples (not changed by this function
  * @param selectedIndices The indices of the selected observations.
  * @param weightings List of weightings. -1 means: no update
  * @return An overall result of the computation
  */
  SensorModelResult computeWeightings(const SampleSet<SelfLocatorSample>& samples,
                                      const vector<int>& selectedIndices, vector<float>& weightings)
  {
    SensorModelResult result = FULL_SENSOR_UPDATE;
    // Precompute some stuff:
    result = FULL_SENSOR_UPDATE;
    const short distanceObserved = static_cast<short>(theLinePercept.circle.pos.abs());
    const float angleObserved = theLinePercept.circle.pos.angle();
    const float& camZ = theCameraMatrix.translation.z;
    const float distanceAsAngleObserved = (pi_2 - atan2(camZ, distanceObserved));
    const float bestPossibleAngleWeighting    = gaussianProbability(0.0f, theSelfLocatorParameters.standardDeviationCenterCircleAngle);
    const float bestPossibleDistanceWeighting = gaussianProbability(0.0f, theSelfLocatorParameters.standardDeviationCenterCircleDistance);
    const Vector2<> centerCirclePosition(0, 0);
    // Iterate over all samples and compute weightings:
    for(int i = 0; i < samples.size(); ++i)
    {
      const SelfLocatorSample& s(samples.at(i));
      const Pose2D sPose(s.angle, (float) s.translation.x, (float) s.translation.y);
      // Check, if circle might have been confused with goal net:
      if(thePerceptValidityChecker.pointIsProbablyInGoalNet(sPose, distanceObserved, angleObserved))
      {
        weightings[i] = -1;
        result = PARTIAL_SENSOR_UPDATE;
        continue;
      }
      // Compute weighting:
      weightings[i] = computeAngleWeighting(angleObserved, centerCirclePosition, sPose,
                                            theSelfLocatorParameters.standardDeviationCenterCircleAngle, bestPossibleAngleWeighting);
      weightings[i] *= computeDistanceWeighting(distanceAsAngleObserved, centerCirclePosition,
                       sPose, camZ, theSelfLocatorParameters.standardDeviationCenterCircleDistance, bestPossibleDistanceWeighting);
    }
    return result;
  }
};
