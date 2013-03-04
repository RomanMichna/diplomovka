/**
* @file GoalPostsSensorModel.h
*
* Sensor model for updating samples by perceived goal posts
*
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
* @author <a href="mailto:thom@tzi.de">Thomas Münder</a>
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#pragma once


/**
* @class GoalPostsSensorModel
*/
class GoalPostsSensorModel: public SensorModel
{
private:
  /** Reference to goal percept which contains goal post information */
  const GoalPercept& theGoalPercept;
  /** Positions of the four goal posts*/
  Vector2<> unknownGoalPostPositions[4];
  /** Buffer for two known goal posts: one post and its mirrored version */
  Vector2<> knownGoalPostPositions[GoalPost::numOfPositions][2];

public:
  /** Constructor. */
  GoalPostsSensorModel(const SelfLocatorParameters& selfLocatorParameters,
                       const GoalPercept& goalPercept, const FrameInfo& frameInfo,
                       const FieldDimensions& fieldDimensions, const CameraMatrix& cameraMatrix,
                       const PerceptValidityChecker& perceptValidityChecker):
    SensorModel(selfLocatorParameters, frameInfo, fieldDimensions, cameraMatrix,
                perceptValidityChecker, Observation::GOAL_POST),
    theGoalPercept(goalPercept)
  {
    knownGoalPostPositions[GoalPost::IS_LEFT][0] = Vector2<>((float) theFieldDimensions.xPosOwnGoalpost, (float) theFieldDimensions.yPosRightGoal);
    knownGoalPostPositions[GoalPost::IS_LEFT][1] = Vector2<>((float) theFieldDimensions.xPosOpponentGoalpost, (float) theFieldDimensions.yPosLeftGoal);
    knownGoalPostPositions[GoalPost::IS_RIGHT][0] = Vector2<>((float) theFieldDimensions.xPosOwnGoalpost, (float) theFieldDimensions.yPosLeftGoal);
    knownGoalPostPositions[GoalPost::IS_RIGHT][1] = Vector2<>((float) theFieldDimensions.xPosOpponentGoalpost, (float) theFieldDimensions.yPosRightGoal);

    unknownGoalPostPositions[0] = Vector2<>((float) theFieldDimensions.xPosOpponentGoalpost, (float) theFieldDimensions.yPosLeftGoal);
    unknownGoalPostPositions[1] = Vector2<>((float) theFieldDimensions.xPosOpponentGoalpost, (float) theFieldDimensions.yPosRightGoal);
    unknownGoalPostPositions[2] = Vector2<>((float) theFieldDimensions.xPosOwnGoalpost, (float) theFieldDimensions.yPosRightGoal);
    unknownGoalPostPositions[3] = Vector2<>((float) theFieldDimensions.xPosOwnGoalpost, (float) theFieldDimensions.yPosLeftGoal);
  }

  /** Function for computing weightings for a sample set.
  * @param samples The samples (not changed by this function
  * @param selectedIndices The indices of the selected observations.
  * @param weightings List of weightings. -1 means: no update
  * @return An overall result of the computation
  */
  SensorModelResult computeWeightings(const SampleSet<SelfLocatorSample>& samples,
                                      const vector<int>& selectedIndices, vector<float>& weightings)
  {
    bool updated(false);
    const float& camZ = theCameraMatrix.translation.z;
    if(theGoalPercept.goalPosts[selectedIndices[0]].position != GoalPost::IS_UNKNOWN)
    {
      // Identified (left or right) goal posts
      for(vector<int>::const_iterator i = selectedIndices.begin(); i != selectedIndices.end(); ++i)
      {
        const GoalPost& post = theGoalPercept.goalPosts[*i];
        // Precompute stuff:
        const float distanceStdDev = (post.distanceType == GoalPost::HEIGHT_BASED) ?
                                     theSelfLocatorParameters.standardDeviationGoalpostSizeDistance : theSelfLocatorParameters.standardDeviationGoalpostBearingDistance;
        const float bestPossibleAngleWeighting    = gaussianProbability(0.0f, theSelfLocatorParameters.standardDeviationGoalpostAngle);
        const float bestPossibleDistanceWeighting = gaussianProbability(0.0f, distanceStdDev);
        const Vector2<> pField((float) post.positionOnField.x, (float) post.positionOnField.y);
        const float angleObserved = pField.angle();
        const float distanceAsAngleObserved = (pi_2 - atan2(camZ, pField.abs()));

        // Iterate over all samples and compute weightings:
        for(int i = 0; i < samples.size(); ++i)
        {
          const SelfLocatorSample& s(samples.at(i));
          const Pose2D sPose(s.angle, (float) s.translation.x, (float) s.translation.y);
          float maxWeighting = 0.f;
          for(int j = 0; j < 2; ++j)
          {
            float weighting = computeAngleWeighting(angleObserved,
                                                    knownGoalPostPositions[post.position][j], sPose, theSelfLocatorParameters.standardDeviationGoalpostAngle,
                                                    bestPossibleAngleWeighting) *
                              computeDistanceWeighting(distanceAsAngleObserved, knownGoalPostPositions[post.position][j],
                                                       sPose, camZ, distanceStdDev, bestPossibleDistanceWeighting);
            if(weighting > maxWeighting)
              maxWeighting = weighting;
          }
          if(updated)
            weightings[i] *= maxWeighting;
          else
            weightings[i] = maxWeighting;
        }
        updated = true;
      }
    }
    else
    {
      // Use unknown posts only, if there has not been an update by a "normal" post
      for(vector<int>::const_iterator i = selectedIndices.begin(); i != selectedIndices.end(); ++i)
      {
        const GoalPost& post = theGoalPercept.goalPosts[*i];
        const float distanceStdDev = (post.distanceType == GoalPost::HEIGHT_BASED) ?
                                     theSelfLocatorParameters.standardDeviationGoalpostSizeDistance : theSelfLocatorParameters.standardDeviationGoalpostBearingDistance;
        const float bestPossibleAngleWeighting    = gaussianProbability(0.0f, theSelfLocatorParameters.standardDeviationGoalpostAngle);
        const float bestPossibleDistanceWeighting = gaussianProbability(0.0f, distanceStdDev);

        const Vector2<> pField((float) post.positionOnField.x, (float) post.positionOnField.y);
        const float distanceAsAngleObserved = (pi_2 - atan2(camZ, pField.abs()));
        const float angleObserved = pField.angle();
        // Iterate over all samples and compute weightings:
        for(int i = 0; i < samples.size(); ++i)
        {
          const SelfLocatorSample& s(samples.at(i));
          const Pose2D sPose(s.angle, (float) s.translation.x, (float) s.translation.y);
          float maxWeighting = 0.f;
          for(int j = 0; j < 4; ++j)
          {
            float weighting = computeAngleWeighting(angleObserved,
                                                    unknownGoalPostPositions[j], sPose, theSelfLocatorParameters.standardDeviationGoalpostAngle,
                                                    bestPossibleAngleWeighting) *
                              computeDistanceWeighting(distanceAsAngleObserved, unknownGoalPostPositions[j],
                                                       sPose, camZ, distanceStdDev, bestPossibleDistanceWeighting);
            if(weighting > maxWeighting)
              maxWeighting = weighting;
          }
          if(updated)
            weightings[i] *= maxWeighting;
          else
            weightings[i] = maxWeighting;
        }
        updated = true;
      }
    }
    return FULL_SENSOR_UPDATE;
  }
};
