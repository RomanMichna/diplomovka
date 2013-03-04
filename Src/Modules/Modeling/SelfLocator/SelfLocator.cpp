/**
* @file SelfLocator.cpp
* Implements a class that performs self-localization using a particle filter.
*
* - May choose different implementations for pose calculation
*
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas R�fer</a>
*/

#include <limits>
#include <iostream>
#include "SelfLocator.h"
#include "PoseCalculator/PoseCalculatorParticleHistory.h"
#include "PoseCalculator/PoseCalculator2DBinning.h"
#include "PoseCalculator/PoseCalculatorBestParticle.h"
#include "PoseCalculator/PoseCalculatorOverallAverage.h"
#include "PoseCalculator/PoseCalculatorKMeansClustering.h"
#include "Tools/Math/GaussianDistribution3D.h"
#include "Tools/Settings.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Streams/OutStreams.h"
#include "SensorModels/CenterCircleSensorModel.h"
#include "SensorModels/GoalPostsSensorModel.h"
#include "SensorModels/LineSensorModel.h"
#include "SensorModels/CornersSensorModel.h"
#include <algorithm>

SelfLocator::SelfLocator() :
  samples(0), slowWeighting(0.0f), fastWeighting(0.0f), updatedBySensors(false),
  poseCalculator(0), perceptValidityChecker(0),
  gameStateHandler(parameters, theFieldDimensions, theGameInfo, theOwnTeamInfo, theRobotInfo, 
                   theRobotPose, theOdometryData, theSideConfidence, theOwnSideModel, theGroundContactState),
  sampleTemplateGenerator(parameters,
                          theGoalPercept, theLinePercept, theFrameInfo, theFieldDimensions, theOdometryData)
{
  InConfigMap stream(Global::getSettings().expandLocationFilename("selfLocator.cfg"));
  ASSERT(stream.exists());
  stream >> parameters;
  
  observations.reserve(100);
  selectedObservations.reserve(100);
  selectedIndices.reserve(100);
  lines.reserve(100);
}

void SelfLocator::setNumberOfSamples(const int num)
{
  if(samples)
    delete samples;
  samples = new SampleSet<Sample>(num);
  sensorModelWeightings.resize(num);

  if(poseCalculator)
    delete poseCalculator;
  poseCalculator = new PoseCalculatorParticleHistory< Sample, SampleSet<Sample> >(*samples);
  poseCalculatorType = POSE_CALCULATOR_PARTICLE_HISTORY;
}

SelfLocator::~SelfLocator()
{
  delete poseCalculator;
  delete samples;
  for(unsigned int i = 0; i < sensorModels.size(); ++i)
    delete sensorModels[i];
  if(perceptValidityChecker)
    delete perceptValidityChecker;
}

void SelfLocator::init()
{
  fieldBoundary = Boundary<int>(Range<int>(theFieldDimensions.xPosOwnFieldBorder, theFieldDimensions.xPosOpponentFieldBorder), Range<int>(theFieldDimensions.yPosRightFieldBorder, theFieldDimensions.yPosLeftFieldBorder));
  fieldModel.init(theFieldDimensions, parameters.maxCrossingLength);
  for(unsigned int i = 0; i < sensorModels.size(); ++i)
    delete sensorModels[i];
  sensorModels.clear();
  if(perceptValidityChecker)
    delete perceptValidityChecker;
  perceptValidityChecker = new PerceptValidityChecker(theFieldDimensions);
  sensorModels.push_back(new GoalPostsSensorModel(parameters,
                         theGoalPercept, theFrameInfo, theFieldDimensions, theCameraMatrix, *perceptValidityChecker));
  sensorModels.push_back(new CenterCircleSensorModel(parameters,
                         theLinePercept, theFrameInfo, theFieldDimensions, theCameraMatrix, *perceptValidityChecker));
  sensorModels.push_back(new LineSensorModel(parameters,
                         lines, theFrameInfo, theFieldDimensions, theCameraMatrix, *perceptValidityChecker,
                         fieldModel));
  sensorModels.push_back(new CornersSensorModel(parameters,
                         theLinePercept, theFrameInfo, theFieldDimensions, theCameraMatrix, *perceptValidityChecker,
                         fieldModel));

  setNumberOfSamples(parameters.numberOfSamples);
  sampleTemplateGenerator.init();
}

void SelfLocator::initSamplesAtGivenPositions(const vector<Pose2D>& poses,
    const vector<Pose2D>& standardDeviations)
{
  for(int i = 0; i < samples->size(); ++i)
  {
    Sample& sample(samples->at(i));
    if(poses.size())
    {
      // Select one of the poses from the list:
      unsigned int index = random((short)poses.size());
      Pose2D pose = poses[index];
      Pose2D stdDev = standardDeviations[index];
      // Create sample:
      sample.translation.x = static_cast<int>(pose.translation.x);
      sample.translation.x += sampleTriangularDistribution(static_cast<int>(stdDev.translation.x));
      sample.translation.y = static_cast<int>(pose.translation.y);
      sample.translation.y += sampleTriangularDistribution(static_cast<int>(stdDev.translation.y));
      float rotation = normalize(pose.rotation + sampleTriangularDistribution(stdDev.rotation));
      sample.angle = rotation;
      sample.rotation = Vector2<int>(int(cos(rotation) * 1024), int(sin(rotation) * 1024));
    }
    else //No given positions, spread uniformly in own half:
    {
      Pose2D pose(theFieldDimensions.randomPoseOnField());
      pose.translation.x = (pose.translation.x + theFieldDimensions.xPosOwnGroundline) / 2;
      sample.translation = Vector2<int>(int(pose.translation.x), int(pose.translation.y));
      sample.angle = pose.rotation;
      sample.rotation = Vector2<int>(int(cos(pose.rotation) * 1024), int(sin(pose.rotation) * 1024));
    }
  }
  lastOdometry = theOdometryData;
  poseCalculator->init();
  sampleTemplateGenerator.init();
  poseSetTime = theFrameInfo.time;
}

void SelfLocator::update(PotentialRobotPose& robotPose)
{
  if(theMotionInfo.motion == MotionRequest::specialAction
     && theMotionInfo.specialActionRequest.specialAction == SpecialActionRequest::sitDownKeeper)
    return;
  MODIFY("module:SelfLocator:parameters", parameters);

  // recreate sampleset if number of samples was changed
  if(parameters.numberOfSamples != samples->size())
    init();

  // Normal computation:
  preExecution(robotPose);
  bool templatesOnly(false);
  bool odometryOnly(false);
  DEBUG_RESPONSE("module:SelfLocator:templates only", templatesOnly = true;);
  DEBUG_RESPONSE("module:SelfLocator:odometry only", odometryOnly = true;);

  if(templatesOnly) //debug
  {
    if(sampleTemplateGenerator.templatesAvailable())
      for(int i = 0; i < this->samples->size(); ++i)
        generateTemplate(samples->at(i));
    poseCalculator->calcPose(robotPose);
  }
  else if(odometryOnly) //debug
  {
    robotPose += theOdometryData - lastOdometry;
    lastOdometry = theOdometryData;
  }
  else //normal case
  {
    motionUpdate(updatedBySensors);
    updatedBySensors = applySensorModels();
    // Decrease weighting of samples that are probably in the wrong half (according to the OwnSideModel)
    for(int i = 0; i < samples->size(); ++i)
    {
      SelfLocatorSample& s = samples->at(i);
      if(s.translation.x > theOwnSideModel.largestXPossible)
      {
        s.weighting *= parameters.tooLargeXFactor;
        updatedBySensors = true;
      }
    }
    if(updatedBySensors)
    {
      // Adapt weighting for Augmented MCL
      adaptWeightings();
      // Perform resampling (including generation of new samples)
      resampling();
    }
    int lastIndexOfBestCluster = poseCalculator->getIndexOfBestCluster();
    poseCalculator->calcPose(robotPose);
    if(lastIndexOfBestCluster != poseCalculator->getIndexOfBestCluster())
      poseSetTime = theFrameInfo.time;
  }
  robotPose.deviation = 100000.f;

  bool mirrored = gameStateHandler.shouldMirror(robotPose);
  if(mirrored)
  {
    (Pose2D&) robotPose = Pose2D(pi) + robotPose;

    if(samples->size() > 0)
    {
      int indexOfBestCluster = poseCalculator->getIndexOfBestCluster();
      for(Sample* s = &samples->at(0), * end = s + samples->size(); s < end; ++s)
        if(s->cluster == indexOfBestCluster || gameStateHandler.shouldMirror(s->toPose()))
        {
          s->translation = -s->translation;
          s->angle = normalize(s->angle + pi);
          s->rotation = -s->rotation;
        }
    }

    poseSetTime = theFrameInfo.time;
  }
  robotPose.poseSetTime = poseSetTime;
  
  gameStateHandler.postProcess(mirrored || redistributed);

  // drawings
  DECLARE_DEBUG_DRAWING("module:SelfLocator:poseCalculator", "drawingOnField"); // Draws the internal representations for pose computation
  COMPLEX_DRAWING("module:SelfLocator:poseCalculator", poseCalculator->draw());

  DECLARE_DEBUG_DRAWING("module:SelfLocator:perceptValidityChecker", "drawingOnField"); // Draws the internal representations for percept validity checking
  COMPLEX_DRAWING("module:SelfLocator:perceptValidityChecker", perceptValidityChecker->draw());

  DECLARE_DEBUG_DRAWING("module:SelfLocator:samples", "drawingOnField"); // Draws the internal representations of the goal locator
  COMPLEX_DRAWING("module:SelfLocator:samples", drawSamples());

  DECLARE_DEBUG_DRAWING("module:SelfLocator:templates", "drawingOnField"); // Draws all available templates
  COMPLEX_DRAWING("module:SelfLocator:templates", sampleTemplateGenerator.draw(););

  DECLARE_DEBUG_DRAWING("module:SelfLocator:selectedPoints", "drawingOnField");

  EXECUTE_ONLY_IN_DEBUG(fieldModel.draw(););
}

void SelfLocator::update(RobotPose& robotPose)
{
  robotPose = (const RobotPose&)thePotentialRobotPose;
  EXECUTE_ONLY_IN_DEBUG(robotPose.draw(theOwnTeamInfo.teamColor != TEAM_BLUE););
}

void SelfLocator::preExecution(RobotPose& robotPose)
{
  gameStateHandler.preProcess();
  sampleTemplateGenerator.bufferNewPerceptions();

  // Reset all weightings to 1.0f
  for(int i = 0; i < samples->size(); ++i)
    samples->at(i).weighting = 1.0f;

  DEBUG_RESPONSE("module:SelfLocator:createFieldModel", fieldModel.create(););

  // Table for checking point / line validity:
  DEBUG_RESPONSE("module:SelfLocator:PerceptValidityChecker:saveGoalNetTable", perceptValidityChecker->saveGoalNetTable(););
  DEBUG_RESPONSE("module:SelfLocator:PerceptValidityChecker:recomputeGoalNetTable", perceptValidityChecker->computeGoalNetTable(););

  // change module for pose calculation or use debug request to reload the pose calculator
  PoseCalculatorType newPoseCalculatorType(poseCalculatorType);
  MODIFY_ENUM("module:SelfLocator:poseCalculatorType", newPoseCalculatorType);

  bool reloadPoseCalculator = (newPoseCalculatorType != poseCalculatorType);
  DEBUG_RESPONSE("module:SelfLocator:reloadPoseCalculator", reloadPoseCalculator = true;);

  if(reloadPoseCalculator)
  {
    delete poseCalculator;
    switch(newPoseCalculatorType)
    {
    case POSE_CALCULATOR_2D_BINNING:
      poseCalculator = new PoseCalculator2DBinning< Sample, SampleSet<Sample>, 10 >
      (*samples, theFieldDimensions);
      break;
    case POSE_CALCULATOR_PARTICLE_HISTORY:
      poseCalculator = new PoseCalculatorParticleHistory< Sample, SampleSet<Sample> >(*samples);
      poseCalculator->init();
      break;
    case POSE_CALCULATOR_BEST_PARTICLE:
      poseCalculator = new PoseCalculatorBestParticle< Sample, SampleSet<Sample> >(*samples);
      break;
    case POSE_CALCULATOR_OVERALL_AVERAGE:
      poseCalculator = new PoseCalculatorOverallAverage< Sample, SampleSet<Sample> >(*samples);
      break;
    case POSE_CALCULATOR_K_MEANS_CLUSTERING:
      poseCalculator = new PoseCalculatorKMeansClustering< Sample, SampleSet<Sample>, 5, 1000 >
      (*samples, theFieldDimensions);
      break;
    default:
      ASSERT(false);
    }
    poseCalculatorType = newPoseCalculatorType;
  }

  std::vector<Pose2D> poses;
  std::vector<Pose2D> standardDeviations;
  redistributed = gameStateHandler.shouldRedistribute(poses, standardDeviations);
  if(redistributed)
    initSamplesAtGivenPositions(poses, standardDeviations);

  // Initialize sample set again:
  DEBUG_RESPONSE("module:SelfLocator:resetSamples", init(););
}

void SelfLocator::motionUpdate(bool noise)
{
  Pose2D odometryOffset = theOdometryData - lastOdometry;

  const float transNoise = noise ? parameters.translationNoise : 0;
  const float rotNoise = noise ? parameters.rotationNoise : 0;
  const int transX = (int) odometryOffset.translation.x;
  const int transY = (int) odometryOffset.translation.y;
  const float dist = odometryOffset.translation.abs();
  const float angle = abs(odometryOffset.rotation);

  lastOdometry += Pose2D(odometryOffset.rotation, (float) transX, (float) transY);

  // precalculate rotational error that has to be adapted to all samples
  const float rotError = max(rotNoise,
                             max(dist * parameters.movedDistWeight,
                                 angle * parameters.movedAngleWeight));

  // precalculate translational error that has to be adapted to all samples
  const int transXError = (int) max(transNoise,
                                    (float) max(abs(transX * parameters.majorDirTransWeight),
                                        abs(transY * parameters.minorDirTransWeight)));
  const int transYError = (int) max(transNoise,
                                    (float) max(abs(transY * parameters.majorDirTransWeight),
                                        abs(transX * parameters.minorDirTransWeight)));
  for(int i = 0; i < samples->size(); i++)
  {
    Sample& s(samples->at(i));

    // the translational error vector
    const Vector2<int> transOffset((((transX - transXError) << 10) + (transXError << 1) * ((rand() & 0x3ff) + 1) + 512) >> 10,
                                   (((transY - transYError) << 10) + (transYError << 1) * ((rand() & 0x3ff) + 1) + 512) >> 10);

    // update the sample
    s.translation = Vector2<int>(((s.translation.x << 10) + transOffset.x * s.rotation.x - transOffset.y * s.rotation.y + 512) >> 10,
                                 ((s.translation.y << 10) + transOffset.x * s.rotation.y + transOffset.y * s.rotation.x + 512) >> 10);
    s.angle += odometryOffset.rotation + (randomFloat() * 2 - 1) * rotError;
    s.angle = normalize(s.angle);
    s.rotation = Vector2<int>(int(cos(s.angle) * 1024), int(sin(s.angle) * 1024));

    // clip to field boundary
    fieldBoundary.clip(s.translation);
  }
}

bool SelfLocator::applySensorModels()
{
  if(!theCameraMatrix.isValid)
    return false;

  // collect all observations
  // goals and the center circle will always be selected
  selectedObservations.clear();
  for(unsigned int i = 0, count = theGoalPercept.goalPosts.size(); i < count; ++i)
  {
    const GoalPost& post = theGoalPercept.goalPosts[i];
    if(post.position != GoalPost::IS_UNKNOWN)
      selectedObservations.push_back(SensorModel::Observation(SensorModel::Observation::GOAL_POST, i));
  }
  if(selectedObservations.empty()) // no known posts
    for(unsigned int i = 0, count = theGoalPercept.goalPosts.size(); i < count; ++i)
    {
      const GoalPost& post = theGoalPercept.goalPosts[i];
      if(post.position == GoalPost::IS_UNKNOWN)
        selectedObservations.push_back(SensorModel::Observation(SensorModel::Observation::GOAL_POST, i));
    }
  if(theLinePercept.circle.found)
    selectedObservations.push_back(SensorModel::Observation(SensorModel::Observation::CENTER_CIRCLE, 0));

  // points and corners are optional
  observations.clear();
  lines.clear();
  int j = 0;
  for(list<LinePercept::Line>::const_iterator i = theLinePercept.lines.begin();
      i != theLinePercept.lines.end(); ++i)
  {
    lines.push_back(&*i);
    observations.push_back(SensorModel::Observation(SensorModel::Observation::POINT, j++));
    observations.push_back(SensorModel::Observation(SensorModel::Observation::POINT, j++));
  }
  for(int i = 0; i < (int) theLinePercept.intersections.size(); ++i)
    observations.push_back(SensorModel::Observation(SensorModel::Observation::CORNER, i));

  if(selectedObservations.empty() && observations.empty())
    return false;

  // select observations
  while((int) selectedObservations.size() < parameters.numberOfObservations)
    if(observations.empty())
      selectedObservations.push_back(selectedObservations[rand() % selectedObservations.size()]);
    else
      selectedObservations.push_back(observations[rand() % observations.size()]);

  // apply sensor models
  bool sensorModelApplied(false);
  vector<SensorModel*>::iterator sensorModel = sensorModels.begin();
  for(; sensorModel != sensorModels.end(); ++sensorModel)
  {
    selectedIndices.clear();
    for(vector<SensorModel::Observation>::const_iterator i = selectedObservations.begin(); i != selectedObservations.end(); ++i)
      if(i->type == (*sensorModel)->type)
        selectedIndices.push_back(i->index);

    SensorModel::SensorModelResult result;
    if(selectedIndices.empty())
      result = SensorModel::NO_SENSOR_UPDATE;
    else
      result = (*sensorModel)->computeWeightings(*samples, selectedIndices, sensorModelWeightings);
    if(result == SensorModel::FULL_SENSOR_UPDATE)
    {
      for(int i = 0; i < samples->size(); ++i)
      {
        samples->at(i).weighting *= sensorModelWeightings[static_cast<unsigned int>(i)];
        sensorModelApplied = true;
      }
    }
    else if(result == SensorModel::PARTIAL_SENSOR_UPDATE)
    {
      // Compute average of all valid weightings, use this average for all invalid ones
      float sum(0.0f);
      int numOfValidSamples(0);
      for(unsigned int i = 0; i < sensorModelWeightings.size(); ++i)
      {
        if(sensorModelWeightings[i] != -1)
        {
          sum += sensorModelWeightings[i];
          ++numOfValidSamples;
        }
      }
      if(numOfValidSamples == 0) //Should not happen, but you never know...
      {
        continue;
      }
      const float averageWeighting(sum / numOfValidSamples);
      for(unsigned int i = 0; i < sensorModelWeightings.size(); ++i)
      {
        if(sensorModelWeightings[i] != -1)
          samples->at(i).weighting *= sensorModelWeightings[i];
        else
          samples->at(i).weighting *= averageWeighting;
      }
      sensorModelApplied = true;
    }
  }
  return sensorModelApplied;
}

void SelfLocator::resampling()
{
  // swap sample arrays
  Sample* oldSet = samples->swap();
  const int numberOfSamples(samples->size());
  const float weightingsSum(totalWeighting);
  const float resamplingPercentage(sampleTemplateGenerator.templatesAvailable() ? max(0.0f, 1.0f - fastWeighting / slowWeighting) : 0.0f);
  const float numberOfResampledSamples = parameters.disableSensorResetting ? numberOfSamples : numberOfSamples * (1.0f - resamplingPercentage);
  const float threshold = parameters.resamplingThreshold * weightingsSum / numberOfSamples;
  const float weightingBetweenTwoDrawnSamples((weightingsSum + threshold * numberOfSamples) / numberOfResampledSamples);
  float nextPos(randomFloat() * weightingBetweenTwoDrawnSamples);
  float currentSum(0);

  // resample:
  int j(0);
  for(int i = 0; i < numberOfSamples; ++i)
  {
    currentSum += oldSet[i].weighting + threshold;
    while(currentSum > nextPos && j < numberOfSamples)
    {
      samples->at(j++) = oldSet[i];
      nextPos += weightingBetweenTwoDrawnSamples;
    }
  }

  // fill up remaining samples with new poses:
  if(sampleTemplateGenerator.templatesAvailable())
    for(; j < numberOfSamples; ++j)
      generateTemplate(samples->at(j));
  else if(j) // in rare cases, a sample is missing, so add one (or more...)
    for(; j < numberOfSamples; ++j)
      samples->at(j) = samples->at(rand() % j);
  else // resampling was not possible (for unknown reasons), so create a new sample set (fail safe)
#ifdef NDEBUG
  {
    for(int i = 0; i < numberOfSamples; ++i)
    {
      Sample& sample = samples->at(i);
      Pose2D pose(theFieldDimensions.randomPoseOnField());
      sample.translation = Vector2<int>(int(pose.translation.x), int(pose.translation.y));
      sample.rotation = Vector2<int>(int(cos(pose.rotation) * 1024), int(sin(pose.rotation) * 1024));
    }
    poseCalculator->init();
  }
#else
    ASSERT(false);
#endif
}

void SelfLocator::adaptWeightings()
{
  totalWeighting = 0;
  int numberOfSamples(samples->size());
  for(int i = 0; i < numberOfSamples; ++i)
    totalWeighting += samples->at(i).weighting;
  if(totalWeighting == 0)
  {
    OUTPUT(idText, text, "Meeeeeeek. Total weighting of all samples in SelfLocator is 0.");
    return;
  }
  const float averageWeighting = totalWeighting / numberOfSamples;
  if(slowWeighting)
  {
    slowWeighting = slowWeighting + parameters.alphaSlow * (averageWeighting - slowWeighting);
    fastWeighting = fastWeighting + parameters.alphaFast * (averageWeighting - fastWeighting);
  }
  else
    slowWeighting = averageWeighting;
  PLOT("module:SelfLocator:averageWeighting", averageWeighting * 1e3);
  PLOT("module:SelfLocator:slowWeighting", slowWeighting * 1e3);
  PLOT("module:SelfLocator:fastWeighting", fastWeighting * 1e3);
}

void SelfLocator::generateTemplate(Sample& sample)
{
  if(sampleTemplateGenerator.templatesAvailable())
  {
    SampleTemplate st = sampleTemplateGenerator.getNewTemplate();
    if(gameStateHandler.shouldMirror(st))
    {
      st.translation = -st.translation;
      st.rotation = normalize(st.rotation + pi);
    }
    sample.translation = Vector2<int>(int(st.translation.x), int(st.translation.y));
    sample.rotation = Vector2<int>(int(cos(st.rotation) * (1 << 10)),
                                   int(sin(st.rotation) * (1 << 10)));
    sample.angle = st.rotation;
    sample.weighting = 0;
    sample.cluster = poseCalculator->getNewClusterIndex();
  }
}

void SelfLocator::drawSamples()
{
  const int numberOfSamples(samples->size());
  const float maxWeighting = 2 * totalWeighting / numberOfSamples;
  for(int i = 0; i < numberOfSamples; ++i)
  {
    const Sample& s(samples->at(i));
    Pose2D pose(s.angle, (float) s.translation.x, (float) s.translation.y);
    unsigned char weighting = (unsigned char)(s.weighting / maxWeighting * 255);
    Vector2<> headPos(30, 0);
    headPos = pose * headPos;
    ColorRGBA color = s.weighting ? ColorRGBA(weighting, weighting, weighting) : ColorRGBA(255, 0, 0);
    if(s.cluster == poseCalculator->getIndexOfBestCluster())
      color = ColorRGBA(200, 0, 200);
    RECTANGLE2("module:SelfLocator:samples", Vector2<>(pose.translation - Vector2<>(55.f, 90.f)), 110, 180, pose.rotation, 0, Drawings::ps_solid,
            ColorRGBA(180, 180, 180),
            Drawings::bs_solid,
            color);
    CIRCLE("module:SelfLocator:samples",
           headPos.x,
           headPos.y,
           30,
           0, // pen width
           Drawings::ps_solid,
           ColorRGBA(180, 180, 180),
           Drawings::bs_solid,
           ColorRGBA(180, 180, 180));
  }
}

MAKE_MODULE(SelfLocator, Modeling)
