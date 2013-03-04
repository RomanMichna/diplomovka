/**
 * @file SideConfidenceProvider.h
 * Calculates the SideConfidence 
 * @author Michel Bartsch, Thomas Muender, Marcel Steinbeck
 * @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
 */

#include "SideConfidenceProvider.h"
#include "Platform/SoundPlayer.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Probabilistics.h"
#include "Tools/Settings.h"
#include "Tools/Streams/InStreams.h"

MAKE_MODULE(SideConfidenceProvider, Modeling)


SideConfidenceProvider::SideConfidenceProvider() :
lost(false), lastFrameTime(0), timeWhenEnteredOwnPenaltyArea(0), timeOfLastFall(0), robotHasFallen(false), lastTimeWithoutArmContact(0), lastDistanceWalkedAtHighValidity(0), timeOfLastBallObservation(0)
{
  InConfigMap stream(Global::getSettings().expandLocationFilename("sideConfidence.cfg"));
  if(stream.exists())
    stream >> parameters;
}


void SideConfidenceProvider::init()
{
   const Vector2<> maxPos(static_cast<float>(theFieldDimensions.xPosOpponentFieldBorder), static_cast<float>(theFieldDimensions.yPosLeftFieldBorder));
   maxDistanceToFieldCenterForArmConsideration = maxPos.abs() / 2.f;
   maxDistanceToFieldCenterForFallDownConsideration = maxPos.abs() / 3.f;
}


void SideConfidenceProvider::update(SideConfidence& sideConfidence)
{
  MODIFY("parameters:SideConfidenceProvider", parameters);
  updateBallConfidences();
  updateSideConfidenceFromOthers(sideConfidence);
  updateSideConfidenceFromOwn(sideConfidence);
  handleAloneAndConfused(sideConfidence);
  updateConfidenceState(sideConfidence);
  
  // Setting some timings
  lastFrameTime = theFrameInfo.time;
  // Setting the time when not in own penalty area
  if(!(theRobotPose.translation.x < theFieldDimensions.xPosOwnPenaltyArea && abs(theRobotPose.translation.y) < theFieldDimensions.yPosLeftPenaltyArea))
    timeWhenEnteredOwnPenaltyArea = theFrameInfo.time;
  // Setting the distance walked when not having a bad validity
  if(theRobotPose.validity > parameters.halfPercentage)
    lastDistanceWalkedAtHighValidity = theOdometer.distanceWalked;
  // Setting the time when the robot has a good stand 
  if((theFallDownState.state != theFallDownState.upright && 
     theFallDownState.state != theFallDownState.undefined && 
     theFallDownState.state != theFallDownState.staggering) && (theFrameInfo.getTimeSince(timeOfLastFall) > 8000)
     && !robotHasFallen)
  {
    timeOfLastFall = theFrameInfo.time;
    robotHasFallen = true;
  }
  // Setting the time when the robot not has arm contact
  if(!theArmContactModel.contactLeft && !theArmContactModel.contactRight)
    lastTimeWithoutArmContact = theFrameInfo.time;
}


void SideConfidenceProvider::updateSideConfidenceFromOthers(SideConfidence& sideConfidence)
{
  const float& best = theCombinedWorldModel.ballStateOthersMaxSideConfidence;
  // The best other sideConfidence must be better than the own.. and it cannot be
  // the robot itself.
  if(best > sideConfidence.sideConfidence)
  {
    if(averageBallConfidence == MIRROR)
    {
      sideConfidence.mirror = true;
      return;
      //confidence value does not change yet!
    }
    else if(averageBallConfidence == OK)
    {
      // I can trust my teammates and can update my confidence
      sideConfidence.sideConfidence = best;
    }
  }
  sideConfidence.mirror = false;
}


void SideConfidenceProvider::updateSideConfidenceFromOwn(SideConfidence& sideConfidence)
{
  // Not playing -> sideConfidence 100%
  if(theGameInfo.state != STATE_PLAYING || theRobotInfo.penalty != PENALTY_NONE)
  {
    lost = false;
    sideConfidence.sideConfidence = parameters.sideConfidenceConfident;
    return;
  }
  // Playing but on the safe side -> sideConfidence 100%
  if(theOwnSideModel.stillInOwnSide)
  {
    sideConfidence.sideConfidence = parameters.sideConfidenceConfident;
    return;
  }
  // Leaving the safe side -> sideConfidence 95% and may get worse
  else if(sideConfidence.sideConfidence == parameters.sideConfidenceConfident)
  {
    sideConfidence.sideConfidence = parameters.sideConfidenceAlmostConfident;
  }
  
  // long time nothing seen to validate the robot's position, or the seen is not as expected.
  // for each 15cm walked on low valitity the sideconfidence will drop by 10 percent
  if(theOdometer.getDistanceFrom(lastDistanceWalkedAtHighValidity) > parameters.walkdistanceTillDrop)
  {
    sideConfidence.sideConfidence -= parameters.lowLocalizationValidityModificator;
    lastDistanceWalkedAtHighValidity = theOdometer.distanceWalked;
  }
  
  //robot has fallen down
  if(robotHasFallen)
  {
    const float distToFieldCenter(theRobotPose.translation.abs());
    if(distToFieldCenter < maxDistanceToFieldCenterForFallDownConsideration)
    {
      float currentFallDownModificator(1.0);
      const float rCircle = static_cast<float>(theFieldDimensions.centerCircleRadius);
      if(distToFieldCenter > rCircle)
      {
        currentFallDownModificator *= 1 - ((distToFieldCenter - rCircle) / (maxDistanceToFieldCenterForFallDownConsideration - rCircle));
      }
      sideConfidence.sideConfidence -= currentFallDownModificator;
    }
    robotHasFallen = false;
  }
  
  //arm contact, another robot may change my direction
  if(theFrameInfo.getTimeSince(lastTimeWithoutArmContact) > 30)
  {
    const float distToFieldCenter(theRobotPose.translation.abs());
    if(distToFieldCenter < maxDistanceToFieldCenterForArmConsideration)
    {
      float currentArmContactModificator(parameters.armContactModificator);
      const float rCircle = static_cast<float>(theFieldDimensions.centerCircleRadius);
      if(distToFieldCenter > rCircle)
      {
        currentArmContactModificator *= 1 - ((distToFieldCenter - rCircle) / (maxDistanceToFieldCenterForArmConsideration - rCircle));
      }
      sideConfidence.sideConfidence -= currentArmContactModificator;
    }
    lastTimeWithoutArmContact = theFrameInfo.time;
  }
  
  //normalize 
  if(sideConfidence.sideConfidence < parameters.sideConfidenceConfused)
    sideConfidence.sideConfidence = parameters.sideConfidenceConfused;

  //debug sound
  if(!lost && (sideConfidence.sideConfidence == parameters.sideConfidenceConfused))
  {
    lost = true;
    SoundPlayer::play("lost.wav");
  }
  else if(lost && (sideConfidence.sideConfidence > parameters.sideConfidenceConfused))
  {
    lost = false;
    SoundPlayer::play("allright.wav");
  }
}


void SideConfidenceProvider::handleAloneAndConfused(SideConfidence& sideConfidence)
{
  if(theTeamMateData.numOfConnectedPlayers == 0 && theBehaviorControlOutput.behaviorData.aloneAndConfused)
  {
    float expectedX = std::max((float)theFieldDimensions.xPosDropInLineOwnHalf, theRobotPose.translation.x - parameters.relativeBallDropin);
    float expectedXIfMirrored = std::min((float)theFieldDimensions.xPosDropInLineOpponentHalf, theRobotPose.translation.x + parameters.relativeBallDropin);
    float realX = (theRobotPose * theBallModel.estimate.position).x;
    bool mirrored = realX - expectedX > parameters.relativeBallDropin * 1.5f|| 
                    theFrameInfo.getTimeSince(timeWhenEnteredOwnPenaltyArea) > parameters.timeInPenaltyArea;
    bool normal = expectedXIfMirrored - realX > parameters.relativeBallDropin * 1.5f;
    bool confident = normal || mirrored;

    sideConfidence.mirror = mirrored;
    if(confident)
    {
      sideConfidence.sideConfidence = parameters.halfPercentage;
    }
  }
}


void SideConfidenceProvider::updateConfidenceState(SideConfidence& sideConfidence)
{
  if(sideConfidence.sideConfidence == parameters.sideConfidenceConfident)
    sideConfidence.confidenceState = SideConfidence::CONFIDENT;
  else if(sideConfidence.sideConfidence == parameters.sideConfidenceAlmostConfident)
    sideConfidence.confidenceState = SideConfidence::ALMOST_CONFIDENT;
  else if(sideConfidence.sideConfidence > parameters.sideConfidenceConfused)
    sideConfidence.confidenceState = SideConfidence::UNSURE;
  else
    sideConfidence.confidenceState = SideConfidence::CONFUSED;
}


void SideConfidenceProvider::updateBallConfidences()
{
  // Check, if a mirrorcle has occured:
  if(thePotentialRobotPose.poseSetTime == lastFrameTime)
  {
    ballConfidences.init();
    averageBallConfidence = UNKNOWN;
  }
  // Special handling for certain game states:
  if(theGameInfo.state != STATE_PLAYING || theRobotInfo.penalty != PENALTY_NONE)
  {
    ballConfidences.init();
    averageBallConfidence = UNKNOWN;
  }
  // Save current local ball observation:
  if((theBallModel.timeWhenLastSeen == theFrameInfo.time) &&
     (theFieldDimensions.isInsideField(theRobotPose * theBallModel.estimate.position)) &&
     (theBallModel.estimate.velocity.abs() <= parameters.maxBallVelocity))
  {
    lastBallObservation = theBallModel.estimate.position;
    timeOfLastBallObservation = theFrameInfo.time;  
  }
  // Add current confidence to buffer:
  if((theFrameInfo.getTimeSince(timeOfLastBallObservation) < parameters.ballBufferingInterval) &&
     (theCombinedWorldModel.ballIsValidOthers) &&
     (theCombinedWorldModel.ballStateOthers.velocity.abs() <= parameters.maxBallVelocity))
  {
    ballConfidences.add(computeCurrentBallConfidence());
    timeOfLastBallObservation = 0; //For next confidence computation, a "fresh" observation is needed
  }
  else
  {
    ballConfidences.add(UNKNOWN);
  }
  // Check current buffer content for possible mirror situation
  if(ballConfidences.getNumberOfEntries() != ballConfidences.getMaxEntries())
  {
    return;
  }
  int mirrorCount(0);
  int okCount(0);
  for(int i=0; i<ballConfidences.getNumberOfEntries(); ++i)
  {
    switch(ballConfidences[i])
    {
      case MIRROR: ++mirrorCount; break;
      case OK:     ++okCount; break;
      default:     ; // do nothing
    }
  }
  int unknownCount = ballConfidences.getNumberOfEntries() - mirrorCount - okCount;
  if((okCount == 0) && (3 * mirrorCount > unknownCount))
    averageBallConfidence = MIRROR;
  else if((3 * okCount > unknownCount) && (mirrorCount == 0))
    averageBallConfidence = OK;
  else
    averageBallConfidence = UNKNOWN;
}


SideConfidenceProvider::BallModelSideConfidence SideConfidenceProvider::computeCurrentBallConfidence()
{
  // Some constant parameters
  const float distanceObserved = lastBallObservation.abs();
  const float angleObserved = lastBallObservation.angle();
  const float& camZ = theCameraMatrix.translation.z;
  const float distanceAsAngleObserved = (pi_2 - atan2(camZ,distanceObserved));

  // Weighting for original pose
  float originalWeighting = computeAngleWeighting(angleObserved, theCombinedWorldModel.ballStateOthers.position, theRobotPose, 
    parameters.standardDeviationBallAngle);
  originalWeighting *= computeDistanceWeighting(distanceAsAngleObserved, theCombinedWorldModel.ballStateOthers.position, theRobotPose,
    camZ, parameters.standardDeviationBallDistance);

  // Weighting for mirrored pose
  const Pose2D  mirroredPose = Pose2D(pi) + (Pose2D)(theRobotPose);
  float mirroredWeighting = computeAngleWeighting(angleObserved, theCombinedWorldModel.ballStateOthers.position, mirroredPose, 
    parameters.standardDeviationBallAngle);
  mirroredWeighting *= computeDistanceWeighting(distanceAsAngleObserved, theCombinedWorldModel.ballStateOthers.position, mirroredPose,
    camZ, parameters.standardDeviationBallDistance);

  // Decide state based on weights
  if((mirroredWeighting > parameters.weightingFactor * originalWeighting) || (originalWeighting > parameters.weightingFactor * mirroredWeighting))
    return mirroredWeighting > originalWeighting ? MIRROR : OK;
  return UNKNOWN;
}


float SideConfidenceProvider::computeAngleWeighting(float measuredAngle, const Vector2<>& modelPosition,
  const Pose2D& robotPose, float standardDeviation) const
{
  const float modelAngle = Geometry::angleTo(robotPose, modelPosition);
  return gaussianProbability(abs(modelAngle-measuredAngle), standardDeviation);
}


float SideConfidenceProvider::computeDistanceWeighting(float measuredDistanceAsAngle, const Vector2<>& modelPosition,
  const Pose2D& robotPose, float cameraZ, float standardDeviation) const
{
  const float modelDistance = (robotPose.translation - modelPosition).abs();
  const float modelDistanceAsAngle = (pi_2 - atan2(cameraZ,modelDistance));
  return gaussianProbability(abs(modelDistanceAsAngle-measuredDistanceAsAngle), standardDeviation);
}
