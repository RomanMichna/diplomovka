/**
 * @file SideConfidenceProvider.h
 * Calculates the SideConfidence 
 * @author Michel Bartsch, Thomas Muender
 * @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
 */

#pragma once

#include "Tools/RingBuffer.h"
#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/OwnSideModel.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Modeling/ArmContactModel.h"
#include "Representations/Modeling/SideConfidence.h"
#include "Representations/Infrastructure/TeamMateData.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/CombinedWorldModel.h"
#include "Representations/BehaviorControl/BehaviorControlOutput.h"
#include "Representations/Modeling/Odometer.h"


MODULE(SideConfidenceProvider)
  USES(RobotPose)
  USES(PotentialRobotPose)
  USES(CombinedWorldModel)
  USES(BehaviorControlOutput)
  REQUIRES(OwnSideModel)
  REQUIRES(TeamMateData)
  REQUIRES(BallModel)
  REQUIRES(GroundContactState)
  REQUIRES(FieldDimensions)
  REQUIRES(CameraMatrix)
  REQUIRES(FallDownState)
  REQUIRES(ArmContactModel)
  REQUIRES(FrameInfo)
  REQUIRES(GameInfo)
  REQUIRES(RobotInfo)
  REQUIRES(Odometer)
  PROVIDES_WITH_MODIFY_AND_DRAW(SideConfidence)
END_MODULE


class SideConfidenceProvider : public SideConfidenceProviderBase
{
public:
  /** Constructor */
  SideConfidenceProvider();

  /** Initialization, called before first frame */
  void init();
  
  bool lost; /** sideConfidence 0% and lost-sound played */
  
  /**
    * Provides the sideConfidence
    */
  void update(SideConfidence& sideConfidence);

private:
  /**
  * @class Parameters
  * The parameters of the module
  */
  class Parameters: public Streamable
  {
  private:
    /**
    * The method makes the object streamable.
    * @param in The stream from which the object is read
    * @param out The stream to which the object is written
    */
    virtual void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN;
        STREAM(standardDeviationBallAngle);
        STREAM(standardDeviationBallDistance);
        STREAM(weightingFactor);
        STREAM(sideConfidenceConfident);
        STREAM(sideConfidenceAlmostConfident);
        STREAM(sideConfidenceConfused);
        STREAM(maxBallVelocity);
        STREAM(timeInPenaltyArea);
        STREAM(ballBufferingInterval);
      STREAM_REGISTER_FINISH;
    }

  public:
    float standardDeviationBallAngle;    /** As the name says... */
    float standardDeviationBallDistance; /** As the name says... */
    float weightingFactor;               /** Multiplier for defining the minimum difference between normal and mirrored pose */
    float sideConfidenceConfident;       /** Value for side confidence when being definitely in own half */
    float sideConfidenceAlmostConfident; /** Value for side confidence when being sure about own position but not definitely in own half */
    float sideConfidenceConfused;        /** Value for side confidence when bad things have happened */
    float maxBallVelocity;               /** Maximum velocity of balls that are considered for side confidence computation */
    int timeInPenaltyArea;               /** Time till the robot should be removed from own penalty area */
    int ballBufferingInterval;           /** Time for keeping a local ball observation in buffer */
    float halfPercentage; /** value for resetting the percentage, normally to 50%*/
    float lowLocalizationValidityModificator;  /** how much the SideConfidence is influsenced by fall down */
    float armContactModificator; /** how much the SideConfidence is influsenced by  having arm contact */
    float relativeBallDropin; /** the value how far the ball is dorpped in behind the robot after kicking out*/
    int walkdistanceTillDrop; /** the distance the robot must walk till the sideconfidence will drop */
  

    /** Constructor */
    Parameters(): standardDeviationBallAngle(0.2f), standardDeviationBallDistance(0.4f), 
      weightingFactor(2.5f), sideConfidenceConfident(1.0f), sideConfidenceAlmostConfident(0.95f),
        sideConfidenceConfused(0.0f), maxBallVelocity(200.0f), timeInPenaltyArea(5000), halfPercentage(0.5f), 
        lowLocalizationValidityModificator(0.1f), armContactModificator(0.01f), relativeBallDropin(1000.f), walkdistanceTillDrop(500)
    {}
  };

  
  Parameters parameters;              /**< The parameters of this module */
  unsigned lastFrameTime;             /**< Time of last execution */
  enum {BUFFER_SIZE = 60};            /**< Number of ball state observations */
  ENUM(BallModelSideConfidence,
    OK,
    MIRROR,
    UNKNOWN
  );                                  /**< Discrete states of confidence resulting from comparison of ball models (own vs. others) */
  RingBuffer<BallModelSideConfidence, BUFFER_SIZE> ballConfidences; /**< Buffer of last confidences */
  BallModelSideConfidence averageBallConfidence;                    /**< The average side confidence based on buffered confidences */
  unsigned timeWhenEnteredOwnPenaltyArea; /**< Somebody belonging to the group of people who usually complain about missing comments added this member and did not add any comment. */
  unsigned timeOfLastFall; /** Timestamp to see if the robot has fallen down */
  bool robotHasFallen;     /** Flag to indicate that the robot has fallen down */
  unsigned lastTimeWithoutArmContact;
  float lastDistanceWalkedAtHighValidity;
  Vector2<> lastBallObservation;       /**< Position (relative to the robot) of the last estimated ball that has actually been observed */
  unsigned timeOfLastBallObservation;  /**< Point of time of last observation */
  float maxDistanceToFieldCenterForArmConsideration;      /**< Just as the name says ... */
  float maxDistanceToFieldCenterForFallDownConsideration;      /**< Just as the name says ... */


  /**
    * Checks if the other team mates see the ball near the own estimated position
    * or near it´s mirrored position.
    */
  void updateSideConfidenceFromOthers(SideConfidence& sideConfidence);

  /**
    * Updates the own confidence.
    */
  void updateSideConfidenceFromOwn(SideConfidence& sideConfidence);
  
  /**
   * Checks if mirrored after kicking ball out
   */
  void handleAloneAndConfused(SideConfidence& sideConfidence);
  
  /**
   * Maps sideConfidence to ConfidenceState.
   */
  void updateConfidenceState(SideConfidence& sideConfidence);

  /** Updates ball confidence buffer */
  void updateBallConfidences();

  /** Combines confidence based on current ball models */
  BallModelSideConfidence computeCurrentBallConfidence();

  float computeAngleWeighting(float measuredAngle, const Vector2<>& modelPosition,
    const Pose2D& robotPose, float standardDeviation) const;

  float computeDistanceWeighting(float measuredDistanceAsAngle, const Vector2<>& modelPosition,
    const Pose2D& robotPose, float cameraZ, float standardDeviation) const;
}; 
