/**
* @file RobotPoseValidator.h
* Declares a class that validates poses from the self locator on a field
* with yellow goals.
* @author Colin Graf
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Perception/GoalPercept.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/GroundContactState.h"

MODULE(RobotPoseValidator)
  REQUIRES(PotentialRobotPose)
  REQUIRES(FilteredRobotPose)
  REQUIRES(FieldDimensions)
  REQUIRES(OwnTeamInfo)
  REQUIRES(GoalPercept)
  REQUIRES(GroundContactState)
  REQUIRES(FallDownState)
  PROVIDES_WITH_MODIFY_AND_OUTPUT(RobotPose)
END_MODULE

/**
* @class RobotPoseValidator
* A modules that determines the validity of a robot pose.
*/
class RobotPoseValidator : public RobotPoseValidatorBase
{
public:
  /**
  * Default constructor.
  */
  RobotPoseValidator();

private:
  ENUM(Membership, OWN_TEAM, OPPONENT_TEAM);
  Membership membership;

  /**
  * A collection of parameters for the robot pose validator.
  */
  class Parameters : public Streamable
  {
  public:
    /**
    * Default constructor.
    */
    Parameters() {}

    Pose2D validationMaxDeviation; /**< The maximale admissible deviation of the robot pose. (Used for detecing valid poses.) */
    float validationMaxGoalPerceptDistance; /**< The maximale admissible distance of goal percepts to their expected position. (Used for detecing valid poses; In percentage of the distance to the goal post). */
    unsigned int validationMinGoalSightings; /**< The minimal required goal sightings used for detecting valid poses. */
    unsigned int validationMinUnknownGoalSightings; /**< The minimal required unknown goal sightings used for detecting valid poses. */

  private:
    /**
    * The method makes the object streamable.
    * @param in The stream from which the object is read.
    * @param out The stream to which the object is written.
    */
    virtual void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN;
      STREAM(validationMaxDeviation);
      STREAM(validationMaxGoalPerceptDistance);
      STREAM(validationMinGoalSightings);
      STREAM(validationMinUnknownGoalSightings);
      STREAM_REGISTER_FINISH;
    }
  };

  Parameters p; /**< A set of parameters for the module. */

  Vector2<> goalPosts[numOfMemberships][GoalPost::numOfPositions]; /**< The positions of the goal posts. */

  bool validated; /**< Whether the pose is validated or not. */
  unsigned validGoalSightingSinceLastReset; /**< Amount of matching goal post sightings since last loss of validity. */
  unsigned validUnknownGoalSightingSinceLastReset; /**< Amout of "unknown" matching goal post sightings since last loss of validity. */

  unsigned lastFilterResetTime; /**< Time stamp from the last filter reset */

  /**
  * Initializes the module.
  */
  void init();

  /**
  * Updates the validated robot pose provided from this module.
  * @param robotPose The robotPose updated from the module
  */
  void update(RobotPose& robotPose);

};
