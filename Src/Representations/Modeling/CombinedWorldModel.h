/**
 * @file CombinedWorldModel.h
 *
 * Declaration of a representation that represents a combined world model
 *
 * @author Katharina Gillmann
 */

#pragma once

#include <vector>
#include "Representations/Modeling/BallModel.h"
#include "Tools/Math/Matrix2x2.h"
#include "Tools/Math/Pose2D.h"

/**
* @class GaussianDistribution
* a gaussian distribution consisting of a mean and a covariance.
*/
class GaussianDistribution : public Streamable
{
private:
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM_COMPRESSED_POSITION(robotPosition);
    STREAM(covariance.c[0][0]);
    STREAM(covariance.c[0][1]);
    STREAM(covariance.c[1][1]);
    if(in)
    {
      covariance.c[1][0] = covariance.c[0][1]; // covariance is symmetric
    }
    STREAM_REGISTER_FINISH;
  }

public:
  Vector2<> robotPosition; //Position (mean) of the detected robot, mean is the center point in both cases (ultrasonic and vision)
  Matrix2x2<> covariance; // covariance of the mesasurement

  GaussianDistribution() {} // Constructor
  GaussianDistribution(const Vector2<>& robotPosition, const Matrix2x2<>& covariance) :
    robotPosition(robotPosition), covariance(covariance) {}
};

/**
* @class CombinedWorldModel
* a combined world model.
*/
class CombinedWorldModel : public Streamable
{
private:
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(positionsOwnTeam);
    STREAM(positionsOpponentTeam);

    Vector2<> ballStatePosition = ballState.position, ballStateVelocity = ballState.velocity;
    STREAM_COMPRESSED_POSITION(ballStatePosition);
    STREAM_COMPRESSED_POSITION(ballStateVelocity);
    ballState.position = ballStatePosition;
    ballState.velocity = ballStateVelocity;

    STREAM(ballState);
    STREAM(ballIsValid);
    STREAM(ballStateOthers);
    STREAM(ballIsValidOthers);
    STREAM(ballStateOthersMaxSideConfidence);
    STREAM_COMPRESSED_POSITION(expectedEndPosition);
    STREAM_REGISTER_FINISH;
  }

public:

  std::vector<Pose2D> positionsOwnTeam;                     /**< poses of own robots */
  std::vector<GaussianDistribution > positionsOpponentTeam; /**< positions of opponent robots */
  BallState ballState;                                      /**< position and velocity of the ball. */
  bool ballIsValid;                                         /**< ball state is valid, if true */

  BallState ballStateOthers;                                /**< position and velocity of the ball as seen by teammates only */
  bool ballIsValidOthers;                                   /**< if calculated ball state is valid as seen by teammates only */
  float ballStateOthersMaxSideConfidence;                   /**< Maximum SideConfidence of the involved robots */

  Vector2<> expectedEndPosition;                            /**< expected end position of the ball. */

  void draw()const;
};
