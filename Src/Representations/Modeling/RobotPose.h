/**
 * @file RobotPose.h
 *
 * The file contains the definition of the class RobotPose.
 *
 * @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 */

#pragma once

#include "Tools/Math/Pose2D.h"
#include "Tools/Math/Matrix3x3.h"

/**
* @class RobotPose
* The pose of the robot with additional information
*/
class RobotPose : public Pose2D
{
public:
  enum
  {
    unknownDeviation = 100000,
  };

  float validity;                   /**< The validity of the robot pose. (0 = invalid, 1 = perfect) */
  float deviation;                  /**< The deviation of the robot pose. */

  /** Constructor */
  RobotPose() : validity(0.0f), deviation(unknownDeviation) {}

  /** Assignment operator
  * @param other Another RobotPose
  * @return A reference to the object after the assignment
  */
  const RobotPose& operator=(const RobotPose& other)
  {
    (Pose2D&) *this = (const Pose2D&) other;
    validity = other.validity;
    deviation = other.deviation;
    return *this;
  }

  /** Assignment operator for Pose2D objects
  * @param other A Pose2D object
  * @return A reference to the object after the assignment
  */
  const RobotPose& operator=(const Pose2D& other)
  {
    (Pose2D&) *this = other;
    // validity and co are not set
    return *this;
  }

  /** Draws the robot pose in the color of the team to the field view*/
  void draw(bool teamRed);

protected:
  /**
  * The method makes the object streamable.
  * @param in The stream from which the object is read
  * @param out The stream to which the object is written
  */
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM_BASE(Pose2D);
    STREAM(validity);
    STREAM(deviation);
    STREAM_REGISTER_FINISH;
  }
};

class GroundTruthRobotPose : public RobotPose
{
public:
  unsigned timestamp;

  /** Draws the robot pose to the field view*/
  void draw();

private:
  /**
  * The method makes the object streamable.
  * @param in The stream from which the object is read
  * @param out The stream to which the object is written
  */
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM_BASE(RobotPose);
    STREAM(timestamp);
    STREAM_REGISTER_FINISH;
  }
};

class PotentialRobotPose : public RobotPose
{
public:
  unsigned poseSetTime; /**< time stamp of the last manual pose regulation, pose mirroring or particle cluster changeover */

  /** Default constructor */
  PotentialRobotPose() : poseSetTime(0) {}

private:
  /**
  * The method makes the object streamable.
  * @param in The stream from which the object is read
  * @param out The stream to which the object is written
  */
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM_BASE(RobotPose);
    STREAM(poseSetTime);
    STREAM_REGISTER_FINISH;
  }
};

class FilteredRobotPose : public Pose2D
{
public:
  Matrix3x3<> cov; /**< The covariance matrix of the estimated robot pose. */
  unsigned filterResetTime; /**< Time stamp from the last filter reset */

private:
  /**
  * The method makes the object streamable.
  * @param in The stream from which the object is read
  * @param out The stream to which the object is written
  */
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM_BASE(Pose2D);
    STREAM(cov);
    STREAM(filterResetTime);
    STREAM_REGISTER_FINISH;
  }
};

/**
* @class RobotPoseCompressed
* A compressed version of RobotPose used in team communication
*/
class RobotPoseCompressed : public RobotPose
{
public:

  /** Default constructor */
  RobotPoseCompressed() {}

  /** A copy constructor */
  RobotPoseCompressed(const RobotPose& robotPose) {(RobotPose&) *this = robotPose;}

  const RobotPose& unpack() const {return *this;}

private:
  /**
  * The method makes the object streamable.
  * @param in The stream from which the object is read
  * @param out The stream to which the object is written
  */
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM_COMPRESSED_POSITION(translation);
    STREAM_COMPRESSED_ANGLE(rotation);
    STREAM_COMPRESSED_NORMALIZED_FLOAT(validity); // normalized means it is in [0:1]
    STREAM(deviation);
    STREAM_REGISTER_FINISH;
  }
};
