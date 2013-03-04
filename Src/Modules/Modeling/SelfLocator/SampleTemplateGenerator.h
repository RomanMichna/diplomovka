/**
* @file SampleTemplateGenerator.h
*
* This file declares a submodule that generates robot positions from percepts.
*
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#pragma once

#include "SelfLocatorParameters.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Perception/GoalPercept.h"
#include "Representations/Perception/LinePercept.h"
#include "Tools/RingBuffer.h"

/**
* @class SampleTemplate
* Robot position generated from perceptions
*/
class SampleTemplate : public Pose2D
{
public:
  /** Constructor */
  SampleTemplate(): Pose2D(), timestamp(0) {}

  /** Constructor */
  SampleTemplate(const Pose2D& pose): Pose2D(pose), timestamp(0) {}

  /** Timestamp of visual input for construction of this template */
  unsigned timestamp;
};


/**
* @class SampleTemplateGenerator
*
* A module for computing poses from percepts
*/
class SampleTemplateGenerator
{
private:
  const SelfLocatorParameters& parameters;
  const GoalPercept& theGoalPercept;
  const LinePercept& theLinePercept;
  const FrameInfo& theFrameInfo;
  const FieldDimensions& theFieldDimensions;
  const OdometryData& theOdometryData;

  class FullGoal
  {
  public:
    Vector2<> seenLeftPosition;
    Vector2<> realLeftPosition;
    Vector2<> seenRightPosition;
    Vector2<> realRightPosition;
    int timestamp;
    Pose2D odometry;
  };

  class KnownGoalpost
  {
  public:
    Vector2<> seenPosition;
    Vector2<> realPosition;
    int timestamp;
    Pose2D odometry;
    bool centerCircleSeen;
    Vector2<> centerCircleSeenPosition;
  };

  class UnknownGoalpost
  {
  public:
    Vector2<> seenPosition;
    Vector2<> realPositions[2];
    int timestamp;
    Pose2D odometry;
    bool centerCircleSeen;
    Vector2<> centerCircleSeenPosition;
  };

  enum {MAX_PERCEPTS = 10};
  RingBuffer<FullGoal, MAX_PERCEPTS> fullGoals;
  RingBuffer<KnownGoalpost, MAX_PERCEPTS> knownGoalposts;
  RingBuffer<UnknownGoalpost, MAX_PERCEPTS> unknownGoalposts;
  Vector2<> realPostPositions[GoalPost::numOfPositions];

  template<typename T> void removeOldPercepts(RingBuffer<T, MAX_PERCEPTS>& buffer);

  /**
  * Generates a new sample by using the perceptions of both posts of a goal
  * @param goal The goal
  * @return A samples; calling function has to check the timestamp of the generated sample to determine its validity
  */
  SampleTemplate generateTemplateFromFullGoal(const FullGoal& goal) const;

  SampleTemplate generateTemplateFromPositionAndCenterCircle(const Vector2<>& posSeen, const Vector2<>& circlePosSeen,
      const Vector2<>& posReal, const Pose2D& postOdometry) const;

  SampleTemplate generateTemplateFromPosition(const Vector2<>& posSeen,
      const Vector2<>& posReal, const Pose2D& postOdometry) const;

  SampleTemplate generateRandomTemplate() const;

  /**
  * The function checks whether a position is insided the configured area to which template generation should be clipped.
  * @param pos A point on the field
  * @return true if the point is inside this area
  */
  bool checkTemplateClipping(const Vector2<> pos) const;

public:
  SampleTemplateGenerator(const SelfLocatorParameters& parameters,
                          const GoalPercept& goalPercept, const LinePercept& linePercept, const FrameInfo& frameInfo,
                          const FieldDimensions& fieldDimensions, const OdometryData& odometryData);

  /** Empty all buffers. */
  void init();
  
  /** Buffers current goals perceptions. */
  void bufferNewPerceptions();

  SampleTemplate getNewTemplate();

  bool templatesAvailable() const;

  void draw();
};
