/**
* @file Representations/BehaviorControl/BehaviorData.h
* The file declares a class that containts data about the current behavior state.
* @author Colin Graf
*/

#pragma once

#include "Tools/Math/Vector2.h"
#include "Tools/Enum.h"

/**
* @class BehaviorData
* A class that containts data about the current behavior state.
*/
class BehaviorData : public Streamable
{
public:
  ENUM(Role,
    undefined,
    firstRole,
    keeper = firstRole,
    supporter,
    striker,
    defender,
    offensiveSupporter,
    defensiveSupporter
  );
  Role role; /**< A dynamically chosen role. */

  ENUM(Activity,
    unknown,
    dribble,
    goToBall,
    searchForBall,
    goToTarget,
    prepareKick,
    kick,
    kickSidewards,
    pass,
    block,
    hold,
    standUp,
    patrol,
    passBeforeGoal,
    kickoff,
    goAround,
    waitForPass,
    preparePass,
    parry,
    guard,
    defending,
    receive,
    duel
  );
  Activity activity; /**< What is the robot doing in general? */

  ENUM(TeamColor,
    red,
    blue
  );

  float estimatedTimeToReachBall;   /**< The estimated time to reach the kick pose (in ms) */
  bool kickoffInProgress;           /**< Whether the kickoff is currently in progress or not. */
  char selectedKickoff;              /**< The kickoff strategy that has been selected */
  char kickoffPositionAssigment[3]; /**< Assignment of robot numbers to positions (order as specified in current kickoff strategy definition)*/
  TeamColor teamColor;

  enum HeadControlFlags
  {
    checkingBall = 0x01, /**< Whether the robot started panning to the ball or is currently looking at the ball */
    awareOfBall = 0x02, /**< Whether the robot claims to know where the ball is */
  };
  unsigned char headControlFlags;

  bool aloneAndConfused;
  unsigned char enteredStates;
  bool relax;

  /**
  * Default constructor.
  */
  BehaviorData() :
    role(undefined),
    activity(unknown),
    estimatedTimeToReachBall(10000000.0f),
    kickoffInProgress(false),
    selectedKickoff(-1),
    teamColor(red),
    headControlFlags(0),
    aloneAndConfused(false),
    enteredStates(0x0),
    relax(false)
  {}

private:
  /**
  * The method makes the object streamable.
  * @param in The stream from which the object is read (if in != 0).
  * @param out The stream to which the object is written (if out != 0).
  */
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM_AS_UCHAR(role);
    STREAM_AS_UCHAR(activity);
    STREAM(estimatedTimeToReachBall);
    STREAM(kickoffInProgress);
    STREAM(selectedKickoff);
    STREAM(aloneAndConfused);
    STREAM(kickoffPositionAssigment);
    STREAM_AS_UCHAR(teamColor);
    STREAM_AS_UCHAR(headControlFlags);
    STREAM_AS_UCHAR(enteredStates);
    STREAM_REGISTER_FINISH;
  }
};
