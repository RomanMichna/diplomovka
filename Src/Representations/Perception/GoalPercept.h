/**
* @file GoalPercept.h
*
* Representation of a seen goal
*
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
*/

#pragma once

#include "Tools/ColorClasses.h"
#include "Tools/Streams/Streamable.h"
#include "Tools/Math/Vector2.h"
#include "Representations/Configuration/FieldDimensions.h"

/**
* @class GoalPost
* Description of a perceived goal post
*/
class GoalPost: public Streamable
{
  /** Streaming function
  * @param in  streaming in ...
  * @param out ... streaming out.
  */
  void serialize(In* in, Out* out);

public:
  
  //variable for the position of this post
  ENUM(Position, IS_UNKNOWN, IS_LEFT, IS_RIGHT);
  Position position;
  
  /** The position of the goal post in the current image */
  Vector2<int> positionInImage;
  /** The position of the goal post relative to the robot*/
  Vector2<int> positionOnField;
  /** The two different kinds of distance computation*/
  ENUM(DistanceType, HEIGHT_BASED, BEARING_BASED);
  DistanceType distanceType;
  
  //static Vector2<> getRealPositionOnField(Position position, Membership member, const FieldDimensions& fieldDimensions);

  /** Constructor */
  GoalPost() :  position(IS_UNKNOWN), positionInImage(Vector2<int>(0, 0)), positionOnField(Vector2<int>(0, 0)),
    distanceType(GoalPost::BEARING_BASED) {}
};


/**
* @class GoalPercept
* Set of perceived goal posts
*/
class GoalPercept: public Streamable
{
  /** Streaming function
  * @param in  streaming in ...
  * @param out ... streaming out.
  */
  void serialize(In* in, Out* out);

public:
  vector<GoalPost> goalPosts;

  unsigned timeWhenGoalPostLastSeen; /**< Time when a goal post was seen. */
  unsigned timeWhenCompleteGoalLastSeen; /**< Time when complete goal was seen. */

  /** Constructor */
  GoalPercept() :
    timeWhenGoalPostLastSeen(0),
    timeWhenCompleteGoalLastSeen(0)
  {}

  /** Draws the perceived goal posts*/
  void draw();
};
