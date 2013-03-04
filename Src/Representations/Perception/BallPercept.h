/**
* @file BallPercept.h
*
* Very simple representation of a seen ball
*
* @author <a href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</a>
*/

#pragma once

#include "Tools/Math/Vector2.h"

class BallPercept : public Streamable
{
public:
  Vector2<> positionInImage;         /**< The position of the ball in the current image */
  float radiusInImage;                    /**< The radius of the ball in the current image */
  bool ballWasSeen;                        /**< Indicates, if the ball was seen in the current image. */
  Vector2<> relativePositionOnField; /**< Ball position relative to the robot. */

  /** Constructor */
  BallPercept() : ballWasSeen(false) {}

  /** Draws the ball*/
  void draw();

private:
  /** Streaming function
  * @param in  streaming in ...
  * @param out ... streaming out.
  */
  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(positionInImage);
    STREAM(radiusInImage);
    STREAM(ballWasSeen);
    STREAM(relativePositionOnField);
    STREAM_REGISTER_FINISH;
  }
};
