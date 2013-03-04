/**
* @file ArmContactModel.h
*
* Declaration of class ArmContactModel.
* @author <a href="mailto:fynn@informatik.uni-bremen.de">Fynn Feldpausch</a>
* @author <a href="mailto:simont@informatik.uni-bremen.de">Simon Taddiken</a>
* @author <a href="mailto:arneboe@informatik.uni-bremen.de">Arne Böckmann</a>
*
*/

#pragma once

#include "Tools/Debugging/Debugging.h"
/**
* @class ArmContactModel
*/
class ArmContactModel : public Streamable
{

public:
    ENUM(PushDirection,
        N,    /** < Arm is being pushed to the front */
        S,    /** < Arm is being pushed to the back */
        W,    /** < Arm is being pushed to the left */
        E,    /** < Arm is being pushed to the right */
        NW,   /** < Arm is being pushed to the front and the left */
        NE,   /** < Arm is being pushed to the front and the right */
        SW,   /** < Arm is being pushed to the back and the left */
        SE,   /** < Arm is being pushed to the back and the right */
        NONE  /** < If no contact is detected */
    );


  bool contactLeft;          /** The contact state of the robot's left arm. */
  bool contactRight;         /** The contact state of the robot's right arm. */

  /**
   * The duration of the push in motion frames (100fps = 1s).
   */
  unsigned int durationLeft;
  unsigned int durationRight;
  
  /** only evaluate these values if contactLeft or contactRight is true */
  PushDirection pushDirectionLeft;       /**< direction in which the left arm is being pushed */
  PushDirection pushDirectionRight;      /**< direction in which the right arm is being pushed */

  /** only evaluate these values if contactLeft or contactRight is true */
  PushDirection lastPushDirectionLeft;       /**< direction in which the left arm was last pushed */
  PushDirection lastPushDirectionRight;      /**< direction in which the right arm was last pushed */

  /**
   *
   */
  unsigned int timeOfLastContactLeft;
  unsigned int timeOfLastContactRight;



  /**
  * Mirrors the push directions on north/south axis (that is, east and west 
  * are switched)
  */
  static void mirrorDirection(PushDirection& dir) {
    switch (dir) 
    {
      case E:  dir = W;  break;
      case W:  dir = E;  break;
      case NE: dir = NW; break;
      case NW: dir = NE; break;
      case SE: dir = SW; break;
      case SW: dir = SE; break;
      case N:
      case S:
      case NONE:
      case numOfPushDirections:
        // do nothing
        break;
    }
  }


  ArmContactModel() :
    contactLeft(false),
    contactRight(false),
    durationLeft(0),
    durationRight(0),
    pushDirectionLeft(NONE),
    pushDirectionRight(NONE),
    lastPushDirectionLeft(NONE),
    lastPushDirectionRight(NONE),
    timeOfLastContactLeft(0),
    timeOfLastContactRight(0){}

private:
  /** Streaming function
  * @param in Object for streaming in the one direction
  * @param out Object for streaming in the other direction
  */
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(contactLeft);
    STREAM(contactRight);
    STREAM(pushDirectionLeft);
    STREAM(pushDirectionRight);
    STREAM(lastPushDirectionLeft);
    STREAM(lastPushDirectionRight);
    STREAM(durationLeft);
    STREAM(durationRight);
    STREAM(timeOfLastContactLeft);
    STREAM(timeOfLastContactRight);
    STREAM_REGISTER_FINISH;
  }
};
