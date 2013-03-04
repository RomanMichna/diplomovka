/**
 * @file OwnSideModel.h
 * The file implements a model that states that the robot cannot have left its own
 * side since the last kick-off and how far it can have gotten along the field.
 *
 * @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 */

#pragma once

#include "Tools/Streams/Streamable.h"

/**
 * @class OwnSideModel
 * A model that states that the robot cannot have left its own
 * side since the last kick-off and how far it can have gotten along
 * the field.
 */
class OwnSideModel : public Streamable
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
    STREAM(stillInOwnSide);
    STREAM(largestXPossible);
    STREAM(returnFromGameControllerPenalty);
    STREAM(returnFromManualPenalty);
    STREAM_REGISTER_FINISH;
  }
  
public:
  bool stillInOwnSide; /**< The robot must still be in its own side. */
  float largestXPossible; /**< The largest x-coordinate that is currently possible. */
  bool returnFromGameControllerPenalty; /**< The robot was unpenalized by the GameController and believes it. */
  bool returnFromManualPenalty; /**< The robot was unpenalized by the GameController and believes it. */
  
  OwnSideModel() : stillInOwnSide(false), largestXPossible(100000.f), 
                   returnFromGameControllerPenalty(false), returnFromManualPenalty(false) {}
};
