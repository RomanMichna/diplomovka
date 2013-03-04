/**
* @file SideConfidence.h
*
* Declaration of class SideConfidence.
* @author Michel Bartsch, Thomas Muender
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
*/

#pragma once

#include "Tools/Streams/Streamable.h"
#include "Tools/Enum.h"
#include "Tools/Debugging/DebugDrawings3D.h"


/**
* @class SideConfidence
*/
class SideConfidence : public Streamable
{
public:
  float sideConfidence;            /**< Am I mirrored because of two yellow goals (0 = no idea, 1 = absolute sure I am right). */
  bool mirror;                     /**< Indicates whether ball model of others is mirrored to own ball model. */
  ENUM(ConfidenceState,
    CONFIDENT,
    ALMOST_CONFIDENT,
    UNSURE,
    CONFUSED
  );                               /**< Discrete states of confidence, mapped by provider */
  ConfidenceState confidenceState; /**< The state of confidence */
  
  /** Constructor */
  SideConfidence() : sideConfidence(1.0f), mirror(false), confidenceState(CONFIDENT) {}

  /** Draw representation. */
  void draw()
  {
    DECLARE_DEBUG_DRAWING3D("representation:SideConfidence", "Head", 
    {
      static const ColorRGBA colors[numOfConfidenceStates] =
      {
        ColorRGBA(0, 255, 0),
        ColorRGBA(0, 128, 0),
        ColorRGBA(255, 255, 0),
        ColorRGBA(255, 0, 0)
      };
      POINT3D("representation:SideConfidence", 0, 0, 130, 10, colors[confidenceState]);
    });

    DECLARE_DEBUG_DRAWING("representation:SideConfidence","drawingOnField",
    {
        DRAWTEXT("representation:SideConfidence",-3500,-2600,14,ColorClasses::red,"Sideconfidence: " << sideConfidence);
    }
    );

  }
  
private:
  /**
  * The method makes the object streamable.
  * @param in The stream from which the object is read (if in != 0).
  * @param out The stream to which the object is written (if out != 0).
  */
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(sideConfidence);
    STREAM(mirror);
    STREAM(confidenceState);
    STREAM_REGISTER_FINISH;
  }
};
