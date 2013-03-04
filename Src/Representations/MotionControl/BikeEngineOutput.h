/**
* @file Representations/MotionControl/BikeEngineOutput.h
* This file declares a class that represents the output of modules generating motion.
* @author <A href="mailto:judy@tzi.de">Judith Müller</A>
*/

#pragma once

#include "Representations/Infrastructure/JointData.h"
#include "Tools/Math/Pose2D.h"
#include "Representations/MotionControl/BikeRequest.h"

/**
* @class BikeEngineOutput
* A class that represents the output of the walking engine.
*/
class BikeEngineOutput : public JointRequest
{
protected:
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM_BASE(JointRequest);
    STREAM(odometryOffset);
    STREAM(isLeavingPossible);
    STREAM(executedBikeRequest);
    STREAM_REGISTER_FINISH;
  }

public:
  Pose2D odometryOffset; /**< The body motion performed in this step. */
  bool isLeavingPossible; /**< Is leaving the motion module possible now? */
  BikeRequest executedBikeRequest; /**< The bike request that is currently in execution. */

  /**
  * Default constructor.
  */
  BikeEngineOutput() : isLeavingPossible(true) {}
};
