/**
* @file ReleaseOptions.h
* This file declares a class representing the debugging options in release code.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#pragma once

#include "Tools/Streams/Streamable.h"

/**
* @class ReleaseOptions
* A class representing the debugging options in release code.
*/
class ReleaseOptions : public Streamable
{
private:
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(stopwatch);
    STREAM(sensorData);
    STREAM(robotHealth);
    STREAM(motionRequest);
    STREAM(linePercept);
    STREAM(plotBallTimes);
    STREAM(combinedWorldModel);
    STREAM(freePartOfOpponentGoal);
    STREAM_REGISTER_FINISH;
  }

public:
  /**
  * Default constructor.
  */
  ReleaseOptions() :
    stopwatch(0),
    sensorData(false),
    robotHealth(false),
    motionRequest(false),
    linePercept(false),
    plotBallTimes(false),
    combinedWorldModel(false),
    freePartOfOpponentGoal(false) {}

  char stopwatch;              /**< The first character of the name of the stopwatch to send. */
  bool sensorData,             /**< Activate sending sensorData. */
       robotHealth,            /**< Activate sending robot health data. */
       motionRequest,          /**< Activate sending motion requests. */
       linePercept,            /**< Activate sending line percepts. */
       plotBallTimes,          /**< Activate sending ball time plots. */
       combinedWorldModel,      /**< Activate sending combined world model */
       freePartOfOpponentGoal; /**< Activate sending free part of opponent goal. */
};
