/**
* @file Stopwatch.h
* The file declares the stopwatch macros.
* @author <a href="mailto:juengel@informatik.hu-berlin.de">Matthias Jüngel</a>
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#pragma once

#include "Platform/SystemCall.h"

#ifdef RELEASE
#include "Tools/Team.h"
#include "Tools/Debugging/ReleaseOptions.h"

#define STOP_TIME_ON_REQUEST(eventID, expression) \
  do \
  { \
    const char* _p = eventID; \
    if(Global::getReleaseOptions().stopwatch == *_p) \
    { \
      static unsigned _counter = 0; \
      const unsigned _startTimeN = SystemCall::getCurrentThreadTime(); \
      { expression } \
      TEAM_OUTPUT(idStopwatch, bin, eventID << _startTimeN << (unsigned)SystemCall::getCurrentThreadTime() << SystemCall::getRealSystemTime() << _counter++); \
    } \
    else \
    { \
      expression \
    } \
  } \
  while(false)

#define STOP_TIME_ON_REQUEST_WITH_PLOT(eventID, expression) STOP_TIME_ON_REQUEST(eventID, expression)

#else // RELEASE
#include "Tools/Debugging/Debugging.h"
#include "Tools/Debugging/DebugDrawings.h"
/*
 * Allows for the measurement of time
 * @param eventID The id of the stop watch
 * @param expression The expression of which the execution time is measured
 */
#define STOP_TIME_ON_REQUEST(eventID, expression) \
  do \
  { \
    DEBUG_RESPONSE("stopwatch:" eventID, \
    { \
      static unsigned _counter = 0; \
      const unsigned _startTimeN = (unsigned)SystemCall::getCurrentThreadTime(); \
      { expression } \
      OUTPUT(idStopwatch, bin, eventID << _startTimeN << (unsigned)SystemCall::getCurrentThreadTime() << SystemCall::getRealSystemTime() << _counter++); \
    }); \
    DEBUG_RESPONSE_NOT("stopwatch:" eventID, expression ); \
  } \
  while(false)

#define STOP_TIME_ON_REQUEST_WITH_PLOT(eventID, expression) \
  do \
  { \
    DECLARE_PLOT("stopwatch:" eventID); \
    DEBUG_RESPONSE("stopwatch:" eventID, \
    { \
      static unsigned _counter = 0; \
      const unsigned _startTimeN = (unsigned)SystemCall::getCurrentThreadTime(); \
      { expression } \
      const unsigned _endTimeN = (unsigned)SystemCall::getCurrentThreadTime(); \
      OUTPUT(idStopwatch, bin, eventID << _startTimeN << _endTimeN << SystemCall::getRealSystemTime() << _counter++); \
      PLOT("stopwatch:" eventID, float(_endTimeN - _startTimeN) * 0.001f); \
    }); \
    DEBUG_RESPONSE_NOT("stopwatch:" eventID, expression ); \
  } \
  while(false)

#endif // RELEASE
