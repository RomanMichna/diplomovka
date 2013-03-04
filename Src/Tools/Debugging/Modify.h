/*
* @file Tools/Debugging/Modify.h
* Macros make data visible/editable through RobotControl.
*
* @author Michael Spranger
* @author Tobias Oberlies
*/


#include "DebugDataTable.h"

#ifdef RELEASE
#define MODIFY(name, object) ((void) 0)
#define MODIFY_ENUM(name, ...) ((void) 0)
#else

/**
 * Allows for the modification of an object implementing a complete streaming operator
 * with macros from Tools/Streams/Streamable.h (or scalar).
 * @param id An identifier string of the object. Separate hirarchy levels by a single
 *     colon ':', i.e. "cognition:ball percept". The value must be a string constant.
 * @param object The object to be modified.
 */
#define MODIFY(id, object) \
  do \
  { \
    DebugDataTable::testForStreamable(object); \
    Global::getDebugDataTable().updateObject(id, object); \
    DEBUG_RESPONSE("debug data:"id, \
    { \
      OUTPUT(idDebugDataResponse, bin, id << Streaming::demangle(typeid(object).name()) << object); \
    }); \
  } \
  while(false)

/**
 * Allows for the modification of an enumerated value.
 * @param id An identifier string of the variable.
 * The second parameter is the enumeration to be modified.
 * If the type of that enumeration is not defined in the current class,
 * the name of the class in which it is defined has to be specified as
 * third parameter.
 */
#define MODIFY_ENUM(id, ...) \
  STREAM_EXPAND(STREAM_EXPAND(STREAM_THIRD(__VA_ARGS__, MODIFY_ENUM_WITH_CLASS, MODIFY_ENUM_WITHOUT_CLASS))(id, __VA_ARGS__))

#define MODIFY_ENUM_WITHOUT_CLASS(id, object) \
  do \
  { \
    DebugDataTable::testForStreamable(object); \
    Streaming::registerEnum(typeid(object), Streaming::castFunction(object, getName)); \
    Global::getDebugDataTable().updateEnum(id, object); \
    DEBUG_RESPONSE( "debug data:"id, \
    { \
      OUTPUT(idDebugDataResponse, bin, id << Streaming::demangle(typeid(object).name()) << (int)object); \
    }); \
  } \
  while(false)

#define MODIFY_ENUM_WITH_CLASS(id, object, class) \
  do \
  { \
    DebugDataTable::testForStreamable(object); \
    Streaming::registerEnum(typeid(object), Streaming::castFunction(object, class::getName)); \
    Global::getDebugDataTable().updateEnum(id, object); \
    DEBUG_RESPONSE( "debug data:"id, \
    { \
      OUTPUT(idDebugDataResponse, bin, id << Streaming::demangle(typeid(object).name()) << (int)object); \
    }); \
  } \
  while(false)

#endif
