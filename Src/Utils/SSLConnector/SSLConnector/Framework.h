/**
 * @file Framework.cpp
 * @author <a href="afabisch@tzi.de>Alexander Fabisch</a>
 */
#ifndef FRAMEWORKINITIALIZER_H
#define FRAMEWORKINITIALIZER_H

#include "Tools/MessageQueue/MessageQueue.h"
#include "Tools/Settings.h"
#include "Tools/Debugging/DebugRequest.h"
#include "Tools/Debugging/DebugDataTable.h"
#include "Tools/Streams/StreamHandler.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Module/ModuleManager.h"
#include "Representations/Blackboard.h"
#include <string>
#include <map>

/**
 * A wrapper for the BHuman framework. Create one instance and you do not have
 * to worry about any initializations. This is implemented as a singleton.
 */
class Framework
{
  static std::map<std::string, Framework*> theInstances;

  Framework(const std::string& processName, int playerNumber);
  Framework(const Framework& framework); /**< Should never be used. */
  Framework();                           /**< Should never be used. */
  ~Framework();
public:
  static Framework* getInstance(const std::string& processName, int playerNumber = 0);
  static void destroy(const std::string& processName);

  MessageQueue debugOut;
  MessageQueue teamOut;
  Settings settings;
  DebugRequestTable debugRequestTable;
  DebugDataTable debugDataTable;
  StreamHandler streamHandler;
  DrawingManager drawingManager;
  ModuleManager moduleManager;
  Blackboard* blackboard;
  int playerNumber;
  bool requestShutdown;
};

#endif // FRAMEWORKINITIALIZER_H
