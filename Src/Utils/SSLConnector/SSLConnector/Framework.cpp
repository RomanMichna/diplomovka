/**
 * @file Framework.cpp
 * @author <a href="afabisch@tzi.de>Alexander Fabisch</a>
 */
#include "Framework.h"

#include "Tools/Global.h"
#include "Platform/BHAssert.h"
#include <cstdlib>

std::map<std::string, Framework*> Framework::theInstances;

Framework* Framework::getInstance(const std::string& processName, int playerNumber)
{
  if(!theInstances[processName])
    theInstances[processName] = new Framework(processName, playerNumber);
  return theInstances[processName];
}

void Framework::destroy(const std::string& processName)
{
  delete theInstances[processName];
  theInstances.erase(processName);
}

Framework::Framework(const std::string& processName, int playerNumber)
    : blackboard(new Blackboard),
      playerNumber(playerNumber),
      requestShutdown(false)
{
  ASSERT(!theInstances[processName]);
  ASSERT(Settings::loaded);
  settings.playerNumber = playerNumber;
  Global::theDebugOut = &debugOut.out;
  Global::theTeamOut = &teamOut.out;
  if(!Global::theSettings)
    Global::theSettings = &settings;
  if(!Global::theDebugRequestTable)
    Global::theDebugRequestTable = &debugRequestTable;
  if(!Global::theDebugDataTable)
    Global::theDebugDataTable = &debugDataTable;
  if(!Global::theStreamHandler)
    Global::theStreamHandler = &streamHandler;
  if(!Global::theDrawingManager)
    Global::theDrawingManager = &drawingManager;

  Blackboard::theInstance = blackboard;
  moduleManager = ModuleManager(processName);
  moduleManager.load();
}

Framework::Framework(const Framework& framework)
{
  ASSERT(false);
}

Framework::Framework()
{
  ASSERT(false);
}

Framework::~Framework()
{ // TODO seg fault
  moduleManager.destroy();
  delete blackboard;
  streamHandler.clear();
}
