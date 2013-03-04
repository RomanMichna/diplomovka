/**
 * @file DebugHandler.cpp
 * @author <a href="afabisch@tzi.de>Alexander Fabisch</a>
 */
#include "DebugHandler.h"

#include <iostream>
#include <Representations/Modeling/SSLVisionData.h>

bool DebugHandler::handleMessage(InMessage& message)
{
  switch(message.getMessageID())
  {
    case idText:
    {
      std::string msg = message.text.readAll();
      std::cout << msg << std::endl;
      return true;
    }
    default:
      return false;
  }
}

void DebugHandler::frameFinished()
{
  debugOut.handleAllMessages(*this);
  debugOut.clear();
}
