/**
 * @file DebugHandler.h
 * @author <a href="afabisch@tzi.de>Alexander Fabisch</a>
 */
#ifndef DEBUGHANDLER_H
#define DEBUGHANDLER_H

#include "Tools/MessageQueue/MessageQueue.h"

/**
 * A message handler that prints every text message to stdout.
 */
class DebugHandler : public MessageHandler
{
  MessageQueue& debugOut;

  virtual bool handleMessage(InMessage& message);

public:
  DebugHandler(MessageQueue& debugOut) : debugOut(debugOut) {}

  /** Handles all text messages and clears the message queue. */
  void frameFinished();
};

#endif // DEBUGHANDLER_H
