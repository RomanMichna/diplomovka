/**
 * @file TeamCommWrapper.h
 * @author <a href="afabisch@tzi.de>Alexander Fabisch</a>
 */
#ifndef TEAMCOMMWRAPPER_H
#define TEAMCOMMWRAPPER_H

#include "Tools/MessageQueue/MessageQueue.h"
#include "Tools/NTP.h"
#include "Tools/ProcessFramework/TeamHandler.h"

/**
 * A wrapper for the BHuman team communication. You just have to call send
 * and receive in every frame and you can use the TEAM_OUTPUT macro if the
 * global team out is initialized (see Framework class).
 */
class TeamCommWrapper : public MessageHandler
{
  MessageQueue theTeamReceiver;
  MessageQueue& theTeamSender;
  TeamHandler theTeamHandler;
  NTP ntp;
  unsigned long timeStamp;
  int robotNumber;

  virtual bool handleMessage(InMessage& message);

public:
  TeamCommWrapper(MessageQueue& teamOut, int teamPort);

  void receive();
  void send();
};

#endif // TEAMCOMMWRAPPER_H
