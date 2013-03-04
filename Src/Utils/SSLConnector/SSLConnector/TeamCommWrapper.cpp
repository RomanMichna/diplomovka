/**
 * @file TeamCommWrapper.cpp
 * @author <a href="afabisch@tzi.de>Alexander Fabisch</a>
 */
#include "TeamCommWrapper.h"
#include "Platform/BHAssert.h"
#include "Platform/SystemCall.h"
#include "Tools/Debugging/Debugging.h"

TeamCommWrapper::TeamCommWrapper(MessageQueue& teamOut, int teamPort) : theTeamSender(teamOut), INIT_TEAM_COMM
{
  theTeamSender.setSize(1396); // 1 package without checksum and size
  theTeamReceiver.setSize(4 * 1450); // >= 4 packages

  theTeamHandler.start(teamPort, "255.255.255.255");

  ntp.doSynchronization(SystemCall::getRealSystemTime(), Global::getTeamOut());
  theTeamHandler.send();
  theTeamHandler.receive();
  theTeamReceiver.handleAllMessages(*this);
  theTeamReceiver.clear();
}

void TeamCommWrapper::receive()
{
  OUTPUT(idText, text, "Receiving TeamComm...");
  RECEIVE_TEAM_COMM;
  theTeamReceiver.handleAllMessages(*this);
  theTeamReceiver.clear();
  ntp.doSynchronization(SystemCall::getCurrentSystemTime(), Global::getTeamOut()); // usually 2000 ms between 2 synchronizations
}

void TeamCommWrapper::send()
{
  OUTPUT(idText, text, "Sending TeamComm...");
  SEND_TEAM_COMM;
  theTeamSender.clear();
}

bool TeamCommWrapper::handleMessage(InMessage& message)
{
  OUTPUT(idText, text, "Message " << getName(message.getMessageID()) << " received.");
  switch(message.getMessageID())
  {
    case idNTPHeader:
      VERIFY(ntp.handleMessage(message));
      timeStamp = ntp.receiveTimeStamp;
      return false;
    case idNTPIdentifier:
    case idNTPRequest:
    case idNTPResponse:
    default:
      return ntp.handleMessage(message);
  }
}
