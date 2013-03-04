/**
 * @file USControl.cpp
 * Implementation of a module that controls the firing strategy
 * of the ultrasound sensors.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas R�fer</a>
 */

#include "USControl.h"
#include "Platform/SystemCall.h"
#include "Tools/Settings.h"
#include "Tools/Streams/InStreams.h"

USControl::USControl() : 
lastSendTime(0), lastSwitchTime(0), lastReadTime(0), currentMode(0), commandSent(false)
{
  InConfigMap stream(Global::getSettings().expandLocationFilename("usControl.cfg"));
  ASSERT(stream.exists());
  stream >> parameters;
}

void USControl::update(USRequest& usRequest)
{
  MODIFY("parameters:USControl", parameters);
  

  if(commandSent)
  {
    usRequest.sendMode = -1;
    if (SystemCall::getTimeSince(lastSendTime) >= parameters.timeBetweenSendAndReceive)
    {
      usRequest.receiveMode = parameters.modes[currentMode];
      commandSent = false;
    }
    // wait a while more
  }
  else
  {
    usRequest.receiveMode = -1; //-1 = do not read anything

    //If a command has been sent last frame: check if we should send one this frame
    if(SystemCall::getTimeSince(lastSendTime) >= parameters.sendInterval)
    {
      if(SystemCall::getTimeSince(lastSwitchTime) >= parameters.switchInterval)
      {
        currentMode = (currentMode + 1) % parameters.modes.size();
        lastSwitchTime = SystemCall::getCurrentSystemTime();
      }
      usRequest.sendMode = parameters.modes[currentMode];
      lastSendTime = SystemCall::getCurrentSystemTime();
      commandSent = true;
    }
    else
    {
      usRequest.sendMode = -1; //do not send anything
    }

  }



}

MAKE_MODULE(USControl, Sensing)
