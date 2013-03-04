/*
 * FootContactModelProvider.cpp
 *
 *  Created on: Mar 14, 2012
 *      Author: arne
 *              simont@tzi.de
 */

#include "FootContactModelProvider.h"
#include "Platform/SoundPlayer.h"
#include "Tools/Debugging/DebugDrawings.h"

MAKE_MODULE(FootContactModelProvider, Modeling);


FootContactModelProvider::FootContactModelProvider():
  contactDurationLeft(0), contactDurationRight(0), leftFootLeftDuration(0), leftFootRightDuration(0), rightFootLeftDuration(0), rightFootRightDuration(0)
{
  p.contactThreshold = 15;
  p.debug = false;
  p.malfunctionThreshold = 250;  // bumper is ignored after 2.5seconds of constant contact
  p.soundDelay = 1000;
}


void FootContactModelProvider::update(FootContactModel& model)
{
  MODIFY("module:FootContactModelProvider:parameters", p);

  // Check, if any bumper is pressed
  bool leftFootLeft = checkContact(KeyStates::leftFootLeft, leftFootLeftDuration);
  bool leftFootRight = checkContact(KeyStates::leftFootRight, leftFootRightDuration);
  bool rightFootLeft = checkContact(KeyStates::rightFootLeft, rightFootLeftDuration);
  bool rightFootRight = checkContact(KeyStates::rightFootRight, rightFootRightDuration);

  // Update statistics
  if (leftFootLeft || leftFootRight)
  {
    contactBufferLeft.add(1);
    contactDurationLeft++;
  }
  else
  {
    contactBufferLeft.add(0);
    contactDurationLeft = 0;
  }
  if (rightFootLeft || rightFootRight)
  {
    contactBufferRight.add(1);
    contactDurationRight++;
  }
  else
  {
    contactBufferRight.add(0);
    contactDurationRight = 0;
  }

  // Generate model
  if ((theMotionInfo.motion == MotionInfo::stand || theMotionInfo.motion == MotionInfo::walk) &&
      (theGameInfo.state == STATE_READY || theGameInfo.state == STATE_SET || theGameInfo.state == STATE_PLAYING) && //The bumper is used for configuration in initial
      (theFallDownState.state == FallDownState::upright))
  {
    if(contactBufferLeft.getSum() > p.contactThreshold)
    {
      model.contactLeft = true;
      model.contactDurationLeft = contactDurationLeft;
      model.lastContactLeft = theFrameInfo.time;
    }
    else
    {
      model.contactLeft = false;
      model.contactDurationLeft = 0;
    }
    if(contactBufferRight.getSum() > p.contactThreshold)
    {
      model.contactRight = true;
      model.contactDurationRight = contactDurationRight;
      model.lastContactRight = theFrameInfo.time;
    }
    else
    {
      model.contactRight = false;
      model.contactDurationRight = 0;
    }
  }
  else
  {
    model.contactLeft = false;
    model.contactRight = false;
    model.contactDurationLeft = 0;
    model.contactDurationRight = 0;
  }

  // Debugging stuff:

  if(p.debug && theFrameInfo.getTimeSince(lastSoundTime) > (int)p.soundDelay &&
    (model.contactLeft || model.contactRight))
  {
    lastSoundTime = theFrameInfo.time;
    SoundPlayer::play("doh.wav");
  }


  DECLARE_PLOT("module:FootContactModelProvider:sumLeft");
  DECLARE_PLOT("module:FootContactModelProvider:durationLeft");
  DECLARE_PLOT("module:FootContactModelProvider:contactLeft");
  DECLARE_PLOT("module:FootContactModelProvider:sumRight");
  DECLARE_PLOT("module:FootContactModelProvider:durationRight");
  DECLARE_PLOT("module:FootContactModelProvider:contactRight");
  DECLARE_PLOT("module:FootContactModelProvider:leftFootLeft");
  DECLARE_PLOT("module:FootContactModelProvider:leftFootRight");
  DECLARE_PLOT("module:FootContactModelProvider:rightFootLeft");
  DECLARE_PLOT("module:FootContactModelProvider:rightFootRight");
  PLOT("module:FootContactModelProvider:sumLeft", contactBufferLeft.getSum());
  PLOT("module:FootContactModelProvider:durationLeft", contactDurationLeft);
  PLOT("module:FootContactModelProvider:sumRight", contactBufferRight.getSum());
  PLOT("module:FootContactModelProvider:durationRight", contactDurationRight);
  PLOT("module:FootContactModelProvider:contactLeft", model.contactLeft ? 10 : 0);
  PLOT("module:FootContactModelProvider:contactRight", model.contactRight ? 10 : 0);
  PLOT("module:FootContactModelProvider:leftFootLeft", leftFootLeft ? 10 : 0);
  PLOT("module:FootContactModelProvider:leftFootRight", leftFootRight ? 10 : 0);
  PLOT("module:FootContactModelProvider:rightFootLeft", rightFootLeft ? 10 : 0);
  PLOT("module:FootContactModelProvider:rightFootRight", rightFootRight ? 10 : 0);
}


bool FootContactModelProvider::checkContact(KeyStates::Key key, int& duration)
{
  bool pressed = theKeyStates.pressed[key];
  duration = pressed ? duration + 1 : 0;
  // if key is pressed longer than the malfunction threshold, it is ignored
  return pressed && duration < p.malfunctionThreshold;
}
