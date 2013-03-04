/**
 * @file USControl.h
 * Declaration of a module that controls the firing strategy
 * of the ultrasound sensors.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/USRequest.h"

MODULE(USControl)
  PROVIDES_WITH_MODIFY(USRequest)
END_MODULE

/**
 * @class USControl
 * A module that controls the firing strategy
 * of the ultrasound sensors.
 */
class USControl : public USControlBase
{
private:
  class Parameters : public Streamable
  {
  private:
    /**
     * The method makes the object streamable.
     * @param in The stream from which the object is read.
     * @param out The stream to which the object is written.
     */
    virtual void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN;
      STREAM(sendInterval);
      STREAM(switchInterval);
      STREAM(ignoreAfterSwitchInterval);
      STREAM(timeBetweenSendAndReceive);
      STREAM(modes);
      STREAM_REGISTER_FINISH;
    }

  public:
    int sendInterval;
    int switchInterval;
    int ignoreAfterSwitchInterval;
    int timeBetweenSendAndReceive; /** < time to wait between send an receive command (in ms) */
    std::vector<int> modes;
  };
  
  Parameters parameters; /**< Ultrasonic measurement mode settings. */
  unsigned lastSendTime; /**< The time when the last ultrasonic wave was send. */
  unsigned lastSwitchTime; /**< The time when the used transmitter was changed. */
  unsigned lastReadTime; /**< The time when the last measurement was read. */
  unsigned currentMode; /**< The index of the transmitter mode that is currently active. */
  bool commandSent; /** < True if a command was sent to libbhuman one frame ago */
  
  void update(USRequest& usRequest);
  
public:
  USControl();
};
