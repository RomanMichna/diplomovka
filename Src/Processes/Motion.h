/**
* @file Processes/Motion.h
* Declaration of a class that represents the process that sends commands to the robot at 100Hz.
*/

#pragma once

#include "Tools/ProcessFramework/Process.h"
#include "Tools/Module/ModulePackage.h"

/**
* @class Motion
* A class that represents the process that sends commands to the robot at 100Hz.
*/
class Motion : public Process
{
private:
  DEBUGGING;
  RECEIVER(CognitionToMotion);
  SENDER(MotionToCognition);

public:
  /**
  * Default constructor.
  */
  Motion();

  /**
  * Default destructor.
  * Sets the global pointers again (only for Simulator).
  */
  ~Motion() {setGlobals();}

  /**
  * The method is called from the framework once in every frame.
  * @return Should wait for external trigger?
  */
  virtual bool main();

  /**
  * The method is called directly before the first call of main().
  */
  virtual void init();

  /**
  * The method is called when the process is terminated.
  */
  virtual void terminate();

  /**
  * The function handles incoming debug messages.
  * @param message the message to handle.
  * @return Has the message been handled?
  */
  virtual bool handleMessage(InMessage& message);

private:
  ModuleManager moduleManager; /**< The solution manager handles the execution of modules. */
};
