/**
* @file Processes/Motion.cpp
* Implementation of a class that represents the process that sends commands to the robot at 50Hz.
*/
#include "Motion.h"
#include "Modules/Infrastructure/NaoProvider.h"
#include "Modules/Infrastructure/MotionLogDataProvider.h"
#include "Modules/MotionControl/SpecialActions.h"
#include "Modules/MotionControl/WalkingEngine/WalkingEngine.h"
#include "Modules/MotionControl/MotionSelector.h"

Motion::Motion() :
  INIT_DEBUGGING,
  INIT_RECEIVER(CognitionToMotion),
  INIT_SENDER(MotionToCognition),
  moduleManager("Motion")
{
  theDebugReceiver.setSize(200000);
  theDebugSender.setSize(20000);

  theMotionToCognitionSender.moduleManager = theCognitionToMotionReceiver.moduleManager = &moduleManager;

  if(SystemCall::getMode() == SystemCall::physicalRobot)
    setPriority(50);
}

void Motion::init()
{
  moduleManager.load();
  BH_TRACE_INIT("Motion");
}

void Motion::terminate()
{
  moduleManager.destroy();
  Process::terminate();
}

bool Motion::main()
{
  // there has been no new package from Cognition in more than 500ms and thus we let the robot stand.
  if(SystemCall::getTimeSince(theCognitionToMotionReceiver.timeStamp) > 500)
    MotionSelector::stand();

  // required to detect whether any messages are sent in this frame
  int streamSize = theDebugSender.getStreamedSize();

  if(MotionLogDataProvider::isFrameDataComplete() && NaoProvider::isFrameDataComplete())
  {
    STOP_TIME_ON_REQUEST_WITH_PLOT("Motion", moduleManager.execute(););
    NaoProvider::finishFrame();

    DEBUG_RESPONSE("automated requests:DrawingManager", OUTPUT(idDrawingManager, bin, Global::getDrawingManager()););
    DEBUG_RESPONSE("automated requests:DrawingManager3D", OUTPUT(idDrawingManager3D, bin, Global::getDrawingManager3D()););
    DEBUG_RESPONSE("automated requests:StreamSpecification", OUTPUT(idStreamSpecification, bin, Global::getStreamHandler()););

    theMotionToCognitionSender.timeStamp = SystemCall::getCurrentSystemTime();
    theMotionToCognitionSender.send();

    if(theDebugSender.getStreamedSize() != streamSize)
    {
      // messages were sent in this frame -> send process finished
      OUTPUT(idProcessFinished, bin, 'm');
    }
    theDebugSender.send();
  }

  NaoProvider::waitForFrameData();
  return SystemCall::getMode() != SystemCall::physicalRobot;
}

bool Motion::handleMessage(InMessage& message)
{
  switch(message.getMessageID())
  {
  case idModuleRequest:
  {
    unsigned timeStamp;
    message.bin >> timeStamp;
    moduleManager.update(message.bin, timeStamp);
    return true;
  }

  default:
    return MotionLogDataProvider::handleMessage(message) ||
           SpecialActions::handleMessage(message) ||
           WalkingEngine::handleMessage(message) ||
           Process::handleMessage(message);
  }
}

MAKE_PROCESS(Motion);
