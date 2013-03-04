/**
* @file Processes/Cognition.cpp
* Implementation of a class that represents a process that receives data from the robot at about 30 Hz.
*/

#include "Cognition.h"
#include "Modules/Infrastructure/CameraProvider.h"
#include "Modules/Infrastructure/ExpCameraProvider.h"
#include "Modules/Configuration/CognitionConfigurationDataProvider.h"
#include "Modules/Infrastructure/CognitionLogDataProvider.h"
#include "Modules/Infrastructure/TeamDataProvider.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Platform/BHAssert.h"
#include "Tools/Settings.h"
#include "Tools/Team.h"

Cognition::Cognition() :
  INIT_DEBUGGING,
  INIT_RECEIVER(MotionToCognition),
  INIT_SENDER(CognitionToMotion),
  INIT_TEAM_COMM,
  moduleManager("Cognition")
{
  theDebugSender.setSize(10000000);
  theDebugReceiver.setSize(1400000);
  theTeamSender.setSize(1384); // 1 package without timestamp, size, localId, and message queue header
  theTeamReceiver.setSize(4 * 1450); // >= 4 packages
  theCognitionToMotionSender.moduleManager = theMotionToCognitionReceiver.moduleManager = &moduleManager;
  if(SystemCall::getMode() == SystemCall::physicalRobot)
    setPriority(1);
}

void Cognition::init()
{
  Global::theTeamOut = &theTeamSender.out;
  START_TEAM_COMM;
  moduleManager.load();
  BH_TRACE_INIT("Cognition");
}

void Cognition::terminate()
{
  moduleManager.destroy();
  Process::terminate();
}

bool Cognition::main()
{
  // required to detect whether any messages are sent in this frame
  int streamSize = theDebugSender.getStreamedSize();

  // read from team comm udp socket
  RECEIVE_TEAM_COMM;

  if(CognitionLogDataProvider::isFrameDataComplete() && CameraProvider::isFrameDataComplete() && ExpCameraProvider::isFrameDataComplete())
  {
    // There must not be any TEAM_OUTPUT before this in each frame.
    BH_TRACE_MSG("before TeamDataProvider");
    TeamDataProvider::handleMessages(theTeamReceiver);

    // Reset coordinate system for debug field drawing
    DECLARE_DEBUG_DRAWING("origin:Reset", "drawingOnField"); // Set the origin to the (0,0,0)
    ORIGIN("origin:Reset", 0.0f, 0.0f, 0.0f);

    STOP_TIME_ON_REQUEST_WITH_PLOT("Cognition", moduleManager.execute(););

    DEBUG_RESPONSE("process:Cognition:jointDelay",
    {
      if(&Blackboard::theInstance->theFrameInfo && &Blackboard::theInstance->theFilteredJointData)
      {
        OUTPUT(idText, text, Blackboard::theInstance->theFrameInfo.getTimeSince(Blackboard::theInstance->theFilteredJointData.timeStamp));
      }
    });

    DEBUG_RESPONSE("automated requests:DrawingManager", OUTPUT(idDrawingManager, bin, Global::getDrawingManager()););
    DEBUG_RESPONSE("automated requests:DrawingManager3D", OUTPUT(idDrawingManager3D, bin, Global::getDrawingManager3D()););
    DEBUG_RESPONSE("automated requests:StreamSpecification", OUTPUT(idStreamSpecification, bin, Global::getStreamHandler()););

    theCognitionToMotionSender.timeStamp = SystemCall::getCurrentSystemTime();
    BH_TRACE_MSG("before cognition2Motion.send");
    theCognitionToMotionSender.send();

    // Simulator polls for color table and will block if non is available. So make one available by default.
    if(!&Blackboard::theInstance->theColorTable64)
    {
      DEBUG_RESPONSE("representation:ColorTable64",
      {
        ColorTable64* ct = new ColorTable64;
        OUTPUT(idColorTable64, bin, *ct);
        delete ct;
      });
    }

    BH_TRACE_MSG("before SEND_TEAM_COMM");
    if(!theTeamSender.isEmpty())
    {
      if(&Blackboard::theInstance->theTeamMateData && Blackboard::theInstance->theTeamMateData.sendThisFrame)
      {
        SEND_TEAM_COMM;
      }
      theTeamSender.clear(); // team messages are purged even when not sent.
    }

    if(theDebugSender.getStreamedSize() != streamSize)
    {
      // messages were sent in this frame -> send process finished
      OUTPUT(idProcessFinished, bin, 'c');
    }
    BH_TRACE_MSG("theDebugSender.send()");
    theDebugSender.send();
  }
#ifndef RELEASE
  else if(Global::getDebugRequestTable().poll)
    --Global::getDebugRequestTable().pollCounter;
#endif

  if(SystemCall::getMode() == SystemCall::physicalRobot)
    setPriority(10);
  BH_TRACE_MSG("before waitForFrameData");
  ExpCameraProvider::waitForFrameData();
  if(SystemCall::getMode() == SystemCall::physicalRobot)
    setPriority(1);

  return SystemCall::getMode() != SystemCall::physicalRobot;
}

bool Cognition::handleMessage(InMessage& message)
{
  BH_TRACE_MSG("before Cognition:handleMessage");
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
    return CognitionLogDataProvider::handleMessage(message) ||
           CognitionConfigurationDataProvider::handleMessage(message) ||
           Process::handleMessage(message);
  }
  BH_TRACE_MSG("after Cognition:handleMessage");
}

MAKE_PROCESS(Cognition);
