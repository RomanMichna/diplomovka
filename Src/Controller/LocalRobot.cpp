/**
* @file Controller/LocalRobot.cpp
*
* Implementation of LocalRobot.
*
* @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
* @author <A href="mailto:kspiess@tzi.de">Kai Spiess</A>
*/

#include "LocalRobot.h"
#include "Controller/ConsoleRoboCupCtrl.h"

unsigned int LocalRobot::globalNextImageTimeStamp = 0;

LocalRobot::LocalRobot()
  : RobotConsole(theDebugReceiver, theDebugSender),
    theDebugReceiver(this, "Receiver.MessageQueue.O"),
    theDebugSender(this, "Sender.MessageQueue.S"),
    image(false),
    nextImageTimeStamp(0),
    imageLastTimeStampSent(0),
    jointLastTimeStampSent(0),
    updatedSignal(1),
    puppet(0)
{
  globalNextImageTimeStamp = 0;
  mode = ((ConsoleRoboCupCtrl*)RoboCupCtrl::controller)->getMode();
  addViews();

  if(mode == SystemCall::logfileReplay)
  {
    logFile = ((ConsoleRoboCupCtrl*)RoboCupCtrl::controller)->getLogFile();
    logPlayer.open(logFile.c_str());
    logPlayer.play();
    puppet = (SimRobotCore2::Body*)RoboCupCtrl::application->resolveObject("RoboCup.puppets." + robotName, SimRobotCore2::body);
    if(puppet)
      oracle.init(puppet);
  }
  else if(mode == SystemCall::simulatedRobot)
  {
    SimRobotCore2::Body* robot = (SimRobotCore2::Body*)RoboCupCtrl::application->resolveObject(RoboCupCtrl::getRobotFullName(), SimRobotCore2::body);
    ASSERT(robot);
    oracle.init(robot);
    ctrl->gameController.registerOracle(robotName[5].toAscii() - '1', oracle);
  }
}

bool LocalRobot::main()
{
  if(updateSignal.tryWait())
  {
    {
      // Only one thread can access *this now.
      SYNC;

      if(mode == SystemCall::simulatedRobot)
      {
        if(jointLastTimeStampSent != jointData.timeStamp)
        {
          debugOut.out.bin << 'm';
          debugOut.out.finishMessage(idProcessBegin);
          debugOut.out.bin << jointData;
          debugOut.out.finishMessage(idJointData);
          debugOut.out.bin << sensorData;
          debugOut.out.finishMessage(idSensorData);
          debugOut.out.bin << odometryData;
          debugOut.out.finishMessage(idGroundTruthOdometryData);
          debugOut.out.bin << orientationData;
          debugOut.out.finishMessage(idGroundTruthOrientationData);
          debugOut.out.bin << 'm';
          debugOut.out.finishMessage(idProcessFinished);
          jointLastTimeStampSent = jointData.timeStamp;
        }

        if(imageLastTimeStampSent != image.timeStamp)
        {
          debugOut.out.bin << 'c';
          debugOut.out.finishMessage(idProcessBegin);
          debugOut.out.bin << image;
          debugOut.out.finishMessage(idImage);
          debugOut.out.bin << robotPose;
          debugOut.out.finishMessage(idGroundTruthRobotPose);
          debugOut.out.bin << ballModel;
          debugOut.out.finishMessage(idGroundTruthBallModel);
          debugOut.out.bin << robotsModel;
          debugOut.out.finishMessage(idGroundTruthRobotsModel);
          debugOut.out.bin << imageInfo;
          debugOut.out.finishMessage(idImageInfo);
          ctrl->gameController.writeGameInfo(debugOut.out.bin);
          debugOut.out.finishMessage(idGameInfo);
          int robot = robotName[5].toAscii() - '1';
          ctrl->gameController.writeOwnTeamInfo(robot, debugOut.out.bin);
          debugOut.out.finishMessage(idOwnTeamInfo);
          ctrl->gameController.writeOpponentTeamInfo(robot, debugOut.out.bin);
          debugOut.out.finishMessage(idOpponentTeamInfo);
          ctrl->gameController.writeRobotInfo(robot, debugOut.out.bin);
          debugOut.out.finishMessage(idRobotInfo);
          debugOut.out.bin << 'c';
          debugOut.out.finishMessage(idProcessFinished);
          imageLastTimeStampSent = image.timeStamp;
        }
      }
      theDebugSender.send(true);
    }

    updatedSignal.post();
  }
  return true;
}

void LocalRobot::update()
{
  RobotConsole::update();

  updatedSignal.wait();

  // Only one thread can access *this now.
  {
    SYNC;

    if(mode == SystemCall::logfileReplay)
    {
      bool realtime = (ctrl && ctrl->isRealtime());
      if(logAcknowledged || realtime)
      {
        if(logPlayer.replay(realtime))
          logAcknowledged = false;
      }
      if(puppet)
        oracle.getAndSetJointData((const JointRequest&) RobotConsole::jointData, jointData);
    }
    if(mode == SystemCall::simulatedRobot || puppet)
    {
      if(moveOp != noMove)
      {
        if(moveOp == moveBoth)
          oracle.moveRobot(movePos, moveRot * (pi / 180), true);
        else if(moveOp == movePosition)
          oracle.moveRobot(movePos, Vector3<>(), false);
        else if(moveOp == moveBallPosition)
          oracle.moveBall(movePos);
        moveOp = noMove;
      }
    }
    if(mode == SystemCall::simulatedRobot)
    {
      unsigned int now = SystemCall::getCurrentSystemTime();
      if(now >= nextImageTimeStamp)
      {
        unsigned int newNextImageTimeStamp = globalNextImageTimeStamp;
        if(newNextImageTimeStamp == nextImageTimeStamp)
        {
          int imageDelay = calculateImageFps ? (1000 / calculateImageFps) : 33;
          if(imageRequest.requestedCamera == ImageInfo::toggleCameras)
            imageDelay /= 2;
          int duration = now - globalNextImageTimeStamp;
          globalNextImageTimeStamp = duration >= imageDelay ? now + imageDelay : globalNextImageTimeStamp + imageDelay;
          newNextImageTimeStamp = globalNextImageTimeStamp;
        }
        nextImageTimeStamp = newNextImageTimeStamp;

        if(calculateImage)
          oracle.getImage(image);
        else
          image.timeStamp = now;
        oracle.getRobotPose(robotPose);
        robotPose.timestamp = now;
        oracle.getBallModel(robotPose, ballModel);
      }
      else
        oracle.getRobotPose(robotPose);

      oracle.getOdometryData(robotPose, odometryData);
      oracle.getSensorData(sensorData);
      oracle.getOrientationData(sensorData, orientationData);
      oracle.getAndSetJointData(jointRequest, jointData);
      imageInfo = theImageInfo;
      oracle.getAndSetCamera(imageRequest, theImageInfo);
    }

    std::string statusText;
    if(mode == SystemCall::logfileReplay)
    {
      statusText = std::string("replaying ") + logFile + " ";
      if(logPlayer.currentFrameNumber != -1)
      {
        char buf[33];
        sprintf(buf, "%u", logPlayer.currentFrameNumber + 1);
        statusText += buf;
      }
      else
        statusText += "finished";
    }

    if(mode != SystemCall::logfileReplay && logPlayer.numberOfFrames != 0)
    {
      if(statusText != "")
        statusText += ", ";
      statusText += std::string("recorded ");
      char buf[33];
      sprintf(buf, "%u", logPlayer.numberOfFrames);
      statusText += buf;
    }

    if(pollingFor)
    {
      statusText += statusText != "" ? ", polling for " : "polling for ";
      statusText += pollingFor;
    }

    if(!statusText.empty())
      ((ConsoleRoboCupCtrl*)ConsoleRoboCupCtrl::controller)->printStatusText((robotName + ": " + statusText.c_str()).toAscii().constData());
  }

  updateSignal.post();
  trigger(); // invoke a call of main()
}
