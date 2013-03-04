/**
* @file Controller/RobotConsole.cpp
*
* Implementation of RobotConsole.
*
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#include <algorithm>
#include <fstream>
#include <strstream>

#include "Tools/Settings.h"
#include "Tools/ImageProcessing/ColorModelConversions.h"
#include "Tools/Debugging/QueueFillRequest.h"
#include "URC/MofCompiler.h"
#include "RobotConsole.h"
#include "ConsoleRoboCupCtrl.h"
#include "Views/ImageView.h"
#include "Views/FieldView.h"
#include "Views/TimeView.h"
#include "Views/SensorView.h"
#include "Views/ModuleView.h"
#include "Views/ColorTableView.h"
#include "Views/ColorSpaceView.h"
#include "Views/PlotView.h"
#include "Views/JointView.h"
#include "Views/StateMachineBehaviorView.h"
#include "Views/ViewBike/ViewBike.h"
#include "Views/RepresentationView/RepresentationView.h"
#include "Representations/Perception/JPEGImage.h"
#include "TeamRobot.h"
#include "Platform/File.h"

#define PREREQUISITE(p) pollingFor = #p; if(!poll(p)) return false;

PROCESS_WIDE_STORAGE(StreamHandler::EnumSpecification::const_iterator) RobotConsole::enumSpec;

bool RobotConsole::Printer::handleMessage(InMessage& message)
{
  ASSERT(message.getMessageID() == idText);
  ctrl->printLn(message.text.readAll());
  return true;
}

bool RobotConsole::calculateImage = true;
unsigned int RobotConsole::calculateImageFps = 0;

RobotConsole::RobotConsole(MessageQueue& in, MessageQueue& out)
  : Process(in, out),
    printMessages(true),
    handleMessages(true),
    logAcknowledged(true),
    destructed(false),
    logPlayer(out),
    debugOut(out),
    pollingFor(0),
    moveOp(noMove),
    freePartOfOpponentGoalModelReceived(0),
    obstacleModelReceived(0),
    robotsModelReceived(0),
    robotPoseReceived(0),
    sslVisionDataReceived(0),
    ballModelReceived(0),
    goalPerceptReceived(0),
    robotHealthReceived(0),
    motionRequestReceived(0),
    isPenalizedReceived(0),
    hasGroundContactReceived(0),
    isUprightReceived(0),
    acceptedCamera(ImageInfo::lowerCamera),
    repViewWriter((ConsoleRoboCupCtrl*) RoboCupCtrl::controller, &representationViews),
    logMessages(0),
    background(0.5f, 0.5f, 0.5f),
    colorTableSendDelay(1000),
    updateCompletion(false),
    directMode(false),
    logImagesAsJPEGs(false),
    joystickTrace(false),
    joystickLastTime(0),
    processIdentifier(0),
    maxPlotSize(0),
    bikeView(false),
    imageSaveNumber(0),
    cameraCalibratorHandler(this)
{
  // this is a hack: call global functions to get parameters
  ctrl = (ConsoleRoboCupCtrl*) RoboCupCtrl::controller;
  robotFullName = RoboCupCtrl::getRobotFullName();
  robotName = robotFullName.mid(robotFullName.lastIndexOf('.') + 1);
  lastColorTableSent = colorTableHandler.timeStamp;
  for(int i = 0; i < numOfMessageIDs; ++i)
  {
    waitingFor[i] = 0;
    polled[i] = false;
  }
  for(int i = 0; i < Joystick::numOfAxes; ++i)
  {
    joystickAxisMaxSpeeds[i] = joystickAxisThresholds[i] = joystickAxisCenters[i] = 0.f;
    joystickAxisMappings[i] = 0;
  }
  logPlayer.setSize(1 << 30); // max. 1 GB
}

RobotConsole::~RobotConsole()
{
  SYNC;
  streamHandler.clear();
  if(logMessages)
    delete [] logMessages;
  for(DebugDataInfos::iterator i = debugDataInfos.begin(); i != debugDataInfos.end(); ++i)
    delete i->second.second;
  destructed = true;
}

void RobotConsole::init()
{
  if(mode != SystemCall::teamRobot)
  {
    joystick.init();
    poll(idColorTable64);
  }
}

void RobotConsole::addViews()
{
  SimRobot::Object* category = ctrl->addCategory(robotName);

  ctrl->addView(new TimeView(robotName + ".timing", *this, timeInfo), category);
  ctrl->addView(new SensorView(robotName + ".sensorData", *this, sensorData, filteredSensorData), category);

  if(mode != SystemCall::teamRobot)
  {
    ctrl->addView(new JointView(robotName + ".jointData", *this, jointData, sensorData, jointRequest), category);
    ctrl->addView(new StateMachineBehaviorView(robotName + ".behavior", *this, stateMachineBehaviorInfo), category);
    SimRobot::Object* modulesCategory = ctrl->addCategory("modules", category);
    SimRobot::Object* cognitionCategory = ctrl->addCategory("cognition", modulesCategory);
    ctrl->addView(new ModuleView(robotName + ".modules.cognition.all", *this, 'c', ""), cognitionCategory);
    ctrl->addView(new ModuleView(robotName + ".modules.cognition.behaviorControl", *this, 'c', "Behavior Control"), cognitionCategory);
    ctrl->addView(new ModuleView(robotName + ".modules.cognition.infrastructure", *this, 'c', "Infrastructure"), cognitionCategory);
    ctrl->addView(new ModuleView(robotName + ".modules.cognition.modeling", *this, 'c', "Modeling"), cognitionCategory);
    ctrl->addView(new ModuleView(robotName + ".modules.cognition.perception", *this, 'c', "Perception"), cognitionCategory);
    SimRobot::Object* motionCategory = ctrl->addCategory("motion", modulesCategory);
    ctrl->addView(new ModuleView(robotName + ".modules.motion.all", *this, 'm', ""), motionCategory);
    ctrl->addView(new ModuleView(robotName + ".modules.motion.infrastructure", *this, 'm', "Infrastructure"), motionCategory);
    ctrl->addView(new ModuleView(robotName + ".modules.motion.motionControl", *this, 'm', "Motion Control"), motionCategory);
    ctrl->addView(new ModuleView(robotName + ".modules.motion.sensing", *this, 'm', "Sensing"), motionCategory);
    SimRobot::Object* colorSpaceCategory = ctrl->addCategory("colorSpace", category);
    ctrl->addView(new ColorTableView(robotName + ".colorSpace.colorTable", *this, colorTableHandler, background), colorSpaceCategory);
  }
}

void RobotConsole::addColorSpaceViews(const std::string& id, const std::string& name, bool user)
{
  SimRobot::Object* colorSpaceCategory = ctrl->application->resolveObject(robotName + ".colorSpace");
  SimRobot::Object* nameCategory = ctrl->addCategory(name.c_str(), colorSpaceCategory);

  for(int cm = 0; cm < ColorSpaceView::numOfColorModels - (user ? 0 : 1); ++cm)
  {
    SimRobot::Object* modelCategory = ctrl->addCategory(ColorSpaceView::getName(ColorSpaceView::ColorModel(cm)), nameCategory);
    const QString& modelCategoryName = modelCategory->getFullName();

    if(ColorSpaceView::ColorModel(cm) == ColorSpaceView::user)
      for(int channel = 1; channel < 4; ++channel)
        ctrl->addView(new ColorSpaceView(
                        modelCategoryName + "." +  ColorSpaceView::getChannelNameForColorModel(ColorSpaceView::ColorModel(cm), channel),
                        *this, id, ColorSpaceView::YCbCr, channel + 3, background), modelCategory);
    else
      for(int channel = 0; channel < 4; ++channel)
        ctrl->addView(new ColorSpaceView(
                        modelCategoryName + "." + ColorSpaceView::getChannelNameForColorModel(ColorSpaceView::ColorModel(cm), channel),
                        *this, id, ColorSpaceView::ColorModel(cm), channel + (cm == ColorSpaceView::YCbCr && channel ? 1 : 0), background), modelCategory);
  }
}

bool RobotConsole::handleMessage(InMessage& message)
{
  if(!handleMessages)
    return true;

  {
    // Only one thread can access *this now.
    SYNC;
    if(destructed) // if object is already destructed, abort here
      return true; // avoid further processing of this message

    if(message.getMessageID() < numOfDataMessageIDs)
    {
      if(logImagesAsJPEGs && message.getMessageID() == idImage)
      {
        Image image;
        message.bin >> image;
        MessageQueue queue;
        queue.out.bin << JPEGImage(image);
        queue.out.finishMessage(idJPEGImage);
        logPlayer.handleMessage(queue.in);
      }
      else
        logPlayer.handleMessage(message);
      message.resetReadPosition();
    }

    switch(message.getMessageID())
    {
    case idText:
    {
      std::string buffer(message.text.readAll());
      if(printMessages)
        ctrl->printLn(buffer);
      if(logMessages)
        *logMessages << buffer << std::endl;
      return true;
    }
    case idConsole:
      commands.push_back(message.text.readAll());
      return true;
    case idOdometryData:
      message.bin >> odometryData;
      return true;
    case idImage:
      if(!incompleteImages["raw image"].image)
        incompleteImages["raw image"].image = new Image(false);
      message.bin >> *incompleteImages["raw image"].image;
      return true;
    case idImageOther:
      if(!incompleteImages["raw imageOther"].image)
        incompleteImages["raw imageOther"].image = new Image(false);
      message.bin >> *incompleteImages["raw imageOther"].image;
      return true;
    case idJPEGImage:
    {
      if(!incompleteImages["raw image"].image)
        incompleteImages["raw image"].image = new Image(false);
      JPEGImage jpi;
      message.bin >> jpi;
      jpi.toImage(*incompleteImages["raw image"].image);
      return true;
    }
    case idJPEGImageOther:
    {
      if(!incompleteImages["raw imageOther"].image)
        incompleteImages["raw imageOther"].image = new Image(false);
      JPEGImage jpi;
      message.bin >> jpi;
      jpi.toImage(*incompleteImages["raw imageOther"].image);
      return true;
    }
    case idDebugImage:
    {
      std::string id;
      message.bin >> id;
      if(!incompleteImages[id].image)
        incompleteImages[id].image = new Image(false);
      message.bin >> *incompleteImages[id].image;
      incompleteImages[id].image->timeStamp = SystemCall::getCurrentSystemTime();
      break;
    }
    case idDebugJPEGImage:
    {
      std::string id;
      message.bin >> id;
      if(!incompleteImages[id].image)
        incompleteImages[id].image = new Image(false);
      JPEGImage jpi;
      message.bin >> jpi;
      jpi.toImage(*incompleteImages[id].image);
      incompleteImages[id].image->timeStamp = SystemCall::getCurrentSystemTime();
      break;
    }
    case idSensorData:
    {
      message.bin >> sensorData;
      return true;
    }
    case idFilteredSensorData:
    {
      message.bin >> filteredSensorData;
      return true;
    }
    case idJointData:
    {
      message.bin >> jointData;
      return true;
    }
    case idJointRequest:
    {
      message.bin >> jointRequest;
      if(Global::getSettings().model == Settings::nao)
        jointRequest.angles[JointData::RHipYawPitch] = jointRequest.angles[JointData::LHipYawPitch];
      return true;
    }
    case idLEDRequest:
    {
      message.bin >> ledRequest;
      return true;
    }
    case idDebugDrawing:
    {
      if(waitingFor[idDrawingManager]) // drawing manager not up-to-date
        return true; // skip drawing, may not be known anyway

      char shapeType,
           id;
      message.bin >> shapeType >> id;
      const char* name = drawingManager.getDrawingName(id); // const char* is required here
      std::string type = drawingManager.getDrawingType(name);

      if(type == "drawingOnImage")
        incompleteImageDrawings[name].addShapeFromQueue(message, (::Drawings::ShapeType)shapeType, '?');
      else if(type == "drawingOnField")
        incompleteFieldDrawings[name].addShapeFromQueue(message, (::Drawings::ShapeType)shapeType, '?');
      return true;
    }
    case idDebugDrawing3D:
    {
      if(!waitingFor[idDrawingManager3D])
      {
        char shapeType,
             id;
        message.bin >> shapeType >> id;
        incompleteDrawings3D[drawingManager3D.getDrawingName(id)].addShapeFromQueue(message, (::Drawings3D::ShapeType)shapeType, processIdentifier);
      }
      return true;
    }
    case idPlot:
    {
      std::string id;
      float value;
      message.bin >> id >> value;
      Plot& plot = plots[ctrl->translate(id)];
      plot.points.push_back(value);
      while(plot.points.size() > maxPlotSize)
        plot.points.pop_front();
      plot.timeStamp = SystemCall::getCurrentSystemTime();
      return true;
    }
    case idProcessBegin:
    {
      message.bin >> processIdentifier;
      drawingManager.setProcess(processIdentifier);
      drawingManager3D.setProcess(processIdentifier);
      return true;
    }
    case idProcessFinished:
    {
      char c;
      message.bin >> c;
      ASSERT(processIdentifier == c);
      bool useData = processIdentifier != c || acceptedCamera == ImageInfo::bothCameras || acceptedCamera == imageInfo.camera;

      if(useData)
      {
        // Delete all images received earlier from the current process
        for(Images::iterator i = images.begin(), next; i != images.end(); i = next)
        {
          next = i;
          ++next;
          if(i->second.processIdentifier == processIdentifier)
            images.erase(i);
        }

        // Add all images received now from the current process
        for(Images::iterator i = incompleteImages.begin(); i != incompleteImages.end(); ++i)
        {
          ImagePtr& imagePtr = images[i->first];
          imagePtr.image = i->second.image;
          imagePtr.processIdentifier = processIdentifier;
          i->second.image = 0;
        }

        // Delete all image drawings received earlier from the current process
        for(Drawings::iterator i = imageDrawings.begin(), next; i != imageDrawings.end(); i = next)
        {
          next = i;
          ++next;
          if(i->second.processIdentifier == processIdentifier)
            imageDrawings.erase(i);
        }

        // Add all image drawings received now from the current process
        for(Drawings::const_iterator i = incompleteImageDrawings.begin(); i != incompleteImageDrawings.end(); ++i)
        {
          DebugDrawing& debugDrawing = imageDrawings[i->first];
          debugDrawing = i->second;
          debugDrawing.processIdentifier = processIdentifier;
        }

        // Delete all field drawings received earlier from the current process
        for(Drawings::iterator i = fieldDrawings.begin(), next; i != fieldDrawings.end(); i = next)
        {
          next = i;
          ++next;
          if(i->second.processIdentifier == processIdentifier)
            fieldDrawings.erase(i);
        }

        // Add all field drawings received now from the current process
        for(Drawings::const_iterator i = incompleteFieldDrawings.begin(); i != incompleteFieldDrawings.end(); ++i)
        {
          DebugDrawing& debugDrawing = fieldDrawings[i->first];
          debugDrawing = i->second;
          debugDrawing.processIdentifier = processIdentifier;
        }

        // reset all 3d drawings from current process
        for(Drawings3D::iterator i = drawings3D.begin(), end = drawings3D.end(); i != end; ++i)
        {
          DebugDrawing3D& debugDrawing3D = i->second;
          if(debugDrawing3D.processIdentifier == processIdentifier)
            debugDrawing3D.reset();
        }
      }

      // 3D Drawings
      if(!waitingFor[idDrawingManager3D])
        for(Drawings3D::iterator i = incompleteDrawings3D.begin(); i != incompleteDrawings3D.end();)
        {
          Drawings3D::iterator j = i++;
          if(j->second.processIdentifier == processIdentifier)
          {
            if(useData)
            {
              DebugDrawing3D& debugDrawing3D = drawings3D[j->first];
              bool drawn = debugDrawing3D.drawn;
              bool flip = debugDrawing3D.flip;
              j->second.robotConsole = this;
              debugDrawing3D = j->second;
              debugDrawing3D.drawn = drawn;
              debugDrawing3D.flip = flip;
              if(!drawn)
              {
                std::string type = drawingManager3D.getDrawingType(drawingManager3D.getString(j->first));
                if(type != "unknown")
                {
                  QVector<QString> parts;
                  parts.append(robotName);
                  if(type == "field")
                  {
                    debugDrawing3D.flip = robotFullName[robotFullName.length() - 1] < '5';
                    parts[0] = "RoboCup";
                  }
                  else if(type == "robot")
                    parts.append("origin");
                  else
                    parts.append(type.c_str());
                  SYNC_WITH(*ctrl);
                  SimRobotCore2::PhysicalObject* object = (SimRobotCore2::PhysicalObject*)ctrl->application->resolveObject(parts);
                  object->registerDrawing(debugDrawing3D);
                  debugDrawing3D.drawn = true;
                }
              }
            }
            incompleteDrawings3D.erase(j);
          }
        }

      incompleteImages.clear();
      incompleteImageDrawings.clear();
      incompleteFieldDrawings.clear();

      return true;
    }
    case idStateMachineBehaviorDebugSymbols:
    {
      return stateMachineBehaviorInfo.handleMessage(message);
    }
    case idStateMachineBehaviorDebugMessage:
    {
      return stateMachineBehaviorInfo.handleMessage(message);
    }
    case idStopwatch:
      return timeInfo.handleMessage(message);
    case idDebugResponse:
    {
      SYNC_WITH(*ctrl);
      std::string description;
      int enable;
      message.text >> description >> enable;
      if(description != "pollingFinished")
        debugRequestTable.addRequest(DebugRequest(description, enable != 0, true));
      else if(--waitingFor[idDebugResponse] <= 0)
      {
        ctrl->setDebugRequestTable(debugRequestTable);
        updateCompletion = true;
      }
      return true;
    }
    case idModuleTable:
    {
      SYNC_WITH(*ctrl);
      moduleInfo.handleMessage(message, processIdentifier);
      if(--waitingFor[idModuleTable] <= 0)
      {
        ctrl->setModuleInfo(moduleInfo);
        updateCompletion = true;
      }
      return true;
    }
    case idDrawingManager:
    {
      SYNC_WITH(*ctrl);
      message.bin >> drawingManager;
      if(--waitingFor[idDrawingManager] <= 0)
      {
        ctrl->setDrawingManager(drawingManager);
        updateCompletion = true;
      }
      return true;
    }
    case idDrawingManager3D:
    {
      SYNC_WITH(*ctrl);
      message.bin >> drawingManager3D;
      if(--waitingFor[idDrawingManager3D] <= 0)
      {
        ctrl->setDrawingManager3D(drawingManager3D);
        updateCompletion = true;
      }
      return true;
    }
    case idColorTable64:
      colorTableHandler.handleMessage(message);
      lastColorTableSent = colorTableHandler.timeStamp;
      waitingFor[idColorTable64] = 0;
      return true;
    case idStreamSpecification:
    {
      message.bin >> streamHandler;
      --waitingFor[idStreamSpecification];
      return true;
    }
    case idDebugDataResponse:
    {
      std::string name,
                  type;
      message.bin >> name >> type;
      if(debugDataInfos.find(name) == debugDataInfos.end())
        debugDataInfos[name] = DebugDataInfoPair(type, new MessageQueue);
      debugDataInfos[name].second->clear();
      message >> *debugDataInfos[name].second;
      const char* t = streamHandler.getString(type);
      //check if type info exists for the received data.
      if(streamHandler.basicTypeSpecification.find(t) == streamHandler.basicTypeSpecification.end() &&
         streamHandler.specification.find(t) == streamHandler.specification.end() &&
         streamHandler.enumSpecification.find(t) == streamHandler.enumSpecification.end())
      {
        if(waitingFor[idStreamSpecification] <= 0) //stream specification has not yet been polled, poll it
        {
          polled[idStreamSpecification] = false;
        }
      }
      else
      {
        //only forward the message to the representation viewer if up to date type info exists
        repViewWriter.handleMessage(message, type, name);
      }

      waitingFor[idDebugDataResponse] = 0;
      return true;
    }
    case idLogResponse:
      logAcknowledged = true;
      return true;
    case idTeamMateObstacleModel:
    {
      ObstacleModelCompressed theObstacleModelCompressed;
      message.bin >> theObstacleModelCompressed;
      obstacleModel = theObstacleModelCompressed.unpack();
      obstacleModelReceived = SystemCall::getCurrentSystemTime();
      return true;
    }
    case idTeamMateFreePartOfOpponentGoalModel:
      message.bin >> freePartOfOpponentGoalModel;
      freePartOfOpponentGoalModelReceived = SystemCall::getCurrentSystemTime();
      return true;
    case idTeamMateRobotsModel:
    {
      RobotsModelCompressed robotsModelCompressed;
      message.bin >> robotsModelCompressed;
      robotsModel = robotsModelCompressed.unpack();
      for(size_t i = 0; i < robotsModel.robots.size(); i++)
      {
        robotsModel.robots[i].timeStamp = ctrl->ntp.getRemoteTimeInLocalTime(robotsModel.robots[i].timeStamp);
      }
      robotsModelReceived = SystemCall::getCurrentSystemTime();
      return true;
    }
    case idTeamMateRobotPose:
    {
      RobotPoseCompressed robotPoseCompressed;
      message.bin >> robotPoseCompressed;
      robotPose = robotPoseCompressed.unpack();
      robotPoseReceived = SystemCall::getCurrentSystemTime();
      return true;
    }
    case idTeamMateSSLVisionData:
      message.bin >> sslVisionData;
      sslVisionDataReceived = SystemCall::getCurrentSystemTime();
      return true;
    case idTeamMateBallModel:
    {
      BallModelCompressed ballModelCompressed;
      message.bin >> ballModelCompressed;
      ballModel = ballModelCompressed.unpack();
      if(ballModel.timeWhenLastSeen)
        ballModel.timeWhenLastSeen = ctrl->ntp.getRemoteTimeInLocalTime(ballModel.timeWhenLastSeen);
      ballModelReceived = SystemCall::getCurrentSystemTime();
      return true;
    }
    case idTeamMateCombinedWorldModel:
      message.bin >> combinedWorldModel;
      combinedWorldModelReceived = SystemCall::getCurrentSystemTime();
      return true;
    case idTeamMateGoalPercept:
      message.bin >> goalPercept;
      if(goalPercept.timeWhenGoalPostLastSeen)
        goalPercept.timeWhenGoalPostLastSeen = ctrl->ntp.getRemoteTimeInLocalTime(goalPercept.timeWhenGoalPostLastSeen);
      if(goalPercept.timeWhenCompleteGoalLastSeen)
        goalPercept.timeWhenCompleteGoalLastSeen = ctrl->ntp.getRemoteTimeInLocalTime(goalPercept.timeWhenCompleteGoalLastSeen);
      goalPerceptReceived = SystemCall::getCurrentSystemTime();
      return true;
    case idTeamMateHasGroundContact:
      message.bin >> hasGroundContact;
      hasGroundContactReceived = SystemCall::getCurrentSystemTime();
      return true;
    case idTeamMateIsPenalized:
      message.bin >> isPenalized;
      isPenalizedReceived = SystemCall::getCurrentSystemTime();
      return true;
    case idTeamMateIsUpright:
      message.bin >> isUpright;
      isUprightReceived = SystemCall::getCurrentSystemTime();
      return true;
    case idLinePercept:
      message.bin >> linePercept;
      linePerceptReceived = SystemCall::getCurrentSystemTime();
      return true;
    case idTeamMateBehaviorData:
      message.bin >> behaviorData;
      return true;
    case idRobotHealth:
      message.bin >> robotHealth;
      robotHealthReceived = SystemCall::getCurrentSystemTime();
      return true;
    case idMotionRequest:
      message.bin >> motionRequest;
      motionRequestReceived = SystemCall::getCurrentSystemTime();
      return true;
    case idRobotname:
    {
      std::string buffer;
      message.bin >> buffer;
      Global::getSettings().robot = buffer;
      return true;
    }
    case idRobotDimensions:
    {
      message.bin >> robotDimensions;
      return true;
    }
    case idJointCalibration:
    {
      message.bin >> jointCalibration;
      return true;
    }
    case idImageRequest:
    {
      message.bin >> imageRequest;
      return true;
    }
    case idImageInfo:
    {
      message.bin >> imageInfo;
      return true;
    }
    default:
      return mode == SystemCall::teamRobot
             ? Process::handleMessage(message) : false;
    }
  }

  return false;
}

void RobotConsole::update()
{
  setGlobals(); // this is called in GUI thread -> set globals for this process
  handleJoystick();

  if(lastColorTableSent < colorTableHandler.timeStamp &&
     SystemCall::getTimeSince(lastColorTableSent) >= colorTableSendDelay)
  {
    {
      SYNC;
      colorTableHandler.send(debugOut.out);
    }
    lastColorTableSent = SystemCall::getCurrentSystemTime();
    triggerProcesses();
  }

  while(!lines.empty())
  {
    std::list<std::string> temp = lines;
    lines.clear();
    if(handleConsoleLine(temp.front()))
    {
      temp.pop_front();
      lines.splice(lines.end(), temp);
    }
    else
    {
      lines = temp;
      break;
    }
  }

  if(!commands.empty())
  {
    std::list<std::string> commands;
    {
      SYNC;
      commands.swap(this->commands);
    }
    for(std::list<std::string>::const_iterator iter = commands.begin(), end = commands.end(); iter != end; ++iter)
      ctrl->executeConsoleCommand(*iter, this);
  }

  pollForDirectMode();

  if(updateCompletion)
  {
    SYNC;
    ctrl->updateCommandCompletion();
    updateCompletion = false;
  }
}

void RobotConsole::handleConsole(const std::string& line)
{
  setGlobals(); // this is called in GUI thread -> set globals for this process
  lines.push_back(line);
  while(!lines.empty())
  {
    std::list<std::string> temp = lines;
    lines.clear();
    if(handleConsoleLine(temp.front()))
    {
      temp.pop_front();
      lines.splice(lines.end(), temp);
    }
    else
    {
      lines = temp;
      break;
    }
  }

  pollForDirectMode();
}

void RobotConsole::triggerProcesses()
{
  if(mode == SystemCall::logfileReplay)
  {
    SYNC;
    debugOut.out.bin << 'c';
    debugOut.out.finishMessage(idProcessBegin);
    debugOut.out.bin << 'c';
    debugOut.out.finishMessage(idProcessFinished);
    debugOut.out.bin << 'm';
    debugOut.out.finishMessage(idProcessBegin);
    debugOut.out.bin << 'm';
    debugOut.out.finishMessage(idProcessFinished);
  }
}

bool RobotConsole::poll(MessageID id)
{
  if(waitingFor[id] > 0)
  {
    // When in replay log file mode, force replay while polling to keep Cognition running
    triggerProcesses();
    return false;
  }
  else if(polled[id])
    return true;
  else
  {
    polled[id] = true;
    switch(id)
    {
    case idDebugResponse:
    {
      SYNC;
      debugOut.out.bin << DebugRequest("poll");
      debugOut.out.finishMessage(idDebugRequest);
      if(mode == SystemCall::teamRobot)
        waitingFor[id] = 1; // team robot module should answer
      else
        waitingFor[id] = 3; // Cognition + Motion + Debug will answer
      break;
    }
    case idModuleTable:
    {
      SYNC;
      moduleInfo.clear();
      debugOut.out.bin << DebugRequest("automated requests:ModuleTable", true, true);
      debugOut.out.finishMessage(idDebugRequest);
      waitingFor[id] = 2;  // Cognition + Motion will answer
      break;
    }
    case idStreamSpecification:
    {
      SYNC;
      streamHandler.clear();
      debugOut.out.bin << DebugRequest("automated requests:StreamSpecification", true, true);
      debugOut.out.finishMessage(idDebugRequest);
      if(mode == SystemCall::teamRobot)
        waitingFor[id] = 1; // team robot module should answer
      else
        waitingFor[id] = 2;  // Cognition + Motion will answer
      break;
    }
    case idDrawingManager:
    {
      SYNC;
      drawingManager.clear();
      debugOut.out.bin << DebugRequest("automated requests:DrawingManager", true, true);
      debugOut.out.finishMessage(idDebugRequest);
      if(mode == SystemCall::teamRobot)
        waitingFor[id] = 1; // team robot module should answer
      else
        waitingFor[id] = 2;  // Cognition + Motion will answer
      break;
    }
    case idDrawingManager3D:
    {
      SYNC;
      drawingManager3D.clear();
      debugOut.out.bin << DebugRequest("automated requests:DrawingManager3D", true, true);
      debugOut.out.finishMessage(idDebugRequest);
      if(mode == SystemCall::teamRobot)
        waitingFor[id] = 1; // team robot module should answer
      else
        waitingFor[id] = 2;  // Cognition + Motion will answer
      break;
    }
    case idColorTable64:
    {
      SYNC;
      debugOut.out.bin << DebugRequest("representation:ColorTable64", true, true);
      debugOut.out.finishMessage(idDebugRequest);
      waitingFor[id] = 1;  // Cognition will answer
      break;
    }
    default:
      break;
    }
    return false;
  }
}

void RobotConsole::pollForDirectMode()
{
  if(directMode)
  {
    poll(idDebugResponse);
    poll(idDrawingManager);
    poll(idDrawingManager3D);
    poll(idModuleTable);
  }
}

bool RobotConsole::handleConsoleLine(const std::string& line)
{
  InConfigMemory stream(line.c_str(), line.size());
  std::string command;
  stream >> command;
  bool result = false;
  if(command == "") // comment
    result = true;
  else if(command == "endOfStartScript")
  {
    directMode = true;
    result = true;
  }
  else if(command == "cls")
  {
    ctrl->printLn("_cls");
    result = true;
  }
  else if(command == "dr")
  {
    PREREQUISITE(idDebugResponse);
    PREREQUISITE(idDrawingManager3D);
    result = debugRequest(stream);
  }
  else if(command == "echo")
  {
    ctrl->echo(stream);
    result = true;
  }
  else if(command == "get")
  {
    PREREQUISITE(idDebugResponse);
    result = get(stream, true, true);
  }
  else if(command == "_get") // get, part 2
  {
    PREREQUISITE(idDebugDataResponse);
    PREREQUISITE(idStreamSpecification);
    result = get(stream, false, true);
  }
  else if(command == "_get2") // get, part 1 without printing
  {
    PREREQUISITE(idDebugResponse);
    result = get(stream, true, false);
  }
  else if(command == "_get3") // get, part 2 without printing
  {
    PREREQUISITE(idDebugDataResponse);
    PREREQUISITE(idStreamSpecification);
    result = get(stream, false, false);
  }
  else if(command == "jc")
    result = joystickCommand(stream);
  else if(command == "js")
    result = joystickSpeeds(stream);
  else if(command == "jm")
    result = joystickMaps(stream);
  else if(command == "log")
  {
    PREREQUISITE(idModuleTable);
    result = log(stream);
  }
  else if(command == "msg")
  {
    if(mode != SystemCall::teamRobot)
      PREREQUISITE(idColorTable64); // receive color table before processing "msg disable"
    result = msg(stream);
  }
  else if(command == "poll")
    result = repoll(stream);
  else if(command == "pr" && mode == SystemCall::simulatedRobot)
    result = ctrl->gameController.handleRobotConsole(robotName[5].toAscii() - '1', stream);
  else if(command == "set")
  {
    PREREQUISITE(idDebugResponse);
    result = set(stream);
  }
  else if(command == "_set") // set, part 2
  {
    PREREQUISITE(idDebugDataResponse);
    PREREQUISITE(idStreamSpecification);
    result = set(stream);
  }
  else if(command == "vf")
  {
    PREREQUISITE(idDebugResponse);
    PREREQUISITE(idDrawingManager);
    result = viewField(stream);
  }
  else if(command == "vfd")
  {
    PREREQUISITE(idDebugResponse);
    PREREQUISITE(idDrawingManager);
    result = viewDrawing(stream, fieldViews, "drawingOnField");
  }
  else if(command == "vr") //view representation part 1
  {
    PREREQUISITE(idDebugResponse);
    result = viewRepresentation(stream);
  }
  else if(command == "_vr") //view represenation part 2
  {
    //in part 1 a view has been created and a debug request has been fired.
    //Part 2 waits for the debug response and requests the stream specification afterwards.
    //The specification is needed to parse the data.
    PREREQUISITE(idDebugDataResponse);
    PREREQUISITE(idStreamSpecification);
    result = true;
  }
  else if(command == "vp")
  {
    PREREQUISITE(idDebugResponse);
    result = viewPlot(stream);
  }
  else if(command == "vpd")
  {
    PREREQUISITE(idDebugResponse);
    result = viewPlotDrawing(stream);
  }
  else if(mode == SystemCall::teamRobot) // stop
    result = false;
  else if(command == "ac")
    result = acceptCamera(stream);
  else if(command == "vid")
  {
    PREREQUISITE(idDebugResponse);
    PREREQUISITE(idDrawingManager);
    result = viewDrawing(stream, imageViews, "drawingOnImage");
  }
  else if(command == "vi")
  {
    PREREQUISITE(idDebugResponse);
    PREREQUISITE(idDrawingManager);
    result = viewImage(stream);
  }
  else if(command == "bike")
  {
    result = viewBike();
  }
  else if(command == "cameraCalibrator")
  {
    std::string view,
                choice;
    stream >> view >> choice;
    result = true;
    if(choice == "on")
    {
      cameraCalibratorHandler.setActive(view);
    }
    else if(choice == "off")
    {
      cameraCalibratorHandler.setInactive(view);
    }
    else
    {
      result = false;
    }
  }
  else if(command == "bc")
    result = backgroundColor(stream);
  else if(command == "ci")
    result = calcImage(stream);
  else if(command == "ct")
  {
    PREREQUISITE(idColorTable64);
    result = colorTable(stream);
  }
  else if(command == "mof")
    result = sendMof(stream);
  else if(command == "mr")
  {
    PREREQUISITE(idModuleTable);
    result = moduleRequest(stream);
  }
  else if(command == "mv")
    result = moveRobot(stream);
  else if(command == "mvb")
    result = moveBall(stream);
  else if(command == "qfr")
    result = queueFillRequest(stream);
  else if(command == "v3")
  {
    PREREQUISITE(idDebugResponse);
    PREREQUISITE(idDrawingManager);
    result = view3D(stream);
  }
  else if(command == "wek")
    result = sendWek(stream);
  else if(command == "save")
  {
    PREREQUISITE(idDebugResponse);
    result = saveRequest(stream, true);
  }
  else if(command == "_save")
  {
    PREREQUISITE(idDebugDataResponse);
    PREREQUISITE(idStreamSpecification);
    result = saveRequest(stream, false);
  }
  else if(command == "si")
  {
    result = saveImage(stream);
  }

  pollingFor = 0;
  if(!result)
  {
    if(directMode)
    {
      ctrl->printLn("Syntax Error");
    }
    else
    {
      ctrl->printLn((std::string("Syntax Error: ") + line).c_str());
    }
  }
  return true;
}

bool RobotConsole::msg(In& stream)
{
  std::string state;
  stream >> state;
  if(state == "off")
  {
    printMessages = false;
    return true;
  }
  else if(state == "on")
  {
    printMessages = true;
    return true;
  }
  else if(state == "log")
  {
    if(logMessages)
      delete logMessages;
    stream >> state;
    std::string name(state);
    if(name.size() == 0)
      return false;
    else
    {
      if((int) name.rfind('.') <= (int) name.find_last_of("\\/"))
        name = name + ".txt";
      char buf[FILENAME_MAX];
      if(name[0] != '/' && name[0] != '\\' && name[0] != '.' && (name[0] == 0 || name[1] != ':'))
        sprintf(buf, "%s/Config/Logs/", File::getBHDir());
      else
        buf[0] = 0;
      ASSERT(strlen(buf) + strlen(name.c_str()) < FILENAME_MAX);
      strcat(buf, name.c_str());
      logMessages = new std::fstream(buf, std::ios_base::out);
      return true;
    }
  }
  else if(state == "disable")
  {
    handleMessages = false;
    return true;
  }
  else if(state == "enable")
  {
    handleMessages = true;
    return true;
  }
  return false;
}

bool RobotConsole::backgroundColor(In& stream)
{
  stream >> background.x >> background.y >> background.z;
  background *= 0.01F;
  return true;
}

bool RobotConsole::calcImage(In& stream)
{
  std::string state;
  stream >> state;
  if(state == "off")
  {
    calculateImage = false;
    return true;
  }
  else if(state == "on" || state == "")
  {
    calculateImageFps = 0;
    calculateImage = true;
    return true;
  }
  else
  {
    for(unsigned i = 0; i < state.size(); ++i)
      if(!isdigit(state[i]))
        return false;
    calculateImageFps = atoi(state.c_str());
    calculateImage = true;
    return true;
  }
}

bool RobotConsole::colorTable(In& stream)
{
  std::string command;
  stream >> command;
  if(command == "off")
  {
    SYNC;
    colorTableHandler.active = false;
    return true;
  }
  else if(command == "on")
  {
    SYNC;
    colorTableHandler.active = true;
    return true;
  }
  else if(command == "reclassify")
  {
    SYNC;
    colorTableHandler.reclassify();
    return true;
  }
  else if(command == "training")
  {
    SYNC;
    stream >> command;
    if(command == "" || command == "on")
    {
      colorTableHandler.enableTraining();
      colorTableHandler.active = true;
    }
    else if(command == "off")
    {
      colorTableHandler.disableTraining();
      colorTableHandler.active = false;
    }
    else
      return false;
    return true;
  }
  else if(command == "smart")
  {
    stream >> command;
    SYNC;
    if(command == "off")
      colorTableHandler.smart = false;
    else
      colorTableHandler.smart = true;
    return true;
  }
  else if(command == "undo")
  {
    SYNC;
    colorTableHandler.undo();
  }
  else if(command == "load")
  {
    stream >> command;
    SYNC;
    if(!colorTableHandler.load(command))
      return false;
  }
  else if(command == "save")
  {
    stream >> command;
    if(command != "")
    {
      SYNC;
      colorTableHandler.save(command);
      return true;
    }
    else
      return false;
  }
  else if(command == "send")
  {
    stream >> command;
    if(command == "")
    {
      {
        SYNC;
        colorTableHandler.send(debugOut.out);
      }
      triggerProcesses();
      return true;
    }
    else if(command == "off")
    {
      colorTableSendDelay = 0xffffffff;
      triggerProcesses();
      return true;
    }
    else
      colorTableSendDelay = atoi(command.c_str());
  }
  else if(command == "sendAndWrite")
  {
    stream >> command;
    if(command == "")
    {
      {
        SYNC;
        colorTableHandler.sendAndWrite(debugOut.out);
      }
      triggerProcesses();
      return true;
    }
  }
  else if(command == "clear")
  {
    stream >> command;
    if(command == "")
      colorTableHandler.clear();
    else
    {
      int color;
      for(color = ColorClasses::none; color < ColorClasses::numOfColors; ++color)
        if(ctrl->translate(ColorClasses::getName((ColorClasses::Color) color)) == command)
        {
          SYNC;
          colorTableHandler.clear((ColorClasses::Color) color);
          break;
        }

      if(color == ColorClasses::numOfColors)
        return false;
    }
  }
  else if(command == "mask")
  {
    stream >> command;
    int color;
    for(color = ColorClasses::none; color < ColorClasses::numOfColors; ++color)
      if(ctrl->translate(ColorClasses::getName((ColorClasses::Color) color)) == command)
      {
        SYNC;
        colorTableHandler.maskColorClass = (ColorClasses::Color) color;
        break;
      }

    if(color == ColorClasses::numOfColors)
      return false;
  }
  else if(command == "shrink")
  {
    stream >> command;
    if(command == "")
    {
      SYNC;
      colorTableHandler.shrink();
    }
    else
    {
      int color;
      for(color = ColorClasses::none; color < ColorClasses::numOfColors; ++color)
        if(ctrl->translate(ColorClasses::getName((ColorClasses::Color) color)) == command)
        {
          SYNC;
          colorTableHandler.shrink((ColorClasses::Color) color);
          break;
        }

      if(color == ColorClasses::numOfColors)
        return false;
    }
  }
  else if(command == "grow")
  {
    stream >> command;
    if(command == "")
    {
      SYNC;
      colorTableHandler.grow();
    }
    else
    {
      int color;
      for(color = ColorClasses::none; color < ColorClasses::numOfColors; ++color)
        if(ctrl->translate(ColorClasses::getName((ColorClasses::Color) color)) == command)
        {
          SYNC;
          colorTableHandler.grow((ColorClasses::Color) color);
          break;
        }

      if(color == ColorClasses::numOfColors)
        return false;
    }
  }
  else if(command == "imageRadius")
  {
    SYNC;
    stream >> colorTableHandler.imageRadius;
    return true;
  }
  else if(command == "colorSpaceRadius")
  {
    SYNC;
    stream >> colorTableHandler.colorSpaceRadius;
    return true;
  }
  else if(command == "hash")
  {
    colorTableHandler.hash(ctrl);
    return true;
  }
  else if(command == "kd")
  {
    SYNC;
    stream >> command;
    if(command == "undo")
    {
      stream >> command;
      if(command == "" || command == "hand")
        colorTableHandler.undoLastHandData();
      else if(command == "auto")
        colorTableHandler.undoLastCalculation();
      else
        return false;
    }
    else if(command == "debug")
    {
      stream >> command;
      if(command == "" || command == "on")
        return colorTableHandler.kdDebug(true);
      else
        return colorTableHandler.kdDebug(false);
    }
    else if(command == "clear")
    {
      stream >> command;
      if(command == "")
      {
        colorTableHandler.resetKDAuto();
        colorTableHandler.resetKDHand();
        return true;
      }
      else
      {
        string base = command;
        stream >> command;
        if(command == "")
        {
          if(base == "hand")
            colorTableHandler.resetKDHand();
          else if(base == "auto")
            colorTableHandler.resetKDAuto();
          else
          {
            int color = ColorClasses::numOfColors;
            for(int i = ColorClasses::none; i < ColorClasses::numOfColors; ++i)
            {
              if(ctrl->translate(ColorClasses::getName((ColorClasses::Color) i)) == base)
              {
                color = i;
                break;
              }
            }

            if(color == ColorClasses::numOfColors)
              return false;

            return colorTableHandler.removeColor("all", (ColorClasses::Color) color);
          }
        }
        else
        {
          int color = ColorClasses::numOfColors;
          for(int i = ColorClasses::none; i < ColorClasses::numOfColors; ++i)
          {
            if(ctrl->translate(ColorClasses::getName((ColorClasses::Color) i)) == command)
            {
              color = i;
              break;
            }
          }

          if(color == ColorClasses::numOfColors)
            return false;

          return colorTableHandler.removeColor(base, (ColorClasses::Color) color);
        }
      }
    }
    else if(command == "save")
    {
      stream >> command;
      if(!command.empty())
        return colorTableHandler.saveAutoCtTraining(command);
      else
        return false;
    }
    else if(command == "load")
    {
      stream >> command;
      if(!command.empty())
        return colorTableHandler.loadAutoCtTraining(command);
      else
        return false;
    }
    else if(command == "replace")
    {

      stream >> command;
      int search = ColorClasses::numOfColors;
      for(int i = ColorClasses::none; i < ColorClasses::numOfColors; ++i)
      {
        if(ctrl->translate(ColorClasses::getName((ColorClasses::Color) i)) == command)
        {
          search = i;
          break;
        }
      }

      stream >> command;
      int replaceBy = ColorClasses::numOfColors;
      for(int k = ColorClasses::none; k < ColorClasses::numOfColors; ++k)
      {
        if(ctrl->translate(ColorClasses::getName((ColorClasses::Color) k)) == command)
        {
          replaceBy = k;
          break;
        }
      }

      if(search == ColorClasses::numOfColors ||
         replaceBy   == ColorClasses::numOfColors ||
         search == replaceBy)
        return false;

      colorTableHandler.replaceKDColors((ColorClasses::Color) search, (ColorClasses::Color) replaceBy);
      return true;
    }
    else if(command == "range")
    {
      stream >> command;
      colorTableHandler.setTreeRange(atoi(command.c_str()));
    }
    else if(command == "neighbors")
    {
      stream >> command;
      colorTableHandler.setTreeNeighbor(atoi(command.c_str()));
    }
    else
      return false;
    return true;
  }
  else if(command == "auto")
  {
    SYNC;
    stream >> command;
    if(command == "" || command == "on")
      colorTableHandler.setAutoCt(true);
    else if(command == "off")
      colorTableHandler.setAutoCt(false);
    else
      return false;
    return true;
  }
  else
  {
    int color;
    for(color = ColorClasses::none; color < ColorClasses::numOfColors; ++color)
      if(ctrl->translate(ColorClasses::getName((ColorClasses::Color) color)) == command)
      {
        stream >> command;
        SYNC;
        if(command == "" || command == "replace")
          colorTableHandler.replace = true;
        else if(command == "add")
          colorTableHandler.replace = false;
        else
        {
          int hMin, hMax, sMin, sMax, iMin, iMax;
          stream >> hMax >> sMin >> sMax >> iMin >> iMax;
          sscanf(command.c_str(), "%d", &hMin);
          if((hMin || command == "0") && hMin >= 0 && hMin < 256 && hMax >= 0 && hMax < 256 &&
             sMin >= 0 && sMin < 256 && sMax >= 0 && sMax < 256 && sMin <= sMax &&
             iMin >= 0 && iMin < 256 && iMax >= 0 && iMax < 256 && iMin <= iMax)
          {
            colorTableHandler.fillHSIRange((unsigned char) hMin, (unsigned char) hMax,
                                           (unsigned char) sMin, (unsigned char) sMax,
                                           (unsigned char) iMin, (unsigned char) iMax,
                                           (ColorClasses::Color) color);
            return true;
          }
          return false;
        }
        colorTableHandler.selectedColorClass = (ColorClasses::Color) color;
        return true;
      }

    if(color == ColorClasses::numOfColors)
      return false;
  }

  if(SystemCall::getTimeSince(lastColorTableSent) >= colorTableSendDelay)
  {
    {
      SYNC;
      colorTableHandler.send(debugOut.out);
    }
    lastColorTableSent = SystemCall::getCurrentSystemTime();
    triggerProcesses();
  }
  return true;
}

bool RobotConsole::debugRequest(In& stream)
{
  std::string debugRequestString,
              state;
  stream >> debugRequestString >> state;

  if(debugRequestString == "?")
  {
    for(int i = 0; i < debugRequestTable.currentNumberOfDebugRequests; ++i)
      ctrl->list(ctrl->translate(debugRequestTable.debugRequests[i].description).c_str(), state);
    ctrl->printLn("");
    return true;
  }
  else
  {
    if(debugRequestString == "off")
    {
      SYNC;
      debugOut.out.bin << DebugRequest("disableAll");
      debugOut.out.finishMessage(idDebugRequest);
      return true;
    }
    else
      for(int i = 0; i < debugRequestTable.currentNumberOfDebugRequests; ++i)
        if(ctrl->translate(debugRequestTable.debugRequests[i].description) == debugRequestString)
        {
          DebugRequest& d = debugRequestTable.debugRequests[i];
          if(state == "off")
          {
            d.enable = false;
            d.once = false;
          }
          else if(state == "on" || state == "")
          {
            d.enable = true;
            d.once = false;
          }
          else if(state == "once")
          {
            d.enable = true;
            d.once = true;
          }
          else
            return false;

          SYNC;
          debugOut.out.bin << d;
          debugOut.out.finishMessage(idDebugRequest);
          return true;
        }
  }
  return false;
}

bool RobotConsole::log(In& stream)
{
  std::string command;
  stream >> command;
  SYNC;
  if(command == "start")
  {
    if(logFile != "")
      logPlayer.play();
    else if(logPlayer.state != LogPlayer::recording)
      logPlayer.recordStart();
    return true;
  }
  else if(command == "stop")
  {
    if(logPlayer.state == LogPlayer::recording)
      logPlayer.recordStop();
    else
      logPlayer.stop();
    return true;
  }
  else if(command == "clear")
  {
    logPlayer.init();
    return true;
  }
  else if(command == "save")
  {
    std::string name;
    stream >> name;
    if(name.size() == 0)
      return false;
    else
    {
      if((int) name.rfind('.') <= (int) name.find_last_of("\\/"))
        name = name + ".log";
      if(name[0] != '/' && name[0] != '\\' && (name.size() < 2 || name[1] != ':'))
        name = std::string("Logs\\") + name;
      return logPlayer.save(name.c_str());
    }
  }
  else if(command == "saveImages")
  {
    stream >> command;
    std::string name(command);
    bool raw = false;
    if(name == "raw")
    {
      raw = true;
      stream >> name;
    }
    if(name.size() == 0)
      name = "Images\\image";
    return logPlayer.saveImages(raw, name.c_str());
  }
  else if(command == "keep" || command == "remove")
  {
    std::string buf;
    stream >> buf;
    std::vector<MessageID> messageIDs;
    while(buf != "")
    {
      int i;
      for(i = 1; i < numOfMessageIDs; ++i)
        if(buf == ::getName(MessageID(i)))
        {
          messageIDs.push_back(MessageID(i));
          break;
        }

      if(i == numOfMessageIDs)
        return false;
      stream >> buf;
    }
    if(messageIDs.size())
    {
      messageIDs.push_back(undefined);
      if(command == "keep")
        logPlayer.keep(&messageIDs[0]);
      else
        logPlayer.remove(&messageIDs[0]);
      return true;
    }
  }
  else if(command == "full")
  {
    logImagesAsJPEGs = false;
    return true;
  }
  else if(command == "jpeg")
  {
    logImagesAsJPEGs = true;
    return true;
  }
  else if(command == "?")
  {
    int frequency[numOfMessageIDs];
    logPlayer.statistics(frequency);
    char buf[33];
    for(int i = 0; i < numOfDataMessageIDs; ++i)
      if(frequency[i])
      {
        sprintf(buf, "%u", frequency[i]);
        ctrl->printLn(std::string(buf) + "\t" + ::getName(MessageID(i)));
      }
    sprintf(buf, "%u", logPlayer.getNumberOfMessages());
    ctrl->printLn(std::string(buf) + "\ttotal");
    return true;
  }
  else if(command == "mr") //log mr
  {
    std::string param;
    stream >> param;

    int frequency[numOfMessageIDs];
    logPlayer.statistics(frequency);

    std::list<std::string> commands;
    for(int i = idImage; i < numOfDataMessageIDs; ++i) //idImage is the first
    {
      if(frequency[i])
      {
        std::string representation = std::string(::getName(MessageID(i))).substr(2);
        if(representation == "JPEGImage")
          representation = "Image";
        
        std::string provider;
        for(std::list<ModuleInfo::Provider>::const_iterator k = moduleInfo.providers.begin(); k != moduleInfo.providers.end(); ++k)
          if(k->representation == representation)
          {
            for(std::vector<std::string>::const_iterator j = k->modules.begin(); j != k->modules.end(); ++j)
              if(*j == "CognitionLogDataProvider")
              {
                provider = *j;
                break; // CognitionLogDataProvider is the default
              }
              else if(*j == "MotionLogDataProvider")
                provider = *j;
            break; // found -> stop searching
          }
        
        if(provider != "")
          commands.push_back(representation + " " + provider);
        else
          ctrl->printLn("Error: cannot provide " + representation);
        
        if(i == idBehaviorControlOutput)
        {
          commands.push_back("MotionRequest CognitionLogDataProvider");
          commands.push_back("HeadMotionRequest CognitionLogDataProvider");
          commands.push_back("SoundRequest CognitionLogDataProvider");
          commands.push_back("BehaviorLEDRequest CognitionLogDataProvider");
        }
      }
    }
    
    bool success = true;
    for(std::list<std::string>::const_iterator i = commands.begin(); i != commands.end(); ++i)
      if(param == "list")
        ctrl->printLn("mr " + *i);
      else
      {
        InTextMemory strMem(i->c_str(), i->size());
        success &= moduleRequestWithoutSync(strMem);
      }
    return success;
  }
  else if(logFile != "")
  {
    if(command == "load")
    {
      std::string name;
      stream >> name;
      if(name.size() == 0)
        return false;
      else
      {
        if((int) name.rfind('.') <= (int) name.find_last_of("\\/"))
          name = name + ".log";
        if(name[0] != '/' && name[0] != '\\' && (name.size() < 2 || name[1] != ':'))
          name = std::string("Logs\\") + name;
        logFile = name;
        LogPlayer::LogPlayerState state = logPlayer.state;
        bool result = logPlayer.open(name.c_str());
        if(result && state == LogPlayer::playing)
          logPlayer.play();
        return result;
      }
    }
    else if(command == "cycle")
    {
      logPlayer.setLoop(true);
      return true;
    }
    else if(command == "once")
    {
      logPlayer.setLoop(false);
      return true;
    }
    else if(command == "pause")
    {
      logPlayer.pause();
      return true;
    }
    else if(command == "forward")
    {
      std::string opt;
      stream >> opt;
      if(opt == "image")
        logPlayer.stepImageForward();
      else
        logPlayer.stepForward();
      return true;
    }
    else if(command == "backward")
    {
      std::string opt;
      stream >> opt;
      if(opt == "image")
        logPlayer.stepImageBackward();
      else
        logPlayer.stepBackward();
      return true;
    }
    else if(command == "repeat")
    {
      logPlayer.stepRepeat();
      return true;
    }
    else if(command == "goto")
    {
      LogPlayer::LogPlayerState state = logPlayer.state;

      int frame;
      stream >> frame;
      logPlayer.gotoFrame(std::max<>(std::min<>(frame - 1, logPlayer.numberOfFrames - 1), 0));
      if(state == LogPlayer::playing)
        logPlayer.play();
      return true;
    }
    else if(command == "fast_forward")
    {
      //backup state, gotoFrame will change the state.
      LogPlayer::LogPlayerState state = logPlayer.state;
      int frame = logPlayer.currentFrameNumber + 100;

      logPlayer.gotoFrame(std::max<>(std::min<>(frame - 1, logPlayer.numberOfFrames - 1), 0));
      if(state == LogPlayer::playing)
      {//if the state was playing before, continue playing
        logPlayer.play();
      }
      return true;
    }
    else if(command == "fast_rewind")
    {
      //backup state, gotoFrame will change the state.
      LogPlayer::LogPlayerState state = logPlayer.state;
      int frame = logPlayer.currentFrameNumber - 100;

      logPlayer.gotoFrame(std::max<>(std::min<>(frame - 1, logPlayer.numberOfFrames - 1), 0));
      if(state == LogPlayer::playing)
      {//if the state was playing before, continue playing
        logPlayer.play();
      }
      return true;
    }
  }
  return false;
}

bool RobotConsole::get(In& stream, bool first, bool print)
{
  std::string request,
              option;
  stream >> request >> option;
  if(request == "?")
  {
    for(int i = 0; i < debugRequestTable.currentNumberOfDebugRequests; ++i)
      if(debugRequestTable.debugRequests[i].description.substr(0, 11) == "debug data:")
        ctrl->list(ctrl->translate(debugRequestTable.debugRequests[i].description.substr(11)), option);
    ctrl->printLn("");
    return true;
  }
  else
    for(int i = 0; i < debugRequestTable.currentNumberOfDebugRequests; ++i)
      if(std::string("debugData:") + request == ctrl->translate(debugRequestTable.debugRequests[i].description))
      {
        if(first)
        {
          DebugRequest& d = debugRequestTable.debugRequests[i];


          // request up-to-date data only of the debug request is not enabled or was only requested once
          if( (d.enable && d.once) || !d.enable)
          {
            SYNC;
            d.once = true;
            d.enable = true;
            debugOut.out.bin << d;
            debugOut.out.finishMessage(idDebugRequest);
          }
          waitingFor[idDebugDataResponse] = 1;
          polled[idDebugDataResponse] = true; // no automatic repolling
          polled[idStreamSpecification] = false;
          handleConsole(std::string(print ? "_get " : "_get3 ") + request + " " + option);
          return true;
        }
        else
        {
          DebugDataInfos::const_iterator j = debugDataInfos.find(debugRequestTable.debugRequests[i].description.substr(11));
          ASSERT(j != debugDataInfos.end());
          if(option == "?")
          {
            printType(j->second.first.c_str());
            ctrl->printLn("");
            return true;
          }
          else if(option == "")
          {
            std::list<std::string> lines;
            ConfigMapWriter writer(lines, *this);
            j->second.second->handleAllMessages(writer);
            std::string buffer = "set " + request;
            for(std::list<std::string>::const_iterator i = lines.begin(); i != lines.end(); ++i)
              buffer += " " + i->substr(i->find_first_not_of(" "));
            if(print)
              ctrl->printLn(buffer);
            else
              printBuffer = buffer;
            return true;
          }
        }
        break;
      }
  return false;
}


bool RobotConsole::RepViewWriter::handleMessage(InMessage& message)
{

  std::string name,
            type;
  message.bin >> name >> type;

  return handleMessage(message,type,name);

}

bool RobotConsole::RepViewWriter::handleMessage(InMessage& message, const std::string& type, const std::string& name){
  std::map<std::string,RepresentationView*>::const_iterator view = pRepViews->find(name);
  ASSERT(message.getMessageID() == idDebugDataResponse);

  if(view != pRepViews->end())
  {
    return view->second->handleMessage(message,type,name);
  }
  else
  {
    return false;
  }
}

bool RobotConsole::set(In& stream)
{
  std::string request,
              option;
  stream >> request;
  stream >> option;
  if(request == "?")
  {
    for(int i = 0; i < debugRequestTable.currentNumberOfDebugRequests; ++i)
      if(debugRequestTable.debugRequests[i].description.substr(0, 11) == "debug data:")
        ctrl->list(ctrl->translate(debugRequestTable.debugRequests[i].description.substr(11)), option);
    ctrl->printLn("");
    return true;
  }
  else
    for(int i = 0; i < debugRequestTable.currentNumberOfDebugRequests; ++i)
      if(std::string("debugData:") + request == ctrl->translate(debugRequestTable.debugRequests[i].description))
      {
        if(option == "unchanged")
        {
          SYNC;
          debugOut.out.bin << debugRequestTable.debugRequests[i].description.substr(11) << char(0);
          debugOut.out.finishMessage(idDebugDataChangeRequest);
          return true;
        }
        else
        {
          char buf[10000];
          OutTextMemory temp(buf);
          temp << option;
          bool singleValue = true;
          while(!stream.eof())
          {
            singleValue = false;
            std::string text;
            stream >> text;
            temp << text;
          }
          buf[temp.getLength()] = 0;
          std::string line(buf);
          DebugDataInfos::const_iterator j = debugDataInfos.find(debugRequestTable.debugRequests[i].description.substr(11)); //the substr(11) removes "debug data:" from the descrption string
          if(j == debugDataInfos.end())
          {
            // request type specification
            {
              SYNC;
              debugOut.out.bin << DebugRequest(debugRequestTable.debugRequests[i].description, true, true);
              debugOut.out.finishMessage(idDebugRequest);
            }
            waitingFor[idDebugDataResponse] = 1;
            polled[idDebugDataResponse] = true; // no automatic repolling
            handleConsole(std::string("_set ") + request + " " + line);
            return true;
          }
          else
          {
            if(option == "?")
            {
              printType(j->second.first.c_str());
              ctrl->printLn("");
              return true;
            }
            else
            {
              SYNC;
              if(singleValue)
                line = "value = " + line + ";";
              MessageQueue errors;
              Global::theDebugOut = &errors.out;
              istrstream mem(line.c_str());
              ConfigMap map;
              if(map.read(mem) > 0)
              {
                debugOut.out.bin << debugRequestTable.debugRequests[i].description.substr(11) << char(1);
                InConfigMap stream(map);
                if(parseData(stream, j->second.first.c_str(), singleValue ? "value" : "") && errors.isEmpty())
                {
                  debugOut.out.finishMessage(idDebugDataChangeRequest);
                  setGlobals();
                  return true;
                }
                else
                  debugOut.out.cancelMessage();
              }
              setGlobals();
              Printer printer(ctrl);
              errors.handleAllMessages(printer);
              return !errors.isEmpty(); // return true if error was already printed
            }
          }
        }
        break;
      }
  return false;
}

void RobotConsole::printType(const char* type, const char* field)
{
  if(type[strlen(type) - 1] == ']')
  {
    int index = strlen(type);
    while(type[index - 1] != '[')
      --index;
    int index2 = type[index - 2] == ' ' ? index - 2 : index - 1;
    printType(std::string(type).substr(0, index2).c_str(), (std::string(field) + (type + index - 1)).c_str());
  }
  else if(type[strlen(type) - 1] == '*')
    printType(std::string(type).substr(0, strlen(type) - (strlen(type) > 1 && type[strlen(type) - 2] == ' ' ? 2 : 1)).c_str(),
              (std::string(field) + "[]").c_str());
  else
  {
    std::string typeName = type;
    if(typeName.size() > 6 && typeName.substr(typeName.size() - 6) == " const")
      typeName = typeName.substr(0, typeName.size() - 6);
    const char* t = streamHandler.getString(typeName.c_str());
    StreamHandler::Specification::const_iterator i = streamHandler.specification.find(t);
    StreamHandler::BasicTypeSpecification::const_iterator b = streamHandler.basicTypeSpecification.find(t);
    StreamHandler::EnumSpecification::const_iterator e = streamHandler.enumSpecification.find(t);
    if(i != streamHandler.specification.end())
    {
      if(*field)
        ctrl->print(std::string(field) + " : {");
      for(std::vector<StreamHandler::TypeNamePair>::const_iterator j = i->second.begin(); j != i->second.end();)
      {
        printType(j->second, j->first.c_str());
        if(++j != i->second.end())
          ctrl->print("; ");
        else
          ctrl->print(";");
      }
      if(*field)
        ctrl->print("}");
    }
    else if(b != streamHandler.basicTypeSpecification.end() || e != streamHandler.enumSpecification.end())
      ctrl->print((*field ? std::string(field) + " : " : "") + t);
    else
      ctrl->print((*field ? std::string(field) + " : " : "") + "UNKNOWN");
  }
}

bool RobotConsole::parseData(In& stream, const char* type, const char* field, int idx)
{
  if(type[strlen(type) - 1] == ']' || (type[0] == 'A' && strchr(type, '_')) || type[strlen(type) - 1] == '*')
  {
    ASSERT(*field);
    unsigned size;
    std::string t;
    if(type[strlen(type) - 1] == ']')
    {
      int index = strlen(type);
      while(type[index - 1] != '[')
        --index;
      stream.select(field, -2);
      size = (unsigned) atoi(type + index);
      index = type[index - 2] == ' ' ? index - 2 : index - 1;
      t = std::string(type).substr(0, index);
    }
    else if(type[0] == 'A' && strchr(type, '_'))
    {
      int index = strchr(type, '_') - type;
      stream.select(field, -2);
      size = (unsigned) atoi(type + 1);
      t = std::string(type).substr(index + 1);
    }
    else
    {
      stream.select(field, -1);
      stream >> size;
      debugOut.out.bin << size;
      t = std::string(type).substr(0, strlen(type) - (strlen(type) > 1 && type[strlen(type) - 2] == ' ' ? 2 : 1));
    }

    for(unsigned i = 0; i < size; ++i)
      if(!parseData(stream, t.c_str(), "", i))
        return false;

    stream.deselect();
  }
  else
  {
    std::string typeName = type;
    if(typeName.size() > 6 && typeName.substr(typeName.size() - 6) == " const")
      typeName = typeName.substr(0, typeName.size() - 6);
    const char* t = streamHandler.getString(typeName.c_str());
    StreamHandler::Specification::const_iterator i = streamHandler.specification.find(t);
    StreamHandler::BasicTypeSpecification::const_iterator b = streamHandler.basicTypeSpecification.find(t);
    StreamHandler::EnumSpecification::const_iterator e = streamHandler.enumSpecification.find(t);
    if(i != streamHandler.specification.end())
    {
      if(*field)
        stream.select(field, -2);
      else if(idx >= 0)
        stream.select(0, idx);

      for(std::vector<StreamHandler::TypeNamePair>::const_iterator j = i->second.begin(); j != i->second.end(); ++j)
        if(!parseData(stream, j->second, j->first.c_str()))
          return false;

      if(*field || idx >= 0)
        stream.deselect();
    }
    else if(b != streamHandler.basicTypeSpecification.end())
    {
      if(*field)
        stream.select(field, -2);
      else
        stream.select(0, idx);

      if(!strcmp("double", t))
      {
        double d;
        stream >> d;
        debugOut.out.bin << d;
      }
      else if(!strcmp("bool", t))
      {
        bool b;
        stream >> b;
        debugOut.out.bin << b;
      }
      else if(!strcmp("float", t))
      {
        float f;
        stream >> f;
        debugOut.out.bin << f;
      }
      else if(!strcmp("int", t))
      {
        int i;
        stream >> i;
        debugOut.out.bin << i;
      }
      else if(!strcmp("unsigned", t) || !strcmp("unsigned int", t))
      {
        unsigned int u;
        stream >> u;
        debugOut.out.bin << u;
      }
      else if(!strcmp("short", t))
      {
        short s;
        stream >> s;
        debugOut.out.bin << s;
      }
      else if(!strcmp("unsigned short", t))
      {
        unsigned short u;
        stream >> u;
        debugOut.out.bin << u;
      }
      else if(!strcmp("char", t))
      {
        char c;
        stream >> c;
        debugOut.out.bin << c;
      }
      else if(!strcmp("unsigned char", t))
      {
        unsigned char u;
        stream >> u;
        debugOut.out.bin << u;
      }
      else if(!strcmp("class std::basic_string<char,struct std::char_traits<char>,class std::allocator<char> >", t) ||
              !strcmp(type, "std::string"))
      {
        std::string s;
        stream >> s;
        debugOut.out.bin << s;
      }

      stream.deselect();
    }
    else if(e != streamHandler.enumSpecification.end())
    {
      enumSpec = &e;
      if(*field)
        stream.select(field, -2, &getName);
      else
        stream.select(0, idx, &getName);
      int value;
      stream >> value;
      debugOut.out.bin << value;
      stream.deselect();
    }
    else
      return false;
  }
  return true;
}

const char* RobotConsole::getName(int value)
{
  return (unsigned) value >= (*enumSpec)->second.size() ? 0 : (*enumSpec)->second[value];
}

bool RobotConsole::sendMof(In& stream)
{
  std::string parameter;
  stream >> parameter;
  if(parameter != "")
    return false;

  char buffer[200000];
  const bool success = compileMofs(buffer, sizeof(buffer));

  char* p = buffer;
  while(*p)
  {
    char* p2 = strchr(p, '\n');
    *p2 = 0;
    ctrl->printLn(p);
    p = p2 + 1;
  }

  if(success)
  {
    ctrl->printLn("SpecialActions compiled");
    InBinaryFile file("specialActions.dat");
    if(file.exists())
    {
      memset(buffer, 0, sizeof(buffer));
      file.read(buffer, sizeof(buffer));
      SYNC;
      debugOut.out.bin.write(buffer, strlen(buffer));
      debugOut.out.finishMessage(idMotionNet);
    }
    else
      ctrl->printLn("Error: specialActions.dat not found.");
  }
  return true;
}

bool RobotConsole::sendWek(In& stream)
{
  std::string parameter;
  stream >> parameter;
  if(parameter != "")
    return false;

  for(int i = 1; i < WalkRequest::numOfKickTypes; i += 2)
  {
    char filePath[256];
    sprintf(filePath, "Kicks/%s.cfg", WalkRequest::getName(WalkRequest::KickType(i)));
    File file(filePath, "rb");
    if(!file.exists())
      continue;
    unsigned int size = file.getSize();
    char* buffer = new char[size];
    file.read(buffer, size);
    {
      SYNC;
      debugOut.out.bin << i << size;
      debugOut.out.bin.write(buffer, size);
      debugOut.out.finishMessage(idWalkingEngineKick);
    }
    delete[] buffer;
  }

  return true;
}

bool RobotConsole::repoll(In& stream)
{
  polled[idDebugResponse] = polled[idDrawingManager] = polled[idDrawingManager3D] = false;
  return true;
}

bool RobotConsole::queueFillRequest(In& stream)
{
  std::string request;
  stream >> request;
  QueueFillRequest qfr;
  if(request == "queue")
  {
    qfr.behavior = QueueFillRequest::sendImmediately;
    qfr.filter = QueueFillRequest::sendEverything;
    qfr.target = QueueFillRequest::sendViaNetwork;
  }
  else if(request == "replace")
  {
    qfr.behavior = QueueFillRequest::sendImmediately;
    qfr.filter = QueueFillRequest::latestOnly;
    qfr.target = QueueFillRequest::sendViaNetwork;
  }
  else if(request == "reject")
  {
    qfr.behavior = QueueFillRequest::discardAll;
  }
  else if(request == "collect")
  {
    qfr.behavior = QueueFillRequest::sendAfter;
    qfr.filter = QueueFillRequest::sendEverything;
    qfr.target = QueueFillRequest::sendViaNetwork;

    stream >> qfr.timingMilliseconds;
    qfr.timingMilliseconds *= 1000;
    if(!qfr.timingMilliseconds)
      return false;
  }
  else if(request == "save")
  {
    qfr.filter = QueueFillRequest::sendEverything;
    qfr.target = QueueFillRequest::writeToStick;

    stream >> qfr.timingMilliseconds;
    qfr.timingMilliseconds *= 1000;
    if(!qfr.timingMilliseconds)
      qfr.behavior = QueueFillRequest::sendImmediately;
    else
      qfr.behavior = QueueFillRequest::sendAfter;
  }
  else
    return false;
  SYNC;
  debugOut.out.bin << qfr;
  debugOut.out.finishMessage(idQueueFillRequest);
  return true;
}

bool RobotConsole::moduleRequest(In& stream)
{
  SYNC;
  return moduleRequestWithoutSync(stream);
}


bool RobotConsole::moduleRequestWithoutSync(In& stream)
{
  std::string representation,
              module,
              pattern;
  stream >> representation >> module >> pattern;
  if(representation == "modules")
  {
    for(std::list<ModuleInfo::Module>::const_iterator i = moduleInfo.modules.begin(); i != moduleInfo.modules.end(); ++i)
      if(i->name != "default")
      {
        std::string text = i->name + " (" + (i->processIdentifier == 'c' ? "Cognition" : "Motion") + ", " + i->category + "): ";
        for(std::vector<std::string>::const_iterator j = i->requirements.begin(); j != i->requirements.end(); ++j)
          text += *j + " ";
        text += "-> ";
        for(std::vector<std::string>::const_iterator j = i->representations.begin(); j != i->representations.end(); ++j)
        {
          std::list<ModuleInfo::Provider>::const_iterator k;
          for(k = moduleInfo.providers.begin(); k != moduleInfo.providers.end() && k->representation != *j; ++k)
            ;
          text += *j + (k != moduleInfo.providers.end() && k->selected == i->name ? "* " : " ");
        }
        ctrl->list(text, module, true);
      }

    return true;
  }
  else if(representation == "?")
  {
    for(std::list<ModuleInfo::Provider>::const_iterator i = moduleInfo.providers.begin(); i != moduleInfo.providers.end(); ++i)
    {
      std::string text = i->representation + " (" + (i->processIdentifier == 'c' ? "Cognition" : "Motion") + "): ";
      for(std::vector<std::string>::const_iterator j = i->modules.begin(); j != i->modules.end(); ++j)
        if(*j == i->selected || *j != "default")
          text += *j + (*j == i->selected ? "* " : " ");
      ctrl->list(text, module, true);
    }
    return true;
  }
  else if(representation == "save")
  {
    OutConfigMap stream(Global::getSettings().expandLocationFilename("modules.cfg"), false);
    std::string error = moduleInfo.sendRequest(stream, true);
    if(error != "")
      ctrl->printLn(error);
    else
      stream.write();
    return true;
  }
  else
  {
    bool found = false;
    for(std::list<ModuleInfo::Provider>::iterator i = moduleInfo.providers.begin(); i != moduleInfo.providers.end(); ++i)
      if(i->representation == representation)
      {
        if(module == "?")
        {
          for(std::vector<std::string>::const_iterator j = i->modules.begin(); j != i->modules.end(); ++j)
            if(*j == i->selected || *j != "default")
              ctrl->list(*j + (*j == i->selected ? "*" : ""), pattern);
          found = true;
        }
        else
        {
          if(module == "off")
          {
            i->selected = "";
            found = true;
          }
          else
          {
            if(std::find(i->modules.begin(), i->modules.end(), module) != i->modules.end())
            {
              found = true;
              i->selected = module;
            }
          }
        }
      }

    if(found)
    {
      if(module == "?")
        ctrl->printLn("");
      else
      {

        moduleInfo.timeStamp = SystemCall::getCurrentSystemTime();
        debugOut.out.bin << moduleInfo.timeStamp;
        std::string error = moduleInfo.sendRequest(debugOut.out.bin);
        if(error == "")
          debugOut.out.finishMessage(idModuleRequest);
        else
        {
          ctrl->printLn(error);
          debugOut.out.cancelMessage();
          return true; // Error message already displayed
        }
        polled[idDebugResponse] = polled[idDrawingManager] = polled[idDrawingManager3D] = false;
      }
      return true;
    }
  }
  return false;
}

bool RobotConsole::moveRobot(In& stream)
{
  SYNC;
  stream >> movePos.x >> movePos.y >> movePos.z;
  if(stream.eof())
    moveOp = movePosition;
  else
  {
    stream >> moveRot.x >> moveRot.y >> moveRot.z;
    moveOp = moveBoth;
  }
  return true;
}

bool RobotConsole::moveBall(In& stream)
{
  SYNC;
  stream >> movePos.x >> movePos.y >> movePos.z;
  moveOp = moveBallPosition;
  return true;
}


void RobotConsole::printLn(const std::string & line)
{
  if(NULL != ctrl)
  {
    ctrl->printLn(line);
  }
}

bool RobotConsole::view3D(In& stream)
{
  std::string buffer;
  stream >> buffer;
  if(buffer == "?")
  {
    stream >> buffer;
    ctrl->list("image", buffer);
    for(int i = 0; i < debugRequestTable.currentNumberOfDebugRequests; ++i)
      if(debugRequestTable.debugRequests[i].description.substr(0, 13) == "debug images:")
        ctrl->list(ctrl->translate(debugRequestTable.debugRequests[i].description.substr(13)), buffer);
    ctrl->printLn("");
    return true;
  }
  else
  {
    std::string buffer2;
    bool jpeg = false;
    for(;;)
    {
      stream >> buffer2;
      if(!jpeg && buffer2 == "jpeg")
        jpeg = true;
      else
        break;
    }

    if(buffer == "image")
    {
      std::string name = buffer2 != "" ? buffer2 : std::string("image");
      if(imageViews3D.find(name) != imageViews3D.end())
      {
        ctrl->printLn("View already exists. Specify a (different) name.");
        return true;
      }
      imageViews3D[name];
      addColorSpaceViews("raw image", name, false);
      if(jpeg)
        handleConsole("dr representation:JPEGImage on");
      else
        handleConsole("dr representation:Image on");
      return true;
    }
    else if(!jpeg)
      for(int i = 0; i < debugRequestTable.currentNumberOfDebugRequests; ++i)
        if(debugRequestTable.debugRequests[i].description.substr(0, 13) == "debug images:" &&
           ctrl->translate(debugRequestTable.debugRequests[i].description.substr(13)) == buffer)
        {
          std::string name = buffer2 != "" ? buffer2 : std::string(buffer);
          if(imageViews3D.find(name) != imageViews3D.end())
          {
            ctrl->printLn("View already exists. Specify a (different) name.");
            return true;
          }
          addColorSpaceViews(debugRequestTable.debugRequests[i].description.substr(13), name, true);
          handleConsole(std::string("dr ") + ctrl->translate(debugRequestTable.debugRequests[i].description) + " on");
          return true;
        }
  }
  return false;
}

bool RobotConsole::viewField(In& stream)
{
  std::string name;
  stream >> name;
  if(fieldViews.find(name) != fieldViews.end())
    ctrl->printLn("View already exists. Specify a different name.");
  else
  {
    fieldViews[name];
    ctrl->setFieldViews(fieldViews);
    ctrl->updateCommandCompletion();
    ctrl->addView(new FieldView(robotName + ".field." + name.c_str(), *this, name), robotName + ".field", SimRobot::Flag::copy | SimRobot::Flag::exportAsSvg);
  }
  return true;
}


bool RobotConsole::representationNameExists(const std::string& name)
{
  for(int i = 0; i < debugRequestTable.currentNumberOfDebugRequests; ++i)
  {

    if(debugRequestTable.debugRequests[i].description.substr(0, 11) == "debug data:")
    {
      std::string existingName = ctrl->translate(debugRequestTable.debugRequests[i].description.substr(11));
      if(name == existingName)
      {
        return true;
      }
    }
  }
  return false;

}

bool RobotConsole::viewRepresentation(In& stream)
{
  std::string name;
  stream >> name;
  if(representationViews.find(name) != representationViews.end())
  {
    ctrl->printLn("Representation view already exists.");
  }
  else if (!representationNameExists(name))
  {
    ctrl->printLn("'" + name + "' is not a valid representation name");
  }
  else
  {
    representationViews[name] = new RepresentationView(robotName + ".representation." + name.c_str(), name, *this, streamHandler);
    ctrl->addView(representationViews[name], robotName + ".representation", SimRobot::Flag::copy | SimRobot::Flag::exportAsSvg);

    //enable the debug request.
    //once the debug request is enabled the robot will send up to date data every time.
    //Note: If you use "get represenation:bla" afterwards it will turn the debug request  back off
    for(int i = 0; i < debugRequestTable.currentNumberOfDebugRequests; ++i)
      if(std::string("debugData:") + name == ctrl->translate(debugRequestTable.debugRequests[i].description))
      {
        //enable the debug request if it is not already enabled
        DebugRequest& d = debugRequestTable.debugRequests[i];
        SYNC;
        if(!d.enable || d.once)
        {
          d.enable = true;
          d.once = false;
          debugOut.out.bin << d;
          debugOut.out.finishMessage(idDebugRequest);
        }

        waitingFor[idDebugDataResponse] = 1;
        polled[idDebugDataResponse] = true;
        polled[idStreamSpecification] = false; //repoll stream specification in part 2
        handleConsole(std::string("_vr"));//continue in part 2

        break;
      }
  }
  return true;
}


bool RobotConsole::viewDrawing(In& stream, RobotConsole::Views& views, const char* type)
{
  std::string buffer;
  stream >> buffer;
  if(buffer == "?")
  {
    stream >> buffer;
    for(Views::const_iterator i = views.begin(); i != views.end(); ++i)
      ctrl->list(i->first, buffer);
    ctrl->printLn("");
    return true;
  }
  else
    for(Views::const_iterator i = views.begin(); i != views.end(); ++i)
      if(i->first == buffer)
      {
        stream >> buffer;
        if(buffer == "?")
        {
          stream >> buffer;
          for(std::tr1::unordered_map< const char*, DrawingManager::Drawing>::const_iterator j = drawingManager.drawings.begin(); j != drawingManager.drawings.end(); ++j)
            if(!strcmp(drawingManager.getDrawingType(j->first), type))
              ctrl->list(ctrl->translate(j->first), buffer);
          ctrl->printLn("");
          return true;
        }
        else
        {
          for(std::tr1::unordered_map< const char*, DrawingManager::Drawing>::const_iterator j = drawingManager.drawings.begin(); j != drawingManager.drawings.end(); ++j)
            if(ctrl->translate(j->first) == buffer && !strcmp(drawingManager.getDrawingType(j->first), type))
            {
              std::string buffer2;
              stream >> buffer2;
              if(buffer2 == "on" || buffer2 == "")
              {
                views[i->first].remove(j->first);
                views[i->first].push_back(j->first);
                handleConsole(std::string("dr debugDrawing:") + buffer + " on");
                return true;
              }
              else if(buffer2 == "off")
              {
                views[i->first].remove(j->first);
                handleConsole(std::string("dr debugDrawing:") + buffer + " off");
                return true;
              }
              else
                return false;
            }
        }
      }
  return false;
}

bool RobotConsole::viewImage(In& stream)
{
  std::string buffer;
  stream >> buffer;
  if(buffer == "?")
  {
    stream >> buffer;
    ctrl->list("none", buffer);
    ctrl->list("image", buffer);
    ctrl->list("imageOther", buffer);
    for(int i = 0; i < debugRequestTable.currentNumberOfDebugRequests; ++i)
      if(debugRequestTable.debugRequests[i].description.substr(0, 13) == "debug images:")
        ctrl->list(ctrl->translate(debugRequestTable.debugRequests[i].description.substr(13)), buffer);
    ctrl->printLn("");
    return true;
  }
  else if(buffer == "none")
  {
    stream >> buffer;
    std::string name = buffer != "" ? buffer : "none";
    if(imageViews.find(name) != imageViews.end())
    {
      ctrl->printLn("View already exists. Specify a (different) name.");
      return true;
    }
    imageViews[name];
    ctrl->setImageViews(imageViews);
    ctrl->updateCommandCompletion();
    ctrl->addView(new ImageView(robotName + ".image." + name.c_str(), *this, "none", name, false), robotName + ".image", SimRobot::Flag::copy | SimRobot::Flag::exportAsSvg);
    return true;
  }
  else
  {
    std::string buffer2;
    bool jpeg = false,
         segmented = false;
    for(;;)
    {
      stream >> buffer2;
      if(!jpeg && buffer2 == "jpeg")
        jpeg = true;
      else if(!segmented && buffer2 == "segmented")
        segmented = true;
      else
        break;
    }

    if(buffer == "image" || buffer == "imageOther")
    {
      std::string name = buffer2 != "" ? buffer2 : buffer + (segmented ? "Segmented" : "");
      if(imageViews.find(name) != imageViews.end())
      {
        ctrl->printLn("View already exists. Specify a (different) name.");
        return true;
      }
      imageViews[name];
      ctrl->setImageViews(imageViews);
      ctrl->updateCommandCompletion();
      std::string enableGain;
      stream >> enableGain;
      float gain = 1.0f;
      if(enableGain == "gain")
      {
        stream >> gain;
      }
      ctrl->addView(new ImageView(robotName + ".image." + name.c_str(), *this, ("raw " + buffer).c_str(), name, segmented, gain), robotName + ".image", SimRobot::Flag::copy | SimRobot::Flag::exportAsSvg);
      if(jpeg)
        handleConsole("dr representation:JPEGI" + buffer.substr(1) + " on");
      else
        handleConsole("dr representation:I" + buffer.substr(1) + " on");

      return true;
    }
    else if(!jpeg)
      for(int i = 0; i < debugRequestTable.currentNumberOfDebugRequests; ++i)
        if(debugRequestTable.debugRequests[i].description.substr(0, 13) == "debug images:" &&
           ctrl->translate(debugRequestTable.debugRequests[i].description.substr(13)) == buffer)
        {
          std::string name = buffer2 != "" ? buffer2 : std::string(buffer) + (segmented ? "Segmented" : "");
          if(imageViews.find(name) != imageViews.end())
          {
            ctrl->printLn("View already exists. Specify a (different) name.");
            return true;
          }
          imageViews[name];
          ctrl->setImageViews(imageViews);
          ctrl->updateCommandCompletion();
          ctrl->addView(new ImageView(robotName + ".image." + name.c_str(), *this, debugRequestTable.debugRequests[i].description.substr(13), name, segmented), robotName + ".image", SimRobot::Flag::copy | SimRobot::Flag::exportAsSvg);
          handleConsole(std::string("dr ") + ctrl->translate(debugRequestTable.debugRequests[i].description) + " on");
          return true;
        }
  }
  return false;
}

bool RobotConsole::viewPlot(In& stream)
{
  std::string name;
  int plotSize;
  float minValue;
  float maxValue;
  std::string yUnit;
  std::string xUnit;
  float xScale;
  stream >> name >> plotSize >> minValue >> maxValue >> yUnit >> xUnit >> xScale;
  if(plotSize < 2 || minValue >= maxValue)
    return false;
  if(xScale == 0.)
    xScale = 1.;
  QString fullName = robotName + ".plot." + name.c_str();
  if((unsigned int)plotSize > maxPlotSize)
    maxPlotSize = plotSize;
  if(plotViews.find(name) != plotViews.end())
  {
    PlotView* plotView = (PlotView*)ctrl->application->resolveObject(fullName);
    ASSERT(plotView);
    plotView->setParameters((unsigned int)plotSize, minValue, maxValue, yUnit, xUnit, xScale);
    return true;
  }
  plotViews[name];
  ctrl->setPlotViews(plotViews);
  ctrl->updateCommandCompletion();
  ctrl->addView(new PlotView(fullName, *this, name, (unsigned int)plotSize, minValue, maxValue, yUnit, xUnit, xScale), robotName + ".plot", SimRobot::Flag::copy | SimRobot::Flag::exportAsSvg);
  return true;
}

bool RobotConsole::viewBike()
{
  if(bikeView)
    ctrl->printLn("View already exists.");
  else
  {
    if(mode == SystemCall::simulatedRobot)
    {
      bikeView = true;
      ctrl->addView(new ViewBike(robotName + ".BikeView", *this, motionRequest, jointData, jointCalibration, sensorData, robotDimensions,
                                 printBuffer,
                                 (SimRobotCore2::Body*)ctrl->application->resolveObject(robotFullName, SimRobotCore2::body)), robotName);
      return true;
    }
    if(mode == SystemCall::remoteRobot)
    {
      bikeView = true;
      QString puppetName("RoboCup.puppets." + robotName);

      ctrl->addView(new ViewBike(robotName + ".BikeView", *this, motionRequest, jointData, jointCalibration, sensorData, robotDimensions,
                                 printBuffer,
                                 (SimRobotCore2::Body*)ctrl->application->resolveObject(puppetName, SimRobotCore2::body)), robotName);
      return true;
    }
  }
  return false;
}

void RobotConsole::sendDebugMessage(InMessage & msg)
{
  SYNC;
  msg >> debugOut;
}

std::string RobotConsole::getDebugRequest(const std::string& name)
{

  for(int i = 0; i < debugRequestTable.currentNumberOfDebugRequests; ++i)
  {
    if(std::string("debugData:") + name == ctrl->translate(debugRequestTable.debugRequests[i].description))
    {
      return debugRequestTable.debugRequests[i].description.substr(11);
    }
  }
  printLn("Error: RobotConsole: DebugRequest not found.");
  return "";


}


bool RobotConsole::viewPlotDrawing(In& stream)
{
  std::string buffer;
  stream >> buffer;
  if(buffer == "?")
  {
    stream >> buffer;
    for(PlotViews::const_iterator i = plotViews.begin(); i != plotViews.end(); ++i)
      ctrl->list(i->first.c_str(), buffer);
    ctrl->printLn("");
    return true;
  }
  else
    for(PlotViews::const_iterator i = plotViews.begin(); i != plotViews.end(); ++i)
      if(i->first == buffer)
      {
        stream >> buffer;
        if(buffer == "?")
        {
          stream >> buffer;
          for(int j = 0; j < debugRequestTable.currentNumberOfDebugRequests; ++j)
            if(debugRequestTable.debugRequests[j].description.substr(0, 5) == "plot:")
              ctrl->list(ctrl->translate(debugRequestTable.debugRequests[j].description.substr(5)).c_str(), buffer);
          ctrl->printLn("");
          return true;
        }
        else
        {
          for(int j = 0; j < debugRequestTable.currentNumberOfDebugRequests; ++j)
            if(ctrl->translate(debugRequestTable.debugRequests[j].description) == std::string("plot:") + buffer)
            {
              Layer layer;
              layer.layer = buffer;
              stream >> buffer;
              if(buffer == "?")
              {
                stream >> buffer;
                for(int color = 1; color < ColorClasses::numOfColors; ++color)
                  ctrl->list(ctrl->translate(ColorClasses::getName((ColorClasses::Color) color)).c_str(), buffer);
                ctrl->printLn("");
                return true;
              }
              if(buffer == "off")
              {
                {
                  SYNC;
                  plotViews[i->first].remove(layer);
                }
                handleConsole(std::string("dr plot:") + layer.layer + " off");
                return true;
              }
              else
              {
                int color;
                for(color = 1; color < ColorClasses::numOfColors; ++color)
                  if(ctrl->translate(ColorClasses::getName((ColorClasses::Color) color)) == buffer)
                  {
                    layer.color = ColorClasses::Color(color);
                    break;
                  }

                if(color >= ColorClasses::numOfColors)
                {
                  int c = 0;
                  sscanf(buffer.c_str(), "%x", &c);
                  layer.color = ColorRGBA((c >> 16) & 0xff, (c >> 8) & 0xff, c & 0xff);
                }
                stream >> layer.description;
                if(layer.description.empty())
                {
                  const char* name = strrchr(layer.layer.c_str(), ':');
                  if(name)
                    layer.description = name + 1;
                  else
                    layer.description = layer.layer;
                }
                {
                  SYNC;
                  plotViews[i->first].remove(layer);
                  plotViews[i->first].push_back(layer);
                }
                handleConsole(std::string("dr plot:") + layer.layer + " on");
                return true;
              }
              return false;
            }
        }
      }
  return false;
}

bool RobotConsole::joystickExecCommand(const std::string& cmd)
{
  if(cmd == "")
    return false;

  ctrl->executeConsoleCommand(cmd, this);
  if(joystickTrace)
    ctrl->printLn(cmd);

  return true;
}

void RobotConsole::handleJoystick()
{
  if(!joystick.update())
    return; //return if no joystick was found

  // handle joystick events
  unsigned int buttonId;
  bool pressed;
  bool buttonCommandExecuted(false);
  while(joystick.getNextEvent(buttonId, pressed))
  {
    ASSERT(buttonId < Joystick::numOfButtons);
    buttonCommandExecuted |= joystickExecCommand(pressed ? joystickButtonPressCommand[buttonId] : joystickButtonReleaseCommand[buttonId]);
  }

  // walk and move head only when there is no button command
  if(buttonCommandExecuted)
    return;

  unsigned int timeNow = SystemCall::getCurrentSystemTime();
  if(lines.empty()
     && timeNow - joystickLastTime >= 100)  // don't generate too many commands
  {
    joystickLastTime = timeNow;
    float speeds[6];
    bool preparedSpeeds = false;
    for(int j = 0; j < joystickNumOfMotionCommands; ++j)
    {
      JoystickMotionCommand& cmd(joystickMotionCommands[j]);
      if(!cmd.command.empty())
      {
        if(!preparedSpeeds)
        {
          ASSERT(Joystick::numOfAxes == 6);
          for(int i = 0; i < 6; ++i)
          {
            float d = joystick.getAxisState(i);
            float threshold = joystickAxisThresholds[i];
            if(d < -threshold)
              speeds[i] = (d + threshold) / (1 - threshold);
            else if(d > threshold)
              speeds[i] = (d - threshold) / (1 - threshold);
            else
              speeds[i] = 0;
            if(joystickAxisMappings[i])
            {
              bool pressed1 = joystick.isButtonPressed(joystickAxisMappings[i] & 0xffff);
              bool pressed2 = joystick.isButtonPressed(joystickAxisMappings[i] >> 16);
              if(pressed1 != pressed2)
                speeds[i] = pressed1 ? 1.f : -1.f;
            }
            speeds[i] *= joystickAxisMaxSpeeds[i];
            speeds[i] += joystickAxisCenters[i];
          }
          preparedSpeeds = true;
        }
        if(joystickCommandBuffer.size() < cmd.command.length() + 258)
          joystickCommandBuffer.resize(cmd.command.length() + 258);
        sprintf(&joystickCommandBuffer[0], cmd.command.c_str(),
                speeds[cmd.indices[0]],
                speeds[cmd.indices[1]],
                speeds[cmd.indices[2]],
                speeds[cmd.indices[3]],
                speeds[cmd.indices[4]],
                speeds[cmd.indices[5]]);
        if(strcmp(cmd.lastCommand.c_str(), &joystickCommandBuffer[0]))
        {
          joystickExecCommand(&joystickCommandBuffer[0]);
          cmd.lastCommand = &joystickCommandBuffer[0];
        }
      }
    }
  }
}

bool RobotConsole::joystickCommand(In& stream)
{
  std::string command;
  stream >> command;
  if(command == "show")
  {
    joystickTrace = true;
    return true;
  }
  else if(command == "hide")
  {
    joystickTrace = false;
    return true;
  }

  int number;
  stream >> number;
  //rest of line into one string (std::stringstream-like .str() would be nice :-/):
  std::string line;
  stream >> line;
  while(!stream.eof())
  {
    std::string text;
    stream >> text;
    line += ' ';
    line += text;
  }

  if(command == "press" || command == "release")
  {
    if(number > 0 && number <= Joystick::numOfButtons)
    {
      if(command == "release")
        joystickButtonReleaseCommand[number - 1].swap(line);
      else
        joystickButtonPressCommand[number - 1].swap(line);
      return true;
    }
    return false;
  }
  else if(command == "motion")
  {
    if(number > 0 && number <= joystickNumOfMotionCommands)
    {
      JoystickMotionCommand& cmd(joystickMotionCommands[number - 1]);
      for(int i = 0; i < Joystick::numOfAxes; ++i)
        cmd.indices[i] = 0;
      int pos = line.find("$");
      int i = 0;
      while(i < Joystick::numOfAxes && pos != -1)
      {
        int id = line[pos + 1] - '1';
        if(id >= 0 && id < Joystick::numOfAxes)
        {
          cmd.indices[i++] = id;
          line.replace(pos, 2, "%lf");
          pos = line.find("$");
        }
        else
          return false;
      }
      cmd.command.swap(line);
      cmd.lastCommand.clear();
      return true;
    }
    return false;
  }
  return false;
}

bool RobotConsole::joystickSpeeds(In& stream)
{
  int id;
  stream >> id;
  if(id > 0 && id <= Joystick::numOfAxes)
  {
    stream >> joystickAxisMaxSpeeds[id - 1] >> joystickAxisThresholds[id - 1] >> joystickAxisCenters[id - 1];
    return true;
  }
  return false;
}

bool RobotConsole::joystickMaps(In& stream)
{
  int axis, button1, button2;
  stream >> axis >> button1 >> button2;
  if(axis > 0 && axis <= Joystick::numOfAxes && button1 >= 0 && button1 <= Joystick::numOfButtons && button2 > 0 && button2 <= Joystick::numOfButtons)
  {
    joystickAxisMappings[axis - 1] = button1 == 0 ? 0 : ((button1 - 1) | ((button2 - 1) << 16));
    return true;
  }
  return false;
}

bool RobotConsole::saveRequest(In& stream, bool first)
{
  std::string buffer;
  std::string path;
  stream >> buffer >> path;

  if(buffer == "?")
  {
    for(tr1::unordered_map<std::string, std::string>::const_iterator i = ctrl->representationToFile.begin();
        i != ctrl->representationToFile.end(); ++i)
      ctrl->list(i->first.c_str(), path);
    ctrl->printLn("");
    return true;
  }
  else
  {
    for(int i = 0; i < debugRequestTable.currentNumberOfDebugRequests; ++i)
      if(std::string("debugData:") + buffer == ctrl->translate(debugRequestTable.debugRequests[i].description))
      {
        if(first) // request current Values
        {
          SYNC;
          debugOut.out.bin << DebugRequest(debugRequestTable.debugRequests[i].description, true, true);
          debugOut.out.finishMessage(idDebugRequest);

          waitingFor[idDebugDataResponse] = 1;
          polled[idDebugDataResponse] = true; // no automatic repolling
          handleConsole(std::string("_save ") + buffer + " " + path);
          return true;
        }
        else
        {
          DebugDataInfos::const_iterator j = debugDataInfos.find(debugRequestTable.debugRequests[i].description.substr(11));
          ASSERT(j != debugDataInfos.end());

          std::string filename;
          if(path == "") // no path specified, use default location
          {
            filename = getPathForRepresentation(debugRequestTable.debugRequests[i].description.substr(11));
            if(filename == "")
            {
              ctrl->printLn("Error getting filename for " + debugRequestTable.debugRequests[i].description.substr(11) + ". Representation can not be saved.");
              return true;
            }
          }
          else  // path specified
          {
            if(path.find_first_of("/") == 0)
              path = path.substr(1);
            filename = "../" + path;
          }
          std::list<std::string> lines;
          ConfigMapWriter writer(lines, *this);
          j->second.second->handleAllMessages(writer);
          OutTextRawFile file(filename);
          for(std::list<std::string>::const_iterator i = lines.begin(); i != lines.end(); ++i)
            file << *i << endl;
          return true;
        }
      }
    return false;
  }
}

bool RobotConsole::saveImage(In& stream)
{
  std::string filename;
  stream >> filename;
  if(filename == "reset")
  {
    imageSaveNumber = 0;
    return true;
  }
  else
  {
    int number = -1;
    if(filename == "number")
    {
      number = imageSaveNumber++;
      stream >> filename;
    }
    bool both = false;
    if(filename == "both")
    {
      both = true;
      stream >> filename;
    }
    if(filename == "")
      filename = "raw_image.bmp";
    
    std::string filenameOther;
    std::string::size_type p = (int) filename.rfind('.');
    if((int) p > (int) filename.find_last_of("\\/"))
      filenameOther = filename.substr(0, p) + "_other" + filename.substr(p);
    else
      filenameOther = filename + "_other";

    SYNC;
    Image* srcImage = images["raw image"].image;
    Image* srcImageOther = both ? images["raw imageOther"].image : 0;
    return srcImage && LogPlayer::saveImage(*srcImage, filename.c_str(), number) &&
           (!both || (srcImageOther && LogPlayer::saveImage(*srcImageOther, filenameOther.c_str(), number)));
  }
}

void RobotConsole::handleKeyEvent(int key, bool pressed)
{
  if(joystickTrace && pressed)
  {
    char buf[33];
    sprintf(buf, "%u", key + 1);
    ctrl->printLn(std::string("shortcut: ") + buf);
  }
  std::string* joystickButtonCommand(pressed ? joystickButtonPressCommand : joystickButtonReleaseCommand);
  if(key >= 0 && key < Joystick::numOfButtons && joystickButtonCommand[key] != "")
    ctrl->executeConsoleCommand(joystickButtonCommand[key], this);
}

std::string RobotConsole::getPathForRepresentation(std::string representation)
{
  tr1::unordered_map<std::string, std::string>::const_iterator i = ctrl->representationToFile.find(representation);
  if(i == ctrl->representationToFile.end())
    return "";

  // Check where the file has to be placed. Search order:
  // Locations/[locationname]/Robots/[robotname]
  // Locations/Default/[robotname]
  // Robots/[robotname]
  // Robots/Default
  // Locations/[locationname]
  // Locations/Default
  // the config directory, default return value

  std::list<std::string> names = File::getFullNames(Global::getSettings().expandRobotLocationFilename(i->second));
  for(std::list<std::string>::const_iterator i = names.begin(); i != names.end(); ++i)
  {
    File path(*i, "r", false);
    if(path.exists())
      return *i;
  }

  names = File::getFullNames(Global::getSettings().expandRobotFilename(i->second));
  for(std::list<std::string>::const_iterator i = names.begin(); i != names.end(); ++i)
  {
    File path(*i, "r", false);
    if(path.exists())
      return *i;
  }

  names = File::getFullNames(Global::getSettings().expandLocationFilename(i->second));
  for(std::list<std::string>::const_iterator i = names.begin(); i != names.end(); ++i)
  {
    File path(*i, "r", false);
    if(path.exists())
      return *i;
  }

  // if file is not anywhere else, return config directory as default directory
  return i->second;
}

void RobotConsole::writeConfigMap(In& stream, const char* type, std::list<std::string>& lines, const char* field, bool root)
{
  char buf[1000];
  if(type[strlen(type) - 1] == ']' || type[strlen(type) - 1] == '*')
  {
    int size;
    std::string s;
    std::string t;
    if(type[strlen(type) - 1] == ']')
    {
      int index = strlen(type);
      while(type[index - 1] != '[')
        --index;
      int index2 = type[index - 2] == ' ' ? index - 2 : index - 1;
      size = atoi(type + index);
      s = std::string(field) + " = [";
      t = std::string(type).substr(0, index2);
    }
    else
    {
      stream >> size;
      sprintf(buf, "%d", size);
      s = std::string(field) + " = [";
      t = std::string(type).substr(0, strlen(type) - (strlen(type) > 1 && type[strlen(type) - 2] == ' ' ? 2 : 1));
    }

    std::list<std::string> l;
    bool flat = true;
    for(int i = 0; i < size; ++i)
    {
      writeConfigMap(stream, t.c_str(), l);
      char c = l.back()[l.back().size() - 1];
      flat &= c != ')' && c != '}';
      if(i + 1 != size)
        l.back() += ",";
    }
    if(flat)
    {
      for(std::list<std::string>::const_iterator i = l.begin(); i != l.end(); ++i)
      {
        if(i != l.begin())
          s += " ";
        s += *i;
      }
      lines.push_back(s + "]");
    }
    else
    {
      lines.push_back(s);
      for(std::list<std::string>::const_iterator i = l.begin(); i != l.end(); ++i)
        lines.push_back(std::string("  ") + *i);
      lines.push_back("]");
    }
  }
  else
  {
    std::string s;
    if(*field)
      s += std::string(field) + " = ";
    std::string typeName = type;
    if(typeName.size() > 6 && typeName.substr(typeName.size() - 6) == " const")
      typeName = typeName.substr(0, typeName.size() - 6);
    const char* t = streamHandler.getString(typeName.c_str());
    StreamHandler::Specification::const_iterator i = streamHandler.specification.find(t);
    StreamHandler::BasicTypeSpecification::const_iterator b = streamHandler.basicTypeSpecification.find(t);
    StreamHandler::EnumSpecification::const_iterator e = streamHandler.enumSpecification.find(t);
    if(i != streamHandler.specification.end())
    {
      s += "{";
      std::list<std::string> l;
      bool flat = true;
      for(std::vector<StreamHandler::TypeNamePair>::const_iterator j = i->second.begin(); j != i->second.end(); ++j)
      {
        writeConfigMap(stream, j->second, l, j->first.c_str());
        char c = l.back()[l.back().size() - 1];
        flat &= c != ')' && c != '}';
        l.back() += ';';
      }
      if(root)
        for(std::list<std::string>::const_iterator i = l.begin(); i != l.end(); ++i)
          lines.push_back(*i);
      else if(flat)
      {
        for(std::list<std::string>::const_iterator i = l.begin(); i != l.end(); ++i)
        {
          if(i != l.begin())
            s += " ";
          s += *i;
        }
        lines.push_back(s + "}");
      }
      else
      {
        lines.push_back(s);
        for(std::list<std::string>::const_iterator i = l.begin(); i != l.end(); ++i)
          lines.push_back(std::string("  ") + *i);
        lines.push_back("}");
      }
    }
    else if(b != streamHandler.basicTypeSpecification.end())
    {
      if(!strcmp("double", t))
      {
        double d;
        stream >> d;
        sprintf(buf, "%lg", d);
      }
      else if(!strcmp("bool", t))
      {
        bool b;
        stream >> b;
        strcpy(buf, b ? "true" : "false");
      }
      else if(!strcmp("float", t))
      {
        float f;
        stream >> f;
        sprintf(buf, "%g", f);
      }
      else if(!strcmp("int", t))
      {
        int i;
        stream >> i;
        sprintf(buf, "%d", i);
      }
      else if(!strcmp("unsigned", t) || !strcmp("unsigned int", t))
      {
        unsigned int u;
        stream >> u;
        sprintf(buf, "%u", u);
      }
      else if(!strcmp("short", t))
      {
        short s;
        stream >> s;
        sprintf(buf, "%d", s);
      }
      else if(!strcmp("unsigned short", t))
      {
        unsigned short u;
        stream >> u;
        sprintf(buf, "%u", u);
      }
      else if(!strcmp("char", t))
      {
        char c;
        stream >> c;
        sprintf(buf, "%d", c);
      }
      else if(!strcmp("unsigned char", t))
      {
        unsigned char u;
        stream >> u;
        sprintf(buf, "%u", u);
      }
      else if(!strcmp("class std::basic_string<char,struct std::char_traits<char>,class std::allocator<char> >", t) ||
              !strcmp("std::string", t))
      {
        std::string s,
                    s2;
        stream >> s;
        int j = 0;
        bool containsSpaces = s.empty() || s[0] == '"' || s.find_first_of(" \n\r\t") != std::string::npos;
        if(containsSpaces)
          buf[j++] = '"';
        for(unsigned int i = 0; i < s.size() && j < int(sizeof(buf) - 4); ++i)
          if(s[i] == '"' && containsSpaces)
          {
            buf[j++] = '\\';
            buf[j++] = '"';
          }
          else if(s[i] == '\n')
          {
            buf[j++] = '\\';
            buf[j++] = 'n';
          }
          else if(s[i] == '\r')
          {
            buf[j++] = '\\';
            buf[j++] = 'r';
          }
          else if(s[i] == '\t')
          {
            buf[j++] = '\\';
            buf[j++] = 't';
          }
          else if(s[i] == '\\')
          {
            buf[j++] = '\\';
            buf[j++] = '\\';
          }
          else
            buf[j++] = s[i];
        if(containsSpaces)
          buf[j++] = '"';
        buf[j] = 0;
      }
      lines.push_back(s + buf);
    }
    else if(e != streamHandler.enumSpecification.end())
    {
      unsigned i;
      stream >> i;
      if(i < e->second.size())
        lines.push_back(s + e->second[i]);
      else
      {
        sprintf(buf, "%d", i);
        lines.push_back(s + buf);
      }
    }
    else
      lines.push_back(s + "UNKNOWN");
  }
}

bool RobotConsole::acceptCamera(In& stream)
{
  std::string option;
  stream >> option;
  if(option == "lower")
    acceptedCamera = ImageInfo::lowerCamera;
  else if(option == "upper")
    acceptedCamera = ImageInfo::upperCamera;
  else if(option == "both")
    acceptedCamera = ImageInfo::bothCameras;
  else
    return false;
  return true;
}
