/**
 * @file Controller/ConsoleRoboCupCtrl.cpp
 *
 * This file implements the class ConsoleRoboCupCtrl.
 *
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include <QDir>
#include <QInputDialog>

#include "ConsoleRoboCupCtrl.h"
#include "LocalRobot.h"
#include "Controller/Views/ConsoleView.h"
#include "Platform/SimRobotQt/Robot.h"
#include "RemoteRobot.h"
#include "TeamRobot.h"
#include "SimRobotEditor.h"
#include "VideoCtrl.h"
#include "Platform/File.h"

ConsoleRoboCupCtrl::ConsoleRoboCupCtrl(SimRobot::Application& application) : RoboCupCtrl(application), mode(SystemCall::simulatedRobot),
  INIT_TEAM_COMM,
  timeStamp(0),
  robotNumber(-1),
  realtime(false)
{
  debugRequestTable = 0;
  moduleInfo = 0;
  drawingManager = 0;
  imageViews = 0;
  fieldViews = 0;
  plotViews = 0;
  newLine = true;
  nesting = 0;
  timeCtrl = 0;

  consoleView = new ConsoleView("Console", *this);
  addView(consoleView, 0, SimRobot::Flag::verticalTitleBar);
  addView(new ConsoleView("Console.Pad", *this, true), consoleView, SimRobot::Flag::verticalTitleBar);

  representationToFile["parameters:BodyContourProvider"] = "bodyContour.cfg";
  representationToFile["representation:CameraCalibration"] = "cameraCalibration.cfg";
  representationToFile["representation:CameraSettings"] = "cameraSettings.cfg";
  representationToFile["representation:JointCalibration"] = "jointCalibration.cfg";
  representationToFile["representation:MassCalibration"] = "massCalibration.cfg";
  representationToFile["representation:PassParameters"] = "passParameters.cfg";
  representationToFile["representation:RobotDimensions"] = "robotDimensions.cfg";
  representationToFile["representation:SensorCalibration"] = "sensorCalibration.cfg";
  representationToFile["module:ExpGroundContactDetector2:parameters"] = "expGroundContact2.cfg";
}

bool ConsoleRoboCupCtrl::compile()
{
  if(!RoboCupCtrl::compile())
    return false;

  if(!robots.empty())
    selected.push_back((*robots.begin())->getRobotProcess());

  start();
  Global::theStreamHandler = &streamHandler;

  std::string fileName = application->getFilePath().toAscii().constData();
  int p = fileName.find_last_of("\\/"),
      p2 = fileName.find_last_of(".");
  if(p2 > p)
    fileName = fileName.substr(0, p2);
  executeFile(fileName.c_str(), false, 0);

  for(std::list<Robot*>::iterator i = robots.begin(); i != robots.end(); ++i)
    (*i)->getRobotProcess()->handleConsole("endOfStartScript");
  for(std::list<RemoteRobot*>::iterator i = remoteRobots.begin(); i != remoteRobots.end(); ++i)
    (*i)->handleConsole("endOfStartScript");
  for(std::list<TeamRobot*>::iterator i = teamRobots.begin(); i != teamRobots.end(); ++i)
    (*i)->handleConsole("endOfStartScript");
  Global::theStreamHandler = &streamHandler;
  return true;
}

void ConsoleRoboCupCtrl::link()
{
  SimRobotEditor::Editor* editor = (SimRobotEditor::Editor*)application->resolveObject("Editor");
  if(editor)
  {
    QFileInfo fileInfo(application->getFilePath());
    editor->addFile(fileInfo.path() + "/" + fileInfo.baseName() + ".con", "call[ ]+([\\\\/a-z0-9\\.\\-_]+)");

    SimRobotEditor::Editor* kicksFolder = editor->addFolder("Kicks");
    QString configDir = QFileInfo(fileInfo.dir().path()).dir().path();
    for(int i = 1; i < WalkRequest::numOfKickTypes; i += 2)
    {
      char filePath[256];
      sprintf(filePath, "/Kicks/%s.cfg", WalkRequest::getName(WalkRequest::KickType(i)));
      kicksFolder->addFile(configDir + filePath, "");
    }
  }
}

ConsoleRoboCupCtrl::~ConsoleRoboCupCtrl()
{
  delete timeCtrl;

  for(std::list<RemoteRobot*>::iterator i = remoteRobots.begin(); i != remoteRobots.end(); ++i)
    (*i)->announceStop();
  for(std::list<TeamRobot*>::iterator i = teamRobots.begin(); i != teamRobots.end(); ++i)
    (*i)->announceStop();

  for(std::list<RemoteRobot*>::iterator i = remoteRobots.begin(); i != remoteRobots.end(); ++i)
    delete *i;
  for(std::list<TeamRobot*>::iterator i = teamRobots.begin(); i != teamRobots.end(); ++i)
    delete *i;

  for(std::list<VideoCtrl*>::iterator i = videoCtrls.begin(); i != videoCtrls.end(); ++i)
    delete *i;

  stop();
  mode = SystemCall::simulatedRobot;
}

void ConsoleRoboCupCtrl::update()
{
  {
    SYNC;
    for(std::list<std::string>::const_iterator i = textMessages.begin(); i != textMessages.end(); ++i)
      if(*i == "_cls")
        consoleView->clear();
      else if(newLine || &*i != &*textMessages.rend())
        consoleView->printLn(i->c_str());
      else
        consoleView->print(i->c_str());
    textMessages.clear();
  }
  RoboCupCtrl::update();

  for(std::list<RemoteRobot*>::iterator i = remoteRobots.begin(); i != remoteRobots.end(); ++i)
    (*i)->update();

  Global::theStreamHandler = &streamHandler;
  ntp.doSynchronization(SystemCall::getRealSystemTime(), theTeamSender.out);
  theTeamHandler.send();
  theTeamHandler.receive();
  theTeamReceiver.handleAllMessages(*this);
  theTeamReceiver.clear();
  for(std::list<TeamRobot*>::iterator i = teamRobots.begin(); i != teamRobots.end(); ++i)
    (*i)->update();
  Global::theStreamHandler = &streamHandler;

  application->setStatusMessage(statusText.c_str());

  if(completion.empty())
    createCompletion();
}

void ConsoleRoboCupCtrl::executeFile(std::string name, bool printError, RobotConsole* console)
{
  if(nesting == 10)
    printLn("Nesting Error");
  else
  {
    ++nesting;
    if((int) name.rfind('.') <= (int) name.find_last_of("\\/"))
      name = name + ".con";
    if(name[0] != '/' && name[0] != '\\' && (name.size() < 2 || name[1] != ':'))
      name = std::string("Scenes\\") + name;
    InBinaryFile stream(name.c_str());
    if(!stream.exists())
    {
      if(printError)
        printLn(name + " not found");
    }
    else
    {
      std::string line;
      while(!stream.eof())
      {
        line.resize(0);
        while(!stream.eof())
        {
          char c[2] = " ";
          stream >> c[0];
          if(c[0] == '\n')
            break;
          else if(c[0] != '\r')
            line = line + c;
        }
        if(line.find_first_not_of(" ") != std::string::npos)
          executeConsoleCommand(line, console);
      }
    }
    --nesting;
  }
}

void ConsoleRoboCupCtrl::selectedObject(const SimRobot::Object& obj)
{
  const QString& fullName = obj.getFullName();
  std::string robotName = fullName.mid(fullName.lastIndexOf('.') + 1).toAscii().constData();
  for(std::list<Robot*>::iterator i = robots.begin(); i != robots.end(); ++i)
    if(robotName == (*i)->getName())
    {
      selected.clear();
      selected.push_back((*i)->getRobotProcess());
      printLn(std::string("robot ") + (*i)->getName());
      return;
    }
}

void ConsoleRoboCupCtrl::pressedKey(int key, bool pressed)
{
  if(key > 10)
    for(std::list<RobotConsole*>::iterator i = selected.begin(); i != selected.end(); ++i)
      (*i)->handleKeyEvent(key - 11, pressed);
}

void ConsoleRoboCupCtrl::executeConsoleCommand(std::string command, RobotConsole* console)
{
  showInputDialog(command);
  std::string buffer;
  InConfigMemory stream(command.c_str(), command.size());
  stream >> buffer;
  if(buffer == "") // comment
    return;
  else if(buffer == "help" || buffer == "?")
    help(stream);
  else if(buffer == "robot")
  {
    stream >> buffer;
    if(buffer == "?")
    {
      for(std::list<Robot*>::iterator i = robots.begin(); i != robots.end(); ++i)
        print(std::string((*i)->getName()) + " ");
      for(std::list<RemoteRobot*>::iterator i = remoteRobots.begin(); i != remoteRobots.end(); ++i)
        print(std::string((*i)->getName()) + " ");
      for(std::list<TeamRobot*>::iterator i = teamRobots.begin(); i != teamRobots.end(); ++i)
        print(std::string((*i)->getName()) + " ");
      printLn("");
      return;
    }
    else if(buffer == "all")
    {
      selected.clear();
      for(std::list<Robot*>::iterator i = robots.begin(); i != robots.end(); ++i)
        selected.push_back((*i)->getRobotProcess());
      for(std::list<RemoteRobot*>::iterator i = remoteRobots.begin(); i != remoteRobots.end(); ++i)
        selected.push_back(*i);
      for(std::list<TeamRobot*>::iterator i = teamRobots.begin(); i != teamRobots.end(); ++i)
        selected.push_back(*i);
      return;
    }
    else
    {
      selected.clear();
      for(;;)
      {
        if(!selectRobot(buffer))
          break;
        else if(stream.getEof())
          return;
        stream >> buffer;
      }
    }
    printLn("Syntax Error");
  }
  else if(buffer == "ar")
  {
    stream >> buffer;
    if(buffer == "on")
      gameController.automatic = true;
    else if(buffer == "off")
      gameController.automatic = false;
    else
      printLn("Syntax Error");
  }
  else if(buffer == "call")
  {
    stream >> buffer;
    executeFile(buffer, true, console);
  }
  else if(buffer == "ro")
  {
    if(!setReleaseOptions(stream))
      printLn("Syntax Error");
  }
  else if(buffer == "st")
  {
    stream >> buffer;
    if(buffer == "on")
    {
      if(!simTime)
      {
        // simulation time continues at real time
        time = getTime();
        simTime = true;
      }
    }
    else if(buffer == "off")
    {
      if(simTime)
      {
        // real time contiues at simulation time
        time = getTime() - SystemCall::getRealSystemTime();
        simTime = false;
      }
    }
    else
      printLn("Syntax Error");
  }
  else if(buffer == "dt")
  {
    stream >> buffer;
    if(buffer == "on")
      dragTime = true;
    else if(buffer == "off")
      dragTime = false;
    else
      printLn("Syntax Error");
  }
  else if(buffer == "gc")
  {
    if(!gameController.handleGlobalConsole(stream))
      printLn("Syntax Error");
  }
  else if(buffer == "sc")
  {
    if(!startRemote(stream))
    {
      selected.clear();
      if(!robots.empty())
        selected.push_back((*robots.begin())->getRobotProcess());
    }
  }
  else if(buffer == "sl")
  {
    if(!startLogFile(stream))
      printLn("Logfile not found!");
  }
  else if(buffer == "su")
    startTeamRobot(stream);
  else if(buffer == "sv")
  {
    if(!startVideo(stream))
      printLn("Video could not be opened!");
  }
  else if(buffer == "stc")
  {
    startTimeCtrl(stream);
  }
  else if(buffer == "tc")
  {
    int port;
    std::string subnet;
    stream >> port >> subnet;
    if(subnet == "")
      subnet = "255.255.255.255";
    if(port)
      theTeamHandler.start(port, subnet.c_str());
  }
  else if(buffer == "tc2")
  {
    // ignore this command (it is used in the TeamComm3D scene)
  }
  else if(selected.empty())
    if(buffer == "cls")
      printLn("_cls");
    else if(buffer == "echo")
      echo(stream);
    else
      printLn("No robot selected!");
  else if(console)
  {
    console->handleConsole(command);
    Global::theStreamHandler = &streamHandler;
  }
  else
  {
    for(std::list<RobotConsole*>::iterator i = selected.begin(); i != selected.end(); ++i)
      (*i)->handleConsole(command);
    Global::theStreamHandler = &streamHandler;
  }
  if(completion.empty())
    createCompletion();
}

bool ConsoleRoboCupCtrl::selectRobot(const std::string& name)
{
  for(std::list<Robot*>::iterator i = robots.begin(); i != robots.end(); ++i)
    if(name == (*i)->getName())
    {
      selected.push_back((*i)->getRobotProcess());
      return true;
    }
  for(std::list<RemoteRobot*>::iterator i = remoteRobots.begin(); i != remoteRobots.end(); ++i)
    if(name == (*i)->getName())
    {
      selected.push_back(*i);
      return true;
    }

  return false;
}

void ConsoleRoboCupCtrl::help(In& stream)
{
  std::string pattern;
  stream >> pattern;
  list("Initialization commands:", pattern, true);
  list("  sc <name> [<a.b.c.d>] : Starts a TCP connection to a remote robot.", pattern, true);
  list("  sl <name> <file> : Starts a robot reading its inputs from a log file.", pattern, true);
  list("  ssl : Provides ground truth data from the local SSL server via team communication.", pattern, true);
  list("  su <name> <number> : Starts a UDP team connection to a remote robot with a certain player number. Requires command tc.", pattern, true);
  list("  tc <port> <subnet>: Listen to team communication on given port and broadcast to a subnet.", pattern, true);
  list("Global commands:", pattern, true);
  list("  ar off | on : Switches automatic referee on or off.", pattern, true);
  list("  call <file> : Execute a script file.", pattern, true);
  list("  cls : Clear console window.", pattern, true);
  list("  dt off | on : Switch simulation dragging to realtime on or off.", pattern, true);
  list("  echo <text> : Print text into console window. Useful in console.con.", pattern, true);
  list("  gc initial | ready | set | playing | finished | kickOffBlue | kickOffRed | outByBlue | outByRed : Set GameController state.", pattern, true);
  list("  help | ? [<pattern>] : Display this text.", pattern, true);
  list("  robot ? | all | <name> {<name>} : Connect console to a set of active robots. Alternatively, double click on robot.", pattern, true);
  list("  ro stopwatch ( off | <letter> ) | ( sensorData | robotHealth | motionRequest | linePercept | plotBallTimes | combinedWorldModel | freePartOfOpponentGoal ) ( off | on ) : Set release options sent by team communication.", pattern, true);
  list("  st off | on : Switch simulation of time on or off.", pattern, true);
  list("  # <text> : Comment.", pattern, true);
  list("Robot commands:", pattern, true);
  list("  ac both | lower | upper : Only accept images and drawings from the selected camera.", pattern, true);
  list("  bc <red%> <green%> <blue%> : Set the background color of all 3-D views.", pattern, true);
  list("  bike : Adds the BIKE view.", pattern, true);
  list("  ci off | on | <fps> : Switch the calculation of images on or off or activate it and set the frame rate.", pattern, true);
  list("  ct off | on | undo | <color> [<hMin> <hMax> <sMin> <sMax> <iMin> <iMax>] | load <file> | save <file> | send [<ms> | off] | clear [<color>] | imageRadius <number> | shrink [<color>] | grow [<color>] | colorSpaceRadius <number> | sendAndWrite | smart [off] | hash : Modify color table.", pattern, true);
  list("  dr ? [<pattern>] | off | <key> ( off | on | once ) : Send debug request.", pattern, true);
  list("  get ? [<pattern>] | <key> [?]: Show debug data or show its specification.", pattern, true);
  list("  jc hide | show | motion <num> <command> | ( press | release ) <button> <command> : Set joystick motion (use $1 .. $6) or button command.", pattern, true);
  list("  jm <axis> ( off | <button> <button> ) : Map two buttons on an axis.", pattern, true);
  list("  js <axis> <speed> <threshold> [<center>] : Set axis maximum speed and ignore threshold for \"jc motion <num>\" commands.", pattern, true);
  list("  log start | stop | clear | save <file> | full | jpeg : Record log file and (de)activate image compression.", pattern, true);
  list("  log saveImages (raw) <file> : Save images from log.", pattern, true);
  list("  log ? | load <file> | ( keep | remove ) <message> {<message>} : Load, filter, and display information about log file.", pattern, true);
  list("  log start | pause | stop | forward [image] | backward [image] | repeat | goto <number> | cycle | once | fast_forward | fast_rewind : Replay log file.", pattern, true);
  list("  mof : Recompile motion net and send it to the robot. ", pattern, true);
  list("  msg off | on | log <file> | enable | disable : Switch output of text messages on or off. Log text messages to a file. Switch message handling on or off.", pattern, true);
  list("  mr ? [<pattern>] | modules [<pattern>] | save | <representation> ( ? [<pattern>] | <module> | off ) : Send module request.", pattern, true);
  list("  mv <x> <y> <z> [<rotx> <roty> <rotz>] : Move the selected simulated robot to the given position.", pattern, true);
  list("  mvb <x> <y> <z> : Move the ball to the given position.", pattern, true);
  list("  poll : Poll for all available debug requests and drawings. ", pattern, true);
  list("  pr none | ballHolding | playerPushing | inactivePlayer | illegalDefender | leavingTheField | playingWithHands | requestForPickup : Penalize robot.", pattern, true);
  list("  qfr queue | replace | reject | collect <seconds> | save [<seconds>] : Send queue fill request.", pattern, true);
  list("  set ? [<pattern>] | <key> ( ? | unchanged | <data> ) : Change debug data or show its specification.", pattern, true);
  list("  save ? [<pattern>] | <key> [<path>] : Save debug data to a configuration file.", pattern, true);
  list("  si reset | [number] [both] <file> : Save current image(s).", pattern, true);
  list("  v3 ? [<pattern>] | <image> [jpeg] [<name>] : Add a set of 3-D views for a certain image.", pattern, true);
  list("  vf <name> : Add field view.", pattern, true);
  list("  vfd ? [<pattern>] | <name> ( ? [<pattern>] | <drawing> ( on | off ) ) : (De)activate debug drawing in field view.", pattern, true);
  list("  vi ? [<pattern>] | <image> [jpeg] [segmented] [<name>] [gain <value>] : Add image view.", pattern, true);
  list("  vid ? [<pattern>] | <name> ( ? [<pattern>] | <drawing> ( on | off ) ) : (De)activate debug drawing in image view.", pattern, true);
  list("  vp <name> <numOfValues> <minValue> <maxValue> [<yUnit> <xUnit> <xScale>]: Add plot view.", pattern, true);
  list("  vpd ? [<pattern>] | <name> ( ? [<pattern>] | <drawing> ( ? [<pattern>] | <color> [<description>] | off ) ) : Plot data in a certain color in plot view.", pattern, true);
  list("  wek : Send walking engine kicks to the robot. ", pattern, true);
}

void ConsoleRoboCupCtrl::echo(In& stream)
{
  bool first = true;
  while(!stream.eof())
  {
    std::string text;
    stream >> text;
    if(first)
      first = false;
    else
      print(" ");
    print(text);
  }
  printLn("");
}

bool ConsoleRoboCupCtrl::startRemote(In& stream)
{
  std::string name,
              ip;
  stream >> name >> ip;
  std::string robotName = std::string(".") + name;
  this->robotName = robotName.c_str();

  mode = SystemCall::remoteRobot;
  RemoteRobot* rr = new RemoteRobot(name.c_str(), ip.c_str());
  this->robotName = 0;
  if(!rr->isClient())
  {
    if(ip != "")
    {
      delete rr;
      printLn(std::string("No connection to ") + ip + " established!");
      return false;
    }
    else
      printLn("Waiting for a connection...");
  }
  selected.clear();
  remoteRobots.push_back(rr);
  selected.push_back(remoteRobots.back());
  rr->addViews();
  rr->start();
  return true;
}

bool ConsoleRoboCupCtrl::startLogFile(In& stream)
{
  std::string name,
              fileName;
  int offset;
  stream >> name >> fileName >> offset;
  if(int(fileName.rfind('.')) <= int(fileName.find_last_of("\\/")))
    fileName = fileName + ".log";
  if(fileName[0] != '\\' && fileName[0] != '/' && (fileName.size() < 2 || fileName[1] != ':'))
    fileName = std::string("Logs/") + fileName;
  {
    InBinaryFile test(fileName);
    if(!test.exists())
      return false;
  }
  std::string robotName = std::string(".") + name;
  mode = SystemCall::logfileReplay;
  logFile = fileName;
  this->robotName = robotName.c_str();
  robots.push_back(new Robot(name.c_str()));
  this->robotName = 0;
  logFile = "";
  selected.clear();
  RobotConsole* rc = robots.back()->getRobotProcess();
  selected.push_back(rc);
  rc->getLogPlayer().offset = offset;
  timeControlled.push_back(&(rc->getLogPlayer()));
  robots.back()->start();
  return true;
}

bool ConsoleRoboCupCtrl::startTeamRobot(In& stream)
{
  std::string name;
  int number;
  stream >> name >> number;
  if(!number)
    return false;

  std::string robotName = std::string(".") + name;
  this->robotName = robotName.c_str();
  TeamRobot* tr = new TeamRobot(name.c_str(), number);
  this->robotName = 0;
  teamRobots.push_back(tr);
  selected.clear();
  selected.push_back(teamRobots.back());
  tr->addViews();
  tr->start();
  return true;
}

bool ConsoleRoboCupCtrl::startVideo(In& stream)
{
  std::string name,
              fileName;
  int offset;
  stream >> name >> fileName >> offset;
  if(fileName[0] != '\\' && fileName[0] != '/' && (fileName.size() < 2 || fileName[1] != ':'))
    fileName = File::getBHDir() + std::string("/Config/Logs/") + fileName;
  {
    InBinaryFile test(fileName);
    if(!test.exists())
      return false;
  }
  VideoCtrl* vc = new VideoCtrl(name.c_str(), fileName.c_str());
  vc->offset = offset;
  vc->addViews();
  vc->play();
  timeControlled.push_back(vc);
  videoCtrls.push_back(vc);
  return true;
}

bool ConsoleRoboCupCtrl::startTimeCtrl(In& stream)
{
  // starting multiple timeCtrls is a bad thing.
  if(timeCtrl != 0)
  {
    printLn(std::string("Cannot start multiple TimeCtrls"));
    return false;
  }
  std::string name;
  stream >> name;
  timeCtrl = new TimeCtrl(name.c_str(), timeControlled);
  timeCtrl->addViews();
  realtime = true;
  return true;
}

bool ConsoleRoboCupCtrl::setReleaseOptions(In& stream)
{
  std::string option,
              state;
  stream >> option >> state;
  if(option == "stopwatch")
    if(state == "off")
      releaseOptions.stopwatch = 0;
    else if(state == "")
      return false;
    else
      releaseOptions.stopwatch = state[0];
  else if(option == "sensorData")
    if(state == "on" || state == "")
      releaseOptions.sensorData = true;
    else if(state == "off")
      releaseOptions.sensorData = false;
    else
      return false;
  else if(option == "robotHealth")
    if(state == "on" || state == "")
      releaseOptions.robotHealth = true;
    else if(state == "off")
      releaseOptions.robotHealth = false;
    else
      return false;
  else if(option == "motionRequest")
    if(state == "on" || state == "")
      releaseOptions.motionRequest = true;
    else if(state == "off")
      releaseOptions.motionRequest = false;
    else
      return false;
  else if(option == "linePercept")
    if(state == "on" || state == "")
      releaseOptions.linePercept = true;
    else if(state == "off")
      releaseOptions.linePercept = false;
    else
      return false;
  else if(option == "plotBallTimes")
    if(state == "on" || state == "")
      releaseOptions.plotBallTimes = true;
    else if(state == "off")
      releaseOptions.plotBallTimes = false;
    else
      return false;
  else if(option == "combinedWorldModel")
    if(state == "on" || state == "")
      releaseOptions.combinedWorldModel = true;
    else if(state == "off")
      releaseOptions.combinedWorldModel = false;
    else
      return false;
  else if(option == "freePartOfOpponentGoal")
    if(state == "on" || state == "")
      releaseOptions.freePartOfOpponentGoal = true;
    else if(state == "off")
      releaseOptions.freePartOfOpponentGoal = false;
    else
      return false;
  else
    return false;

  theTeamSender.out.bin << 0;
  theTeamSender.out.finishMessage(idRobot);
  theTeamSender.out.bin << releaseOptions;
  theTeamSender.out.finishMessage(idReleaseOptions);
  return true;
}

void ConsoleRoboCupCtrl::print(const std::string& text)
{
  SYNC;
  if(newLine)
    textMessages.push_back(text);
  else
    textMessages.back() += text;
  newLine = false;
}

void ConsoleRoboCupCtrl::printLn(const std::string& text)
{
  SYNC;
  if(newLine)
    textMessages.push_back(text);
  else
    textMessages.back() += text;
  newLine = true;
}

void ConsoleRoboCupCtrl::printStatusText(const std::string& text)
{
  SYNC;
  if(statusText != "")
    statusText += " | ";
  statusText += text;
}

void ConsoleRoboCupCtrl::createCompletion()
{
  const char* commands[] =
  {
    "sc",
    "sl",
    "su",
    "ssl",
    "tc",
    "ac both",
    "ac lower",
    "ac upper",
    "ar off",
    "ar on",
    "bc",
    "bike",
    "call",
    "cls",
    "ct off",
    "ct on",
    "ct undo",
    "ct send off",
    "ct sendAndWrite",
    "ct imageRadius",
    "ct colorSpaceRadius",
    "ct smart off",
    "ct hash",
    "ct training on",
    "ct training off",
    "ct reclassify",
    "ct auto on",
    "ct auto off",
    "ct kd undo hand",
    "ct kd undo auto",
    "ct kd save",
    "ct kd load",
    "ct kd clear all",
    "ct kd clear auto",
    "ct kd clear hand",
    "ct kd replace",
    "ct kd range",
    "ct kd neighbors",
    "dr off",
    "echo",
    "help",
    "jc motion",
    "jc hide",
    "jc show",
    "jc press",
    "jc release",
    "js",
    "st off",
    "st on",
    "dt off",
    "dt on",
    "ci off",
    "ci on",
    "log start",
    "log stop",
    "log save",
    "log saveImages",
    "log saveImages raw",
    "log clear",
    "log full",
    "log jpeg",
    "log ?",
    "log mr",
    "log mr list",
    "log load",
    "log cycle",
    "log once",
    "log pause",
    "log forward",
    "log backward",
    "log repeat",
    "log goto",
    "log fast_forward",
    "log fast_rewind",
    "mof",
    "mr modules",
    "mr save",
    "msg off",
    "msg on",
    "msg log",
    "msg enable",
    "msg disable",
    "mv",
    "mvb",
    "poll",
    "qfr queue",
    "qfr replace",
    "qfr reject",
    "qfr collect",
    "qfr save",
    "ro sensorData off",
    "ro sensorData on",
    "ro stopwatch off",
    "ro robotHealth off",
    "ro robotHealth on",
    "ro motionRequest off",
    "ro motionRequest on",
    "ro linePercept off",
    "ro linePercept on",
    "ro plotBallTimes off",
    "ro plotBallTimes on",
    "ro combinedWorldModel off",
    "ro combinedWorldModel on",
    "ro freePartOfOpponentGoal off",
    "ro freePartOfOpponentGoal on",
    "robot all",
    "si number both",
    "si reset",
    "v3 image jpeg",
    "vf",
    "vr",
    "vi none",
    "vi image jpeg segmented",
    "vi image segmented",
    "vi imageOther jpeg segmented",
    "vi imageOther segmented",
    "wek"
  };

  SYNC;
  completion.clear();
  const int num = sizeof(commands) / sizeof(commands[0]);
  for(int i = 0; i < num; ++i)
    completion.insert(commands[i]);

  for(std::list<Robot*>::iterator i = robots.begin(); i != robots.end(); ++i)
    completion.insert(std::string("robot ") + (*i)->getName());
  for(std::list<RemoteRobot*>::iterator i = remoteRobots.begin(); i != remoteRobots.end(); ++i)
    completion.insert(std::string("robot ") + (*i)->getName());
  for(std::list<TeamRobot*>::iterator i = teamRobots.begin(); i != teamRobots.end(); ++i)
    completion.insert(std::string("robot ") + (*i)->getName());

  for(int i = 1; i < numOfMessageIDs; ++i)
    {
      completion.insert(std::string("log keep ") + getName(MessageID(i)));
      completion.insert(std::string("log remove ") + getName(MessageID(i)));
    }

  for(int color = ColorClasses::none; color < ColorClasses::numOfColors; ++color)
  {
    completion.insert(std::string("ct ") + translate(ColorClasses::getName((ColorClasses::Color) color)) + " add");
    completion.insert(std::string("ct ") + translate(ColorClasses::getName((ColorClasses::Color) color)) + " replace");
    completion.insert(std::string("ct clear ") + translate(ColorClasses::getName((ColorClasses::Color) color)));
    completion.insert(std::string("ct mask ") + translate(ColorClasses::getName((ColorClasses::Color) color)));
    completion.insert(std::string("ct grow ") + translate(ColorClasses::getName((ColorClasses::Color) color)));
    completion.insert(std::string("ct shrink ") + translate(ColorClasses::getName((ColorClasses::Color) color)));
    completion.insert(std::string("ct kd clear ") + translate(ColorClasses::getName((ColorClasses::Color) color)));
    completion.insert(std::string("ct kd clear hand ") + translate(ColorClasses::getName((ColorClasses::Color) color)));
    completion.insert(std::string("ct kd clear auto ") + translate(ColorClasses::getName((ColorClasses::Color) color)));
    completion.insert(std::string("ct kd replace ") + translate(ColorClasses::getName((ColorClasses::Color) color)));

    for(int innerColor = ColorClasses::none; innerColor < ColorClasses::numOfColors; ++innerColor)
    {
      completion.insert(std::string("ct kd replace ")
                        + translate(ColorClasses::getName((ColorClasses::Color) color))
                        + " "
                        + translate(ColorClasses::getName((ColorClasses::Color) innerColor)));
    }
  }

  addCompletionFiles("log load ", std::string(File::getBHDir()) + "/Config/Logs/*.log");
  addCompletionFiles("log save ", std::string(File::getBHDir()) + "/Config/Logs/*.log");
  addCompletionFiles("call ", std::string(File::getBHDir()) + "/Config/Scenes/*.con");
  addCompletionFiles("ct load ", std::string(File::getBHDir()) + "/Config/" + settings.expandLocationFilename("*.c64"));
  addCompletionFiles("ct save ", std::string(File::getBHDir()) + "/Config/" + settings.expandLocationFilename("*.c64"));
  addCompletionFiles("ct kd load ", std::string(File::getBHDir()) + "/Config/" + settings.expandLocationFilename("*.kdt"));
  addCompletionFiles("ct kd save ", std::string(File::getBHDir()) + "/Config/" + settings.expandLocationFilename("*.kdt"));

  if(moduleInfo)
  {
    for(std::list<ModuleInfo::Provider>::const_iterator i = moduleInfo->providers.begin(); i != moduleInfo->providers.end(); ++i)
    {
      completion.insert(std::string("mr ") + i->representation + " off");
      for(std::vector<std::string>::const_iterator j = i->modules.begin(); j != i->modules.end(); ++j)
        completion.insert(std::string("mr ") + i->representation + " " + *j);
    }
  }

  if(debugRequestTable)
  {
    for(int i = 0; i < debugRequestTable->currentNumberOfDebugRequests; ++i)
    {
      completion.insert(std::string("dr ") + translate(debugRequestTable->debugRequests[i].description) + " on");
      completion.insert(std::string("dr ") + translate(debugRequestTable->debugRequests[i].description) + " off");
      completion.insert(std::string("dr ") + translate(debugRequestTable->debugRequests[i].description) + " once");
      if(debugRequestTable->debugRequests[i].description.substr(0, 13) == "debug images:")
      {
        completion.insert(std::string("v3 ") + translate(debugRequestTable->debugRequests[i].description.substr(13)) + " jpeg");
        completion.insert(std::string("vi ") + translate(debugRequestTable->debugRequests[i].description.substr(13)) + " jpeg segmented");
        completion.insert(std::string("vi ") + translate(debugRequestTable->debugRequests[i].description.substr(13)) + " segmented");
      }
      else if(debugRequestTable->debugRequests[i].description.substr(0, 11) == "debug data:")
      {
        completion.insert(std::string("vr ") + translate(debugRequestTable->debugRequests[i].description.substr(11)));

        completion.insert(std::string("get ") + translate(debugRequestTable->debugRequests[i].description.substr(11)) + " ?");
        completion.insert(std::string("set ") + translate(debugRequestTable->debugRequests[i].description.substr(11)) + " ?");
        completion.insert(std::string("set ") + translate(debugRequestTable->debugRequests[i].description.substr(11)) + " unchanged");
      }
    }
  }

  if(drawingManager)
  {
    if(imageViews)
      for(RobotConsole::Views::const_iterator v = imageViews->begin(); v != imageViews->end(); ++v)
      {
        completion.insert(std::string("cameraCalibrator ") + translate(v->first) + " on");
        completion.insert(std::string("cameraCalibrator ") + translate(v->first) + " off");
      }
    for(std::tr1::unordered_map< const char*, DrawingManager::Drawing>::const_iterator i = drawingManager->drawings.begin(); i != drawingManager->drawings.end(); ++i)
    {
      if(!strcmp(drawingManager->getDrawingType(i->first), "drawingOnImage") && imageViews)
        for(RobotConsole::Views::const_iterator j = imageViews->begin(); j != imageViews->end(); ++j)
        {
          completion.insert(std::string("vid ") + j->first + " " + translate(i->first));
        }
      else if(!strcmp(drawingManager->getDrawingType(i->first), "drawingOnField") && fieldViews)
        for(RobotConsole::Views::const_iterator j = fieldViews->begin(); j != fieldViews->end(); ++j)
          completion.insert(std::string("vfd ") + j->first + " " + translate(i->first));
    }
  }

  if(plotViews)
    for(RobotConsole::PlotViews::const_iterator i = plotViews->begin(); i != plotViews->end(); ++i)
      for(int j = 0; j < debugRequestTable->currentNumberOfDebugRequests; ++j)
        if(translate(debugRequestTable->debugRequests[j].description).substr(0, 5) == "plot:")
        {
          for(int color = 1; color < ColorClasses::numOfColors; ++color)
            completion.insert(std::string("vpd ") + i->first + " " +
                              translate(debugRequestTable->debugRequests[j].description).substr(5) + " " +
                              translate(ColorClasses::getName((ColorClasses::Color) color)));
          completion.insert(std::string("vpd ") + i->first + " " +
                            translate(debugRequestTable->debugRequests[j].description).substr(5) + " off");
        }

  for(tr1::unordered_map<std::string, std::string>::const_iterator i = representationToFile.begin();
      i != representationToFile.end(); ++i)
    completion.insert(std::string("save ") + i->first);
  
  gameController.addCompletion(completion);
}

void ConsoleRoboCupCtrl::addCompletionFiles(const std::string& command, const std::string& pattern)
{
  QString qpattern(pattern.c_str());
  qpattern.replace("\\", "/");
  const int lastSlashIdx = qpattern.lastIndexOf('/');

  QDir qdir(qpattern.left(lastSlashIdx));

  QRegExp regExp(qpattern.right(qpattern.size() - lastSlashIdx - 1).replace(".", "\\.").replace("*", ".*"));

  QStringList filenames = qdir.entryList(QDir::Files);
  for(int i = 0; i < filenames.size(); i++)
  {
    if(regExp.exactMatch(filenames[i]))
    {
      filenames[i].chop(filenames[i].size() - filenames[i].lastIndexOf("."));
      completion.insert(command + filenames[i].toAscii().constData());
    }
  }
}

void ConsoleRoboCupCtrl::completeConsoleCommand(std::string& command, bool forward, bool nextSection)
{
  SYNC;
  std::string separators = " :.";
  char endSeparator = separators[0];
  if(!nextSection && command.size() && separators.find(command[command.size() - 1]) != std::string::npos)
  {
    endSeparator = command[command.size() - 1];
    command = command.substr(0, command.size() - 1);
  }
  std::string constantPart;
  int n = command.find_last_of(separators);
  if(n != -1)
    constantPart = command.substr(0, n + 1);

  if(forward)
  {
    std::set<std::string>::const_iterator i = completion.lower_bound(command + endSeparator + 'z');
    if(i == completion.end() || constantPart != i->substr(0, constantPart.length()))
    {
      i = completion.lower_bound(constantPart);
      if(i == completion.end() || constantPart != i->substr(0, constantPart.length()))
        return;
    }
    command = i->substr(constantPart.length());
  }
  else
  {
    std::set<std::string>::const_iterator i = completion.lower_bound(command);
    if(i != completion.begin())
      --i;
    if(i == completion.begin() || constantPart != i->substr(0, constantPart.length()))
    {
      i = completion.lower_bound(constantPart + "zzzzzzzzzzz");
      --i;
      if(i == completion.begin() || constantPart != i->substr(0, constantPart.length()))
        return;
    }
    command = i->substr(constantPart.length());
  }
  while(command.length() > 0 && separators.find(command[0]) != std::string::npos)
    command = command.substr(1);
  n = command.find_first_of(separators);
  char separator2 = ' ';
  if(n == -1)
    n = command.size();
  else
    separator2 = command[n];
  command = constantPart + command.substr(0, n) + separator2;
}

void ConsoleRoboCupCtrl::list(const std::string& text, const std::string& required, bool newLine)
{
  std::string s1(text), s2(required);
  for(std::string::iterator i = s1.begin(); i != s1.end(); ++i)
    *i = toupper(*i);
  for(std::string::iterator i = s2.begin(); i != s2.end(); ++i)
    *i = toupper(*i);
  if(s1.find(s2) != std::string::npos)
  {
    if(newLine)
    {
      printLn(text);
    }
    else
    {
      print(text + " ");
    }
  }
}

std::string ConsoleRoboCupCtrl::translate(const std::string& text) const
{
  std::string s(text);
  for(unsigned i = 0; i < s.size(); ++i)
    if(s[i] == ' ' || s[i] == '-')
    {
      s = s.substr(0, i) + s.substr(i + 1);
      if(i < s.size())
      {
        if(s[i] >= 'a' && s[i] <= 'z')
          s[i] = s[i] - 32;
        --i;
      }
    }
    else if(i < s.size() - 1 && s[i] == ':' && s[i + 1] == ':')
      s = s.substr(0, i) + s.substr(i + 1);
  return s;
}

bool ConsoleRoboCupCtrl::handleMessage(InMessage& message)
{
  switch(message.getMessageID())
  {
  case idRobot:
    message.bin >> robotNumber;
    return true;

  default:
    if(!ntp.handleMessage(message))
      for(std::list<TeamRobot*>::iterator i = teamRobots.begin(); i != teamRobots.end(); ++i)
        if((*i)->getNumber() == robotNumber)
        {
          (*i)->setGlobals();
          (*i)->handleMessage(message);
          message.resetReadPosition();
        }

    return true;
  }
}


void ConsoleRoboCupCtrl::showInputDialog(std::string& command)
{
  QString qstrCommand(command.c_str());
  QRegExp re("\\$\\{([^\\}]*)\\}", Qt::CaseSensitive, QRegExp::RegExp2);
  while(re.indexIn(qstrCommand) != -1)
  {
    QStringList list = re.cap(1).split(',');
    QString label = list.takeFirst();
    QString input;
    if(list.isEmpty())
    {
      // ${Text input:}
      input = QInputDialog::getText(0, "Input", label);
    }
    else if(list.length() == 1)
    {
      QString qpattern(list.takeFirst());
      qpattern.replace("\\", "/");
      const int lastSlashIdx = qpattern.lastIndexOf('/');

      QDir qdir(qpattern.left(lastSlashIdx));

      QRegExp regExp(qpattern.right(qpattern.size() - lastSlashIdx - 1).replace(".", "\\.").replace("*", ".*"));

      QStringList filenames = qdir.entryList(QDir::Files);
      for(int i = 0; i < filenames.size(); i++)
      {
        if(regExp.exactMatch(filenames[i]))
        {
          filenames[i].chop(filenames[i].size() - filenames[i].lastIndexOf("."));
          list.append(filenames[i]);
        }
      }
      // ${Select Logfile:,../Logs/*.log}
      input = QInputDialog::getItem(0, "Input", label, list);
    }
    else
    {
      // ${Select Robot:,Leonard,Rajesh,Lenny}
      input = QInputDialog::getItem(0, "Input", label, list);
    }
    qstrCommand.replace(re.cap(0), input);
  }
  command = std::string(qstrCommand.toAscii().constData());
}
