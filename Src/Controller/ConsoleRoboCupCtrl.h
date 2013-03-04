/**
* @file Controller/ConsoleRoboCupCtrl.h
*
* This file declares the class ConsoleRoboCupCtrl.
*
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas R�fer</a>
*/

#pragma once

#include <set>

#include "RoboCupCtrl.h"
#include "Tools/ProcessFramework/TeamHandler.h"
#include "Tools/Settings.h"
#include "Tools/NTP.h"
#include "Tools/Debugging/ReleaseOptions.h"
#include "RobotConsole.h"

class VideoCtrl;
class ConsoleView;
class RemoteRobot;
class RemoteRobotWithPuppet;
class TeamRobot;

/**
* The class implements the SimRobot controller for RoboCup.
*/
class ConsoleRoboCupCtrl : public RoboCupCtrl, public MessageHandler
{
public:
  DECLARE_SYNC;
  NTP ntp; /**< The Network Time Protocol. */
  tr1::unordered_map<std::string, std::string> representationToFile;

  /**
  * Constructor.
  * @param application The interface to SimRobot.
  */
  ConsoleRoboCupCtrl(SimRobot::Application& application);

  /**
  * The function returns the mode in which the robot processes constructed shall run.
  * @return The mode for the new processes.
  */
  SystemCall::Mode getMode() const {return mode;}

  /**
  * The function returns whether the current robot constructed shall play back a log file.
  * @return Play back a log file?
  */
  const std::string& getLogFile() const {return logFile;}

  /**
  * The function is called when a console command has been entered.
  * @param command The command.
  * @param console Use this console to execute the command.
  */
  void executeConsoleCommand(std::string command, RobotConsole* console = 0);

  /**
  * The function is called when the tabulator key is pressed.
  * It can replace the given command line by a new one.
  * @param command The command.
  * @param forward Complete in forward direction.
  * @param nextSection Progress to next section in the command.
  */
  void completeConsoleCommand(std::string& command, bool forward, bool nextSection);

  /**
  * The function forces an update of the command completion table.
  */
  void updateCommandCompletion() {SYNC; completion.clear();}

  /**
  * The function prints a string into the console window.
  * @param text The text to be printed.
  */
  void print(const std::string& text);

  /**
  * The function prints a string into the console window.
  * Future text will be printed on the next line.
  * @param text The text to be printed.
  */
  void printLn(const std::string& text);

  /**
  * The function prints a text as part of a list to the console if it contains a required subtext.
  * @param text The text to print. On the console, it will be followed by a space.
  * @param required The subtext that is search for.
  * @param newLine Should the text be finished by a carriage return?
  */
  void list(const std::string& text, const std::string& required, bool newLine = false);

  /**
  * The function prints a string into the status bar.
  * @param text The text to be printed.
  */
  void printStatusText(const std::string& text);

  /**
  * The function translates a debug request string into a simplyfied version.
  * @param text The text to translate.
  * @return A string that does not contain spaces anymore.
  */
  std::string translate(const std::string& text) const;

  /**
  * The function sets the DebugRequestTable used by the command completion.
  * @param drt The new debug request table.
  */
  void setDebugRequestTable(const DebugRequestTable& drt) {debugRequestTable = &drt;}

  /**
  * The function sets the solution info used by the command completion.
  * @param moduleInfo The new solution info.
  */
  void setModuleInfo(const ModuleInfo& moduleInfo) {this->moduleInfo = &moduleInfo;}

  /**
  * The function sets the drawing manager used by the command completion.
  * @param drawingManager The new drawing manager.
  */
  void setDrawingManager(const DrawingManager& drawingManager) {this->drawingManager = &drawingManager;}

  /**
  * The function sets the drawing manager 3D used by the command completion.
  * @param drawingManager3D The new drawing manager.
  */
  void setDrawingManager3D(const DrawingManager3D& drawingManager3D) {this->drawingManager3D = &drawingManager3D;}

  /**
  * The function sets the map of image views used by the command completion.
  * @param imageViews The map of image views.
  */
  void setImageViews(const RobotConsole::Views& imageViews) {this->imageViews = &imageViews;}

  /**
  * The function sets the map of field views used by the command completion.
  * @param fieldViews The map of field views.
  */
  void setFieldViews(const RobotConsole::Views& fieldViews) {this->fieldViews = &fieldViews;}

  /**
  * The function sets the map of plot views used by the command completion.
  * @param plotViews The map of plot views.
  */
  void setPlotViews(const RobotConsole::PlotViews& plotViews) {this->plotViews = &plotViews;}

  /**
  * The function read text from the stream and prints it to the console.
  * @param stream The text stream.
  */
  void echo(In& stream);

  bool isRealtime() {return realtime;}

private:
  SystemCall::Mode mode; /**< The mode of the robot currently constructed. */
  std::string logFile; /**< States whether the current robot constructed shall play back a log file. */
  ConsoleView* consoleView; /**< The scene graph object that describes the console widget. */
  std::list<RobotConsole*> selected; /**< The currently selected simulated robot. */
  std::list<RemoteRobot*> remoteRobots; /**< The list of all remote robots. */
  std::list<TeamRobot*> teamRobots; /**< The list of all team robots. */
  std::list<std::string> textMessages; /**< A list of all text messages received in the current frame. */
  std::list<TimeControlled*> timeControlled; /**< A list of all time controlled Log- and Videoplayers during log playback. */
  std::list<VideoCtrl*> videoCtrls; /**< A list of all Videoplayers during log playback. */
  bool newLine; /**< States whether the last line of text was finished by a new line. */
  int nesting; /**< The number of recursion level during the execution of console files. */
  std::set<std::string> completion; /**< A list for command completion. */
  Settings settings; /**< The current location. */
  StreamHandler streamHandler; /**< The handler used by streams in this thread. */
  TEAM_COMM; // Listen to team communication
  unsigned timeStamp; /**< The time when the messages currently processed were received. */
  int robotNumber; /**< The number of the robot the messages of which are currently processed. */
  ReleaseOptions releaseOptions; /**< The release options sent by team communication. */
  const DebugRequestTable* debugRequestTable; /**< Points to the debug request table used for tab-completion. */
  const ModuleInfo* moduleInfo; /**< Points to the solution info used for tab-completion. */
  const DrawingManager* drawingManager; /**< Points to the drawing manager used for tab-completion. */
  const DrawingManager3D* drawingManager3D; /**< Points to the drawing manager used for tab-completion. */
  const RobotConsole::Views* imageViews, /**< Points to the map of image views used for tab-completion. */
                           * fieldViews; /**< Points to the map of field views used for tab-completion. */
  const RobotConsole::PlotViews* plotViews; /**< Points to the map of plot views used for tab-completion. */
  TimeCtrl* timeCtrl; /**< Pointer to the TimeCtrl controlling log Playback. Should be 0 if not used. */
  bool realtime; /**< Controls whether the Log should be replayed in realtime or as fast as possible. */

  /**
  * The function executes the specified file.
  * @param name The file to execute.
  * @param printError Print error message if file is not found.
  * @param console Use this console to execute all commands in the file.
  */
  void executeFile(std::string name, bool printError, RobotConsole* console);

  /**
  * The function adds a robot with a certain name to the set of selected robots.
  * @param name The name of the robot.
  * @return Does a robot with the specified name exist?
  */
  bool selectRobot(const std::string& name);

  /**
  * The function prints a help text.
  * @param stream The text stream.
  */
  void help(In& stream);

  /**
  * The function handles the console input for the "sc" command.
  * @param stream The stream containing the parameters of "sc".
  * @return Returns true if the parameters were correct.
  */
  bool startRemote(In& stream);

  /**
  * The function handles the console input for the "scwp" command.
  * @param stream The stream containing the parameters of "scwp".
  * @return Returns true if the parameters were correct.
  */
  bool startRemoteRobotWithPuppet(In& stream);

  /**
  * The function handles the console input for the "sl" command.
  * @param stream The stream containing the parameters of "sl".
  * @return Returns true if the parameters were correct.
  */
  bool startLogFile(In& stream);

  /**
  * The function handles the console input for the "su" command.
  * @param stream The stream containing the parameters of "su".
  * @return Returns true if the parameters were correct.
  */
  bool startTeamRobot(In& stream);

  /**
  * The function handles the console input for the "sv" command.
  * @param stream The stream containing the parameters of "sv".
  * @return Returns true if the parameters were correct.
  */
  bool startVideo(In& stream);

  /**
  * The function handles the console input for the "stc" command.
  * @param stream The stream containing the parameters of "stc".
  * @return Returns true if the parameters were correct.
  */
  bool startTimeCtrl(In& stream);

  /**
  * The function handles the console input for the "ro" command.
  * @param stream The stream containing the parameters of "ro".
  * @return Returns true if the parameters were correct.
  */
  bool setReleaseOptions(In& stream);

  /**
  * The function creates the map for command completion.
  */
  void createCompletion();

  /**
  * The function adds the tab-completion entries for a command followed by a file name.
  * @param command The command.
  * @param pattern The pattern for the files following the command. The pattern may include a path.
  */
  void addCompletionFiles(const std::string& command, const std::string& pattern);

  /**
  * The method is called for every incoming team message.
  * @param message An interface to read the message from the queue.
  * @return true Was the message handled?
  */
  bool handleMessage(InMessage& message);

  /**
  * Destructor.
  */
  virtual ~ConsoleRoboCupCtrl();

  /**
  * The function is called to initialize the module.
  */
  virtual bool compile();

  /**
  * The function is called to create connections to other scene graph objects from other modules.
  */
  virtual void link();

  /**
  * The function is called from SimRobot in each simulation step.
  */
  virtual void update();

  /**
  * The function is called from SimRobot when Shift+Ctrl+letter was pressed or released.
  * @param key 0..9 for the keys 0..9, 10 for the decimal point, and
  * above for Shift+Ctrl+A-Z.
  * @param pressed Whether the key was pressed or released.
  */
  virtual void pressedKey(int key, bool pressed);

  /**
  * The function is called when a movable object has been selected.
  * @param obj The object.
  */
  virtual void selectedObject(const SimRobot::Object& obj);

  /**
  * Show dialog and replace ${}-substrings with user input.
  * ${Robot name:} for text field,
  * ${Robot:,Rajesh Sheldon} for selection.
  */
  void showInputDialog(std::string& command);
};
