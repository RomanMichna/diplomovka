#include "Utils/bush/cmds/SimCmd.h"
#include "Utils/bush/cmdlib/Context.h"
#include "Utils/bush/cmdlib/Commands.h"
#include "Utils/bush/cmdlib/ProcessRunner.h"
#include "Utils/bush/tools/Platform.h"
#include "Utils/bush/tools/Sleeper.h"
#include "Utils/bush/Session.h"
#include "Platform/File.h"
#include <cstdlib>

SimCmd SimCmd::theSimCmd;

SimCmd::SimCmd()
{
  Commands::getInstance().addCommand(this);
}

std::string SimCmd::getName() const
{
  return "sim";
}

std::string SimCmd::getDescription() const
{
  return "Connects to robots via simulator. Requires a Simulator built with Develop configuration.";
}

bool SimCmd::preExecution(Context &context, const std::vector<std::string> &params)
{
  const std::string buildConfig = "Develop";
  simulatorExecutable = getSimulatorExecutable(buildConfig);
  remoteRobotScene = File::getBHDir() + linuxToPlatformPath(
                       "/Config/Scenes/ScriptRemoteRobot.ros2");
  connectScript = File::getBHDir() + linuxToPlatformPath(
                    "/Config/Scenes/scriptconnect.con");

  File simFile(simulatorExecutable, "r");
  if(!simFile.exists())
  {
    bool compileStatus = Commands::getInstance().execute(&context, "compile " + buildConfig + " SimRobot");
    if(!compileStatus)
      return false;
  }

  return true;
}

Task* SimCmd::perRobotExecution(Context &context, Robot &robot)
{
  Sleeper::msleep(1000); // since the same file is used for every robot
  File* conFile = new File(connectScript, "w");
  std::string command = "sc Remote " + robot.getBestIP();
  conFile->write(command.c_str(), command.size());
  delete conFile;

  ProcessRunner r(context, simulatorExecutable + " " + remoteRobotScene);
  r.run();
  if (r.error())
    context.errorLine("Failed.");

  return 0;
}

std::string SimCmd::getSimulatorExecutable(const std::string& buildConfig)
{
  std::string simulatorExecutable = File::getBHDir() + linuxToPlatformPath(
                                      "/Build/SimRobot/" + platformDirectory() + "/" + buildConfig
                                      + "/SimRobot");
#ifdef MACOSX
  simulatorExecutable += ".app/Contents/MacOS/SimRobot";
#elif defined WIN32
  simulatorExecutable += ".exe";
#endif

  return simulatorExecutable;
}
