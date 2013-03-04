#include "Utils/bush/cmds/ColorDeployCmd.h"
#include "Utils/bush/cmdlib/Context.h"
#include "Utils/bush/cmdlib/ProcessRunner.h"
#include "Utils/bush/Session.h"
#include "Utils/bush/tools/Filesystem.h"
#include "Utils/bush/tools/Platform.h"
#include "Utils/bush/tools/ShellTools.h"
#include "Utils/bush/tools/StringTools.h"
#include "Controller/Platform/Directory.h"
#include "Utils/bush/cmdlib/Commands.h"
#include "Utils/bush/cmdlib/Context.h"
#include "Platform/File.h"
#include <QProcess>
#include <QStringList>

ColorDeployCmd::ColorDeployTask::ColorDeployTask(Context &context,
                                                 Robot *robot,
                                                 const std::string& location)
  : RobotTask(context, robot),
    location(location)
{ }

bool ColorDeployCmd::ColorDeployTask::execute()
{
  bool completeFailure = true;
  std::vector<std::string> colorTables = Filesystem::getColtables(location);
  for(size_t i = 0; i < colorTables.size(); i++)
  {
    // TODO config should contain endings c64 and kdt
    // TODO copy all files at once
    // TODO check if kdt file exists
    std::string file = std::string(File::getBHDir()) + "/Config/Locations/"
                       + location + "/" + colorTables[i] + ".c64";
    QString command = fromString(scpCommandToRobot(file, robot->getBestIP(),
                                 bhumandDirOnRobot + "Config/Locations/" + location));
    ProcessRunner r(context(), command);
    r.run();
    if(!r.error())
      completeFailure = false;

    file = std::string(File::getBHDir()) + "/Config/Locations/" + location
           + "/" + colorTables[i] + ".kdt";
    command = fromString(scpCommandToRobot(file, robot->getBestIP(),
                                           bhumandDirOnRobot + "Config/Locations/" + location));
    ProcessRunner r2(context(), command);
    r2.run();
    if(!r2.error())
      completeFailure = false;
  }

  if(completeFailure)
    context().errorLine("colorDeploy of \"" + robot->name + "\" failed!");
  else
    context().printLine("colorDeploy of \"" + robot->name + "\" succesful!");

  return !completeFailure;
}

ColorDeployCmd ColorDeployCmd::theColorDeployCmd;

ColorDeployCmd::ColorDeployCmd()
{
  Commands::getInstance().addCommand(this);
}

std::string ColorDeployCmd::getName() const
{
  return "colorDeploy";
}

std::string ColorDeployCmd::getDescription() const
{
  return "deploys colortables to all selected robots.";
}

std::vector<std::string> ColorDeployCmd::complete(const std::string& cmdLine) const
{
  std::vector<std::string> commandWithArgs = split(cmdLine);

  if(commandWithArgs.size() == 1)
    return Filesystem::getLocations();
  else
    return Filesystem::getLocations(commandWithArgs[1]);
}

bool ColorDeployCmd::preExecution(Context &context, const std::vector<std::string> &params)
{
  location = "Default";
  if(!params.empty())
  {
    if(params.size() > 1)
    {
      context.errorLine("You can only choose one location at the moment.");
      return false;
    }
    location = params[0];

    Directory locationDir;
    if(!locationDir.open(linuxToPlatformPath(std::string(File::getBHDir()) + "/Config/Locations/" + location)))
    {
      context.errorLine("\"" + location + "\" is no valid location.");
      return false;
    }
  }

  return true;
}

Task* ColorDeployCmd::perRobotExecution(Context &context, Robot &robot)
{
  return new ColorDeployTask(context, &robot, location);
}
