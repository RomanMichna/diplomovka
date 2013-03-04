#include "Utils/bush/cmds/StatisticsCmd.h"
#include "Utils/bush/cmdlib/Context.h"
#include "Utils/bush/cmdlib/ProcessRunner.h"
#include "Utils/bush/tools/StringTools.h"
#include "Utils/bush/tools/ShellTools.h"
#include "Utils/bush/tools/Platform.h"
#include "Utils/bush/Session.h"
#include "Platform/File.h"
#include <cstdlib>

StatisticsCmd StatisticsCmd::theStatisticsCmd;

StatisticsCmd::StatisticsCmd()
{
  Commands::getInstance().addCommand(this);
}

std::string StatisticsCmd::getName() const
{
  return "statistics";
}

std::string StatisticsCmd::getDescription() const
{
  return "Saves the match statistics in Config/<robot>.stx.";
}

Task* StatisticsCmd::perRobotExecution(Context &context, Robot &robot)
{
  // TODO use DebugRequestCmd
  std::vector<std::string> answer = Session::getInstance().sendDebugRequest(
                                      &robot, "dr module:MatchStatisticProvider:save once");
  if(answer.empty())
  {
    context.errorLine(robot.name + ": Could not save match statistics.");
    return NULL;
  }
  else
  {
    context.printLine(robot.name + ": ");
    for(size_t i = 0; i < answer.size(); i++)
      context.printLine("\t" + answer[i]);
  }

  std::string robotIP = robot.getBestIP();
  std::string targetDir = File::getBHDir() + linuxToPlatformPath("/Config/");
  std::string file = bhumandDirOnRobot + "Config/" + robot.name + ".stx";
  std::string command = scpCommandFromRobot(file, robotIP, targetDir);
  ProcessRunner r(context, command);
  if(r.error())
    context.errorLine(robot.name + ": Could not download match statistics.");
  else
    context.printLine(robot.name + ": Downloaded match statistics");

  return NULL;
}
