/*
 * DownloadStatisticsCmd.cpp
 *
 *  Created on: 29.03.2012
 *      Author: marcel
 */

#include "DownloadStatisticsCmd.h"
#include <iostream>
#include "Platform/File.h"
#include <QString>
#include <QStringList>
#include "Utils/bush/agents/PingAgent.h"
#include "Utils/bush/cmdlib/Context.h"
#include "Utils/bush/cmdlib/Commands.h"
#include "Utils/bush/cmdlib/ProcessRunner.h"
#include "Utils/bush/tools/StringTools.h"
#include "Utils/bush/tools/Platform.h"
#include "Utils/bush/Session.h"
#include "Utils/bush/models/Team.h"
#include "Utils/bush/models/Robot.h"

DownloadStatisticsCmd DownloadStatisticsCmd::theDownloadStatisticsCmd;

DownloadStatisticsCmd::DownloadStatisticsCmd()
{
  Commands::getInstance().addCommand(this);
}

std::string DownloadStatisticsCmd::getName() const
{
  return "downloadStatistics";
}

std::string DownloadStatisticsCmd::getDescription() const
{
  return "Downloads all files in Statistics and removes them from the nao";
}

QString DownloadStatisticsCmd::getCommand()
{
  return fromString(std::string(File::getBHDir()) + "/Make/" + makeDirectory() + "/downloadStatistics");
}

bool DownloadStatisticsCmd::preExecution(Context & context, const std::vector<std::string> & params)
{
  return true;
}

Task* DownloadStatisticsCmd::perRobotExecution(Context & context, Robot & robot)
{

  QString command = getCommand();

  QStringList args = QStringList();

  args.push_back(fromString(robot.name));
  args.push_back(fromString(robot.getBestIP()));

  ProcessRunner r(context, command, args);
  r.run();

  if(r.error())
    context.errorLine("Downloading the statistics failed!");
  else
    context.printLine("Success! (" + robot.name + ")");
  return NULL;
}

bool DownloadStatisticsCmd::postExecution(Context & context, const std::vector<std::string> & params)
{
  return true;
}

