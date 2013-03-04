/*
 * ChangeBuild.cpp
 *
 *  Created on: 28.03.2012
 *      Author: marcel
 */

#include "ChangeBuildCmd.h"
#include <iostream>
#include "Platform/File.h"
#include <QString>
#include <QStringList>
#include "Utils/bush/tools/StringTools.h"
#include "Utils/bush/agents/PingAgent.h"
#include "Utils/bush/cmdlib/Context.h"
#include "Utils/bush/cmdlib/Commands.h"
#include "Utils/bush/cmdlib/ProcessRunner.h"
#include "Utils/bush/tools/StringTools.h"
#include "Utils/bush/tools/Platform.h"
#include "Utils/bush/Session.h"
#include "Utils/bush/models/Team.h"
#include "Utils/bush/models/Robot.h"
#include <algorithm>

ChangeBuildCmd ChangeBuildCmd::theChangeBuildCommand;

ChangeBuildCmd::ChangeBuildCmd()
{
  Commands::getInstance().addCommand(this);
}

std::string ChangeBuildCmd::getName() const
{
  return "changeBuild";
}

std::string ChangeBuildCmd::getDescription() const
{
  return "changes the active build to one of Operate, Release, Develop, Debug";
}

QString ChangeBuildCmd::getCommand()
{
  return fromString(std::string(File::getBHDir()) + "/Make/" + makeDirectory() + "/changebuild");
}

bool ChangeBuildCmd::preExecution(Context & context, const std::vector<std::string> & params)
{

  if(params.size() != 1)
  {
    return false;
  }

  config = params[0];
  std::transform(config.begin(), config.begin() + 1, config.begin(), ::toupper);
  if(config != "Operate" &&
      config != "Release" &&
      config != "Develop" &&
      config != "Debug")
  {
    return false;
  }

  return true;
}

Task* ChangeBuildCmd::perRobotExecution(Context & context, Robot & robot)
{

  QString command = getCommand();

  QStringList args = QStringList();

  args.push_back(fromString(config));
  args.push_back(fromString(robot.getBestIP()));

  ProcessRunner r(context, command, args);
  r.run();

  if(r.error())
    context.errorLine("Change build to " + config + " failed!");
  else
    context.printLine("Success! (" + robot.name + ")");
  return NULL;
}

bool ChangeBuildCmd::postExecution(Context & context, const std::vector<std::string> & params)
{
  config = "";
  return true;
}


