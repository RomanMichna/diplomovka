#include "Utils/bush/cmds/PrepareColtableCmd.h"
#include "Utils/bush/cmdlib/Context.h"
#include "Utils/bush/cmdlib/Commands.h"
#include "Utils/bush/cmdlib/ProcessRunner.h"
#include "Utils/bush/tools/Platform.h"
#include "Utils/bush/tools/Sleeper.h"
#include "Utils/bush/tools/StringTools.h"
#include "Utils/bush/Session.h"
#include "Platform/File.h"
#include <cstdlib>

PrepareColtableCmd PrepareColtableCmd::thePrepareColtableCmd;

PrepareColtableCmd::PrepareColtableCmd()
{
  Commands::getInstance().addCommand(this);
}

std::string PrepareColtableCmd::getName() const
{
  return "prepareColtable";
}

std::string PrepareColtableCmd::getDescription() const
{
  return "Deploys code to selected robots and connects to them via simulator.";
}

bool PrepareColtableCmd::execute(Context &context, const std::vector<std::string> &params)
{
  std::vector<Robot*> selectedRobots = context.getSelectedRobots();
  if(selectedRobots.empty())
  {
    context.errorLine("No robots selected.");
    return false;
  }

  bool deployStatus = context.execute("deploy");
  if(!deployStatus)
    return false;

  bool restartStatus = Commands::getInstance().execute(&context, "restart bhuman");
  if(!restartStatus)
    return false;

  Sleeper::msleep(3000);

  bool simStatus = Commands::getInstance().execute(&context, "sim");
  if(!simStatus)
    return false;

  return true;
}
