#include "Platform/File.h"
#include "Utils/bush/cmdlib/ProcessRunner.h"
#include "Utils/bush/cmds/UpdateWirelessCmd.h"
#include "Utils/bush/Session.h"
#include "Utils/bush/tools/Filesystem.h"
#include "Utils/bush/tools/StringTools.h"
#include <algorithm>

//The connman profiles contain placeholders where the robots actual
//team id and part number should be.
//The team id and part number make up the last 2 pieces of the
//ip address.
const std::string TEAM_ID("%teamID%");
const std::string ROBOT_PART("%robotPart%");


UpdateWirelessCmd::UpdateWirelessTask::UpdateWirelessTask(Context &context,
                                                          Robot *robot,
                                                          const QString& keyFile)
  : RobotTask(context, robot),
    keyFile(keyFile)
{ }

bool UpdateWirelessCmd::UpdateWirelessTask::execute()
{

  bool status = true;
  
  
  QString command = "scp";
  QStringList args;
  args << "-r";
  args << "-q"; // This disables the man-in-the-middle warning
  args << "-i";
  args << keyFile;
  args << "-o";
  args << "StrictHostKeyChecking=no";
  args << fromString(std::string(File::getBHDir()) + "/Install/Robots/" + robot->name + "/Profiles/");
  args << fromString("nao@" + robot->getBestIP() + ":/home/nao/");

  ProcessRunner r(context(), command, args);
  r.run();

  if(r.error())
  {
    context().errorLine("UpdateWireless of \"" + robot->name + "\" failed!");
    status = false;
  }

  return status;
}

UpdateWirelessCmd::UpdateWirelessCmd()
{
  Commands::getInstance().addCommand(this);
}

std::string UpdateWirelessCmd::getName() const
{
  return "updateWireless";
}

std::string UpdateWirelessCmd::getDescription() const
{
  return "updates the wireless configurations on selected robots.";
}

bool UpdateWirelessCmd::preExecution(Context &context, const std::vector<std::string> &params)
{
  if(!params.empty())
  {
    context.errorLine("No parameters allowed.");
    return false;
  }

  keyFile = fromString(std::string(File::getBHDir()) + "/Config/Keys/id_rsa_nao_tmp");

  return true;
}

Task* UpdateWirelessCmd::perRobotExecution(Context &context, Robot &robot)
{
  return new UpdateWirelessTask(context, &robot, keyFile);
}

UpdateWirelessCmd UpdateWirelessCmd::theUpdateWirelessCmd;
