#include "Utils/bush/cmds/UpdateSettingsCmd.h"
#include "Utils/bush/cmdlib/Context.h"
#include "Utils/bush/cmdlib/ProcessRunner.h"
#include "Utils/bush/models/Robot.h"
#include "Utils/bush/Session.h"
#include "Utils/bush/tools/StringTools.h"
#include "Platform/File.h"

UpdateSettingsCmd UpdateSettingsCmd::theUpdateSettingsCmd;

UpdateSettingsCmd::UpdateSettingsCmd()
{
  Commands::getInstance().addCommand(this);
}

std::string UpdateSettingsCmd::getName() const
{
  return "updateSettings";
}

std::string UpdateSettingsCmd::getDescription() const
{
  return "Updates settings on selected robots.";
}

bool UpdateSettingsCmd::preExecution(Context &context, const std::vector<std::string> &params)
{
  keyFile = fromString(std::string(File::getBHDir()) + "/Config/Keys/id_rsa_nao_tmp");

  team = context.getSelectedTeam();
  if(!team)
  {
    context.errorLine("No team selected.");
    return false;
  }
  if(params.size() > 1)
  {
    context.errorLine("Too many locations");
    return false;
  }
  location = fromString(team->location);
  if(!params.empty())
    location = fromString(params[0]);

  return true;
}

Task* UpdateSettingsCmd::perRobotExecution(Context &context, Robot &robot)
{
  return new UpdateSettingsTask(context, &robot, *team, keyFile, location);
}

UpdateSettingsCmd::UpdateSettingsTask::UpdateSettingsTask(Context &context,
                                                          Robot *robot,
                                                          const Team& team,
                                                          const QString& keyFile,
                                                          const QString& location)
  : RobotTask(context, robot),
    team(team),
    keyFile(keyFile),
    location(location)
{ }

bool UpdateSettingsCmd::UpdateSettingsTask::execute()
{
  QString command = "ssh";
  QStringList args;
  args << "-q"; // This disables die man-in-the-middle warning
  args << "-i";
  args << keyFile;
  args << "-o";
  args << "StrictHostKeyChecking=no";
  args << fromString("nao@" + robot->getBestIP());
  args << "cat" << "-" << ">" << "/home/nao/Config/settings.cfg; sync";

  QByteArray data(robot->getSettingsString(team).c_str());
  RemoteWriteProcessRunner r(context(), command, args, data);
  r.run();

  if(r.error())
  {
    context().errorLine("UpdateSettings of \"" + robot->name + "\" failed!");
    return false;
  }
  else
    context().printLine("UpdateSettings of \"" + robot->name + "\" succesful!");

  return true;
}
