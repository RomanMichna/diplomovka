#include "Utils/bush/cmds/CompileCmd.h"
#include "Utils/bush/cmdlib/Context.h"
#include "Utils/bush/tools/Filesystem.h"
#include "Utils/bush/tools/StringTools.h"
#include "Utils/bush/models/Team.h"
#include "Platform/File.h"

CompileCmd CompileCmd::theCompileCmd;

CompileCmd::CompileCmd()
{
  Commands::getInstance().addCommand(this);
}

std::string CompileCmd::getName() const
{
  return "compile";
}

std::string CompileCmd::getDescription() const
{
  return "[ <config> [ <project> ] ]\nCompiles a project with a specified build configuration. [Default: Develop and Nao]";
}

std::vector<std::string> CompileCmd::complete(const std::string& cmdLine) const
{
  std::vector<std::string> commandWithArgs = split(cmdLine);

  if(commandWithArgs.size() == 1)
    return getBuildConfigs();
  else if(commandWithArgs.size() == 2 && *--cmdLine.end() != ' ')
    return getBuildConfigs(commandWithArgs[1]);
  else if(commandWithArgs.size() == 2)
    return Filesystem::getProjects("");
  else
    return Filesystem::getProjects(commandWithArgs[2]);
}

CompileCmd::CompileTask::CompileTask(Context &context,
                                     const std::string &label,
                                     const QString &command,
                                     const QStringList &args)
  : Task(context),
    r(context, command, args),
    label(label)
{ }

bool CompileCmd::CompileTask::execute()
{
  r.run();
  if (context().isCanceled())
  {
    context().cleanupFinished();
    return true;
  }
  bool status = true;
  if(r.error())
  {
    context().errorLine("Failed to compile.");
    status = false;
  }
  return status;
}

void CompileCmd::CompileTask::cancel()
{
  r.stop();
}

void CompileCmd::CompileTask::setContext(Context *context)
{
  r.setContext(*context);
  Task::setContext(context);
}

std::string CompileCmd::CompileTask::getLabel()
{
  return label;
}

#ifdef LINUX
  #define LABEL "make"
#else
  #ifdef MACOSX
    #define LABEL "xcodebuild"
  #else
    #ifdef WIN32
      #define LABEL "vcproj"
    #endif
  #endif
#endif

bool CompileCmd::execute(Context &context, const std::vector<std::string> &params)
{
  QString command = getCommand();
  QStringList args;

  if(params.size() > 2)
  {
    context.errorLine("Too many parameters specified.");
    return false;
  }
  else if(params.empty())
  {
    Team* team = context.getSelectedTeam();
    if(team && team->buildConfig.length() > 0)
      args = getParams(fromString(team->buildConfig), "Nao");
    else
      args = getParams("Develop", "Nao");
  }
  else if(params.size() == 1)
    args = getParams(fromString(params[0]), "Nao");
  else
    args = getParams(fromString(params[0]), fromString(params[1]));

  context.executeDetached(new CompileTask(context,
                                          LABEL,
                                          command,
                                          args));
  return context.waitForChildren();
}

#ifdef WIN32
QString CompileCmd::getCommand()
{
  return QString(getenv("VS100COMNTOOLS")) + "..\\IDE\\devenv.com";
}

QStringList CompileCmd::getParams(const QString& config, const QString& project)
{
  QStringList args;
  args << fromString(std::string(File::getBHDir())).replace("/", "\\") + "\\Make\\VS2010\\BHuman.sln";
  args << QString("/Build") << config << QString("/Project") << project;
  return args;
}
#else
QString CompileCmd::getCommand()
{
  return fromString("bash"); // hack for those who use 64bit systems and bash functions to set right CFLAGS
}

#ifdef MACOSX
QStringList CompileCmd::getParams(const QString& config, const QString& project)
{
  QStringList args;
  args << "-c";

  QString mke = "cd " + fromString(std::string(File::getBHDir())) + "/Make/MacOS && xcodebuild " +
                "-jobs `sysctl -n hw.ncpu` -parallelizeTargets -target " +
                project + " -configuration " + config;

  args << mke;

  return args;
}
#else
QStringList CompileCmd::getParams(const QString& config, const QString& project)
{
  QStringList args;
  args << "-c";

  QString mke = "make -C " + fromString(std::string(File::getBHDir()) + "/Make/Linux");
  mke += fromString(" CONFIG=") + config + " " + project;

  args << mke;

  return args;
}
#endif // MacOS
#endif // WIN32
