#include "Utils/bush/cmds/CommitColtableCmd.h"
#include "Utils/bush/cmdlib/Context.h"
#include "Utils/bush/cmdlib/ProcessRunner.h"
#include "Utils/bush/Session.h"
#include "Utils/bush/toolwrapper/VersionControlSystem.h"
#include "Utils/bush/tools/Filesystem.h"
#include "Utils/bush/tools/Platform.h"
#include "Utils/bush/tools/StringTools.h"
#include "Platform/File.h"

CommitColtableCmd CommitColtableCmd::theCommitColtableCmd;

CommitColtableCmd::CommitColtableCmd()
{
  Commands::getInstance().addCommand(this);
}

std::string CommitColtableCmd::getName() const
{
  return "commitColtable";
}

std::string CommitColtableCmd::getDescription() const
{
  return std::string("Commits all edited color tables of the current location ")
         + "to the remote repository. The commit message will contain the hashes.";
}

bool CommitColtableCmd::execute(Context &context, const std::vector<std::string> &params)
{
  // TODO
  // * select location (parameter?, gui?)

#ifndef MACOSX
  std::string ctHashConfig = "Debug";
#else
  std::string ctHashConfig = "Release";
#endif
  std::string ctHash = getCtHash(ctHashConfig);

  File ctHashFile(ctHash, "r");
  if(!ctHashFile.exists())
  {
    bool compileStatus = context.execute("compile " + ctHashConfig + " ctHash");
    if(!compileStatus)
      return false;
  }

  std::string location = "Default";
  std::vector<std::string> coltables = Filesystem::getColtables(location);
  std::vector<std::string> files; // *.kdt and *.c64
  std::string message = "Bushed color tables.\n\n";
  std::string pathPrefix =  linuxToPlatformPath(std::string(File::getBHDir())
                            + "/Config/Locations/" + location + "/");
  for(size_t i = 0; i < coltables.size(); i++)
  {
    const std::string& coltable = coltables[i];
    if(coltable != "coltableSim")
    {
      files.push_back(pathPrefix + coltable + ".c64");
      files.push_back(pathPrefix + coltable + ".kdt");
      QStringList args;
      args << fromString(pathPrefix + coltable + ".c64");
      ProcessRunner r(fromString(ctHash), args);
      r.run();
      std::string hash = toString(r.getOutput().split("\t")[0]);
      context.printLine("Found " + coltable + " (" + hash + ")");
    }
  }

  VersionControlSystem* vcs = Session::getInstance().getVCS(); // TODO mutex
  vcs->publish(files, message);
  return true;
}

#ifdef WIN32
std::string CommitColtableCmd::getCtHash(const std::string& ctHashConfig)
{
  return linuxToPlatformPath(std::string(File::getBHDir())
                             + "/Build/ctHash/" + platformDirectory() + "/" + ctHashConfig + "/ctHash.exe");
}
#else
std::string CommitColtableCmd::getCtHash(const std::string& ctHashConfig)
{
  return linuxToPlatformPath(std::string(File::getBHDir())
                             + "/Build/ctHash/" + platformDirectory() + "/" + ctHashConfig + "/ctHash");
}
#endif
