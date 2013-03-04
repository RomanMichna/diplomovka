#include "Utils/bush/cmdlib/ProcessRunner.h"
#include "Utils/bush/tools/StringTools.h"
#include "Utils/bush/toolwrapper/Git.h"
#include "Utils/bush/Session.h"

std::string Git::getName() const
{
  return "git";
}

bool Git::publish(const std::vector<std::string>& files, const std::string& message)
{
  for(size_t i = 0; i < files.size(); i++)
  {
    ProcessRunner r("git add " + fromString(files[i]));
    r.run();
    if(r.error())
    {
      Session::getInstance().log(WARN, "Git: Adding " + files[i] + " failed.");
    }
  }

  ProcessRunner r("git commit -m \"" + fromString(message) + "\"");
  r.run();
  if(r.error())
  {
    Session::getInstance().log(WARN, "Git: Commit failed.");
    return false;
  }

  r = ProcessRunner("git push");
  r.run();
  if(r.error())
  {
    Session::getInstance().log(WARN, "Git: Could not push without errors or warnings.");
    return false;
  }

  return true;
}
