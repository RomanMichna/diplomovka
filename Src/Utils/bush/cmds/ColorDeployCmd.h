#pragma once

#include "Utils/bush/cmdlib/RobotCommand.h"
#include <string>

class ColorDeployCmd : public RobotCommand
{
  class ColorDeployTask : public RobotTask
  {
    std::string location;
  public:
    ColorDeployTask(Context &context,
                    Robot *robot,
                    const std::string& location);
    virtual bool execute();
  };

  std::string location;

  ColorDeployCmd();
  virtual std::string getName() const;
  virtual std::string getDescription() const;
  virtual std::vector<std::string> complete(const std::string& cmdLine) const;
  virtual bool preExecution(Context &context, const std::vector<std::string> &params);
  virtual Task* perRobotExecution(Context &context, Robot &robot);
public:
  static ColorDeployCmd theColorDeployCmd;
};
