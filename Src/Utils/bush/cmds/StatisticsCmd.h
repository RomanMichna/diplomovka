#pragma once

#include "Utils/bush/cmdlib/Commands.h"
#include "Utils/bush/cmdlib/RobotCommand.h"

class StatisticsCmd;

class StatisticsCmd : public RobotCommand
{
  StatisticsCmd();
  virtual std::string getName() const;
  virtual std::string getDescription() const;
  virtual Task* perRobotExecution(Context &context, Robot &robot);
public:
  static StatisticsCmd theStatisticsCmd;
};
