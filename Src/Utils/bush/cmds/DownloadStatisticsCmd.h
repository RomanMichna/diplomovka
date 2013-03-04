/*
 * DownloadStatisticsCmd.h
 *
 *  Created on: 29.03.2012
 *      Author: marcel
 */
#pragma once
#include "Utils/bush/cmdlib/RobotCommand.h"

class DownloadStatisticsCmd : public RobotCommand
{
public:
  DownloadStatisticsCmd();
  virtual std::string getName() const;
  virtual std::string getDescription() const;
  virtual bool preExecution(Context &context, const std::vector<std::string> &params);
  virtual Task* perRobotExecution(Context &context, Robot &robot);
  virtual bool postExecution(Context &context, const std::vector<std::string> &params);
  QString getCommand();

public:
    static DownloadStatisticsCmd theDownloadStatisticsCmd;
};

