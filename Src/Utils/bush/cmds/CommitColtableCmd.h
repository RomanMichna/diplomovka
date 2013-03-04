#pragma once

#include "Utils/bush/cmdlib/CommandAdapter.h"
#include "Utils/bush/cmdlib/Commands.h"

class CommitColtableCmd : public CommandAdapter
{
  CommitColtableCmd();
  virtual std::string getName() const;
  virtual std::string getDescription() const;
  virtual bool execute(Context &context, const std::vector<std::string> &params);
  std::string getCtHash(const std::string& ctHashConfig);
public:
  static CommitColtableCmd theCommitColtableCmd;
};
