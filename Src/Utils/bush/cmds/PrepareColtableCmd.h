#pragma once

#include "Utils/bush/cmdlib/CommandAdapter.h"

class PrepareColtableCmd : public CommandAdapter
{
  PrepareColtableCmd();
  virtual std::string getName() const;
  virtual std::string getDescription() const;
  virtual bool execute(Context &context, const std::vector<std::string> &params);
public:
  static PrepareColtableCmd thePrepareColtableCmd;
};
