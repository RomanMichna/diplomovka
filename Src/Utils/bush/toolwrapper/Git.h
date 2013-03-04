#pragma once

#include "Utils/bush/toolwrapper/VersionControlSystem.h"

class Git : public VersionControlSystem
{
public:
  virtual std::string getName() const;
  virtual bool publish(const std::vector<std::string>& files, const std::string& message);
};
