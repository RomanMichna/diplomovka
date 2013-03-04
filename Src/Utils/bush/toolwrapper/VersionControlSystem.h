#pragma once

#include <string>
#include <vector>

class VersionControlSystem
{
public:
  virtual std::string getName() const = 0;
  virtual bool publish(const std::vector<std::string>& files, const std::string& message) = 0;
};
