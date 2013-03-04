#pragma once

#include <vector>
#include <string>

class Filesystem
{
public:
  static std::vector<std::string> getWlanConfigs(const std::string prefix = "");
  static std::vector<std::string> getLocations(const std::string prefix = "");
  /** Returns the color tables in location without suffix .c64. */
  static std::vector<std::string> getColtables(const std::string& location,
      const std::string prefix = "");
  static std::vector<std::string> getProjects(const std::string prefix = "");
  static std::vector<std::string> getEntries(const std::string& directory,
      bool files,
      bool directories,
      const std::string& suffix = "",
      bool keepSuffixes = true);
  static std::string getFileAsString(const std::string& filename);
};
