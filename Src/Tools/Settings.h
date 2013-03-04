/**
* @file Tools/Settings.h
* Definition of a class that provides access to settings-specific configuration directories.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#pragma once

#include <string>
#include "Tools/Enum.h"

#ifdef TARGET_TOOL
class Framework;
#endif

/**
* @class Settings
* The class provides access to settings-specific configuration directories.
*/
class Settings
{
private:
  int teamNumber; /**< The number of our team in the game controller. Private, use theOwnTeamInfo.teamNumber instead. */
  unsigned int teamColor; /**< The color of our goal. Private, use theOwnTeamInfo.teamColour instead. */
  int playerNumber; /**< The number of the robot in the team. Private, use theRobotInfo.playerNumber instead. */
  std::string location; /**< The name of the location. */

public:
  ENUM(Model, nao);

  int teamPort; /**< The UDP port our team uses for team communication. */
  Model model; /**< The model of this robot. */
  std::string robot; /**< The name of this robot. */
  static bool recover; /**< Start directly without the pre-initial state. */

  /**
  * Default constructor. Creates a copy of the master settings instance.
  */
  Settings();

  /**
  * The function prefixes the given filename by a path based on the host's name.
  * @param file The name of a configuration file that is located in the Config/Hosts
  *             directory tree.
  * @return A path relative to the Config directory.
  */
  std::string expandHostFilename(const std::string& file) const;

  /**
  * The function prefixes the given filename by a path based on the selected location.
  * @param file The name of a configuration file that is located in the Config/Location
  *             directory tree.
  * @return A path relative to the Config directory.
  */
  std::string expandLocationFilename(const std::string& file) const;

  /**
  * The function prefixes the given filename by a path based on the robot's name.
  * @param file The name of a configuration file that is located in the Config/Robot
  *             directory tree.
  * @return A path relative to the Config directory.
  */
  std::string expandRobotFilename(const std::string& file) const;

  /**
  * The function prefixes the given filename by a path based on the host name and
  * the selected location.
  * @param file The name of a configuration file that is located in the Config/Location
  *             directory tree.
  * @return A path relative to the Config directory.
  */
  std::string expandHostLocationFilename(const std::string& file) const;

  /**
  * The function prefixes the given filename by a path based on the robot's name and
  * the selected location.
  * @param file The name of a configuration file that is located in the Config/Location
  *             directory tree.
  * @return A path relative to the Config directory.
  */
  std::string expandRobotLocationFilename(const std::string& file) const;

  /**
  * The function prefixes the given filename by a path based on the robot's model.
  * @param file The name of a configuration file that is located in the Config/Robots
  *             directory tree in one of the directories dedicated to robot models.
  * @return A path relative to the Config directory.
  */
  std::string expandModelFilename(const std::string& file) const;

  /**
  * Access to the location member
  * @return The name of the current location
  */
  std::string getLocation() const {return location;}

private:
  static Settings settings; /**< The master settings instance. */
  static bool loaded; /**< The load() of the master settings instance was called or not. */

  /**
  * Constructor for the master settings instance.
  */
  Settings(bool master);

  /**
  * The function loads the settings from disk.
  * @return Whether the settings were loaded successfully.
  */
  bool load();

  /**
  * Assignment operator
  * @param other The other settings that is assigned to this one
  * @return A reference to this object after the assignment.
  */
  Settings& operator=(const Settings& other)
  {
    teamNumber = other.teamNumber;
    teamColor = other.teamColor;
    playerNumber = other.playerNumber;
    location = other.location.c_str(); // avoid copy-on-write
    teamPort = other.teamPort;
    model = other.model;
    robot = other.robot.c_str(); // avoid copy-on-write
    return *this;
  }

  friend class Cognition; /**< To access playerNumber used for team communciation of simulated robots. */
  friend class TeamDataProvider; /**< To access teamNumber, teamColor and playerNumber. */
  friend class GameDataProvider; /**< To access teamNumber, teamColor and playerNumber. */
  friend class CognitionLogDataProvider; /**< To access playerNumber. */
  friend class Framework; /**< To access playerNumber. */
};
