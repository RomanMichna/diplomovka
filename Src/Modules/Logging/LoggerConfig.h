/*
 * LoggerConfig.h
 *
 *  Created on: Jan 28, 2012
 *      Author: arne
 */

#include "Tools/Streams/Streamable.h"
#include "Tools/Streams/InStreams.h"
#include <vector>

#ifndef LOGGERCONFIG_H_
#define LOGGERCONFIG_H_

class LoggerConfig : public Streamable
{
public:

  /**
   *
   */
  LoggerConfig() : logFilePath(""), maxBufferSize(0), frameSize(0), enabled(false), writeBlockSize(0), outputStatistics(false),playerNumber(loadPlayerNo())
  {
  }

  /**
   * Initializes the config from the specified config file.
   *
   * @param configPath Path to the config file.
   */
  LoggerConfig(const std::string& configPath) : playerNumber(loadPlayerNo())
  {
    InConfigMap conf(configPath);
    ASSERT(conf.exists());

    if(conf.exists())
    {
      conf >> *this;
    }

    ASSERT(maxBufferSize > 0);
  }

  virtual  ~LoggerConfig(){};

  /**
   * Where to store the log file
   */
  inline std::string  getLogFilePath() const {return logFilePath;}

  /**
   * Maximum size of the buffer in bytes.
   */
  inline unsigned int  getMaxBufferSize() const {return maxBufferSize;}

  inline std::vector<std::string>  getCognitionRepresentations() const
  {
    return cognition_representations;
  }

  inline bool  getEnabled() const
  {
    return enabled;
  }

  inline unsigned int getFrameSize() const
  {return frameSize;}

  inline unsigned int getWriteBlockSize() const
  {
    return writeBlockSize;
  }

  inline void setEnabled(bool value)
  {
    enabled = value;
  }

  inline unsigned int getPlayerNumber()
  {
    return playerNumber;
  }

  inline bool getOutputStatistics()
  {
    return outputStatistics;
  }


private:
  unsigned int loadPlayerNo()
  {
    unsigned int id;
    ConfigMap cm;
    if(cm.read("settings.cfg", false, &ConfigMap::printOnErr) >= 0)
    {
      cm.setFlags(cm.getFlags() | ConfigMap::READONLY);
      cm["playerNumber"] >> id;
    }
    else
    {
      OUTPUT_WARNING("Logger: Unable to read settings.cfg");
    }

    return id;

  }


private:

  void
  serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(logFilePath);
    STREAM(maxBufferSize);
    STREAM(frameSize);
    STREAM(cognition_representations);
    STREAM(enabled);
    STREAM(writeBlockSize);
    STREAM(outputStatistics);
    STREAM_REGISTER_FINISH;
  }

  /**
   * Where to write the log file.
   */
  std::string logFilePath;
  /**
   * Max size of the buffer in bytes.
   */
  unsigned int maxBufferSize;

  /**
   * Size per frame. (in bytes)
   */
  unsigned int frameSize;

  /**
   * Contains the representations that should be logged.
   */
  std::vector<std::string> cognition_representations;

  /**
   * Determines whether the logger is enabled or disabled.
   */
  bool enabled;

  //The Logger writes to disk if the game state is finished or the robot has been penalized.
  //This value defines how many frames will be written to disc per cognition frame.
  //If you set the value too high the robot may hang while writing to disk and might react to game state changes.
  //If you set the value too low it will take a long time to write the whole log file.
  unsigned int writeBlockSize;

  /**
   * Output statistics if true.
   */
  bool outputStatistics;

  unsigned int playerNumber;

};

#endif /* LOGGERCONFIG_H_ */
