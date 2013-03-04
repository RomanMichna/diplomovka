/**
* @file Controller/LogPlayer.h
*
* Definition of class LogPlayer
*
* @author Martin L�tzsch
*/

#pragma once

#include "Tools/MessageQueue/MessageQueue.h"
#include "TimeCtrl.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Perception/JPEGImage.h"

/**
* @class LogPlayer
*
* A message queue that can record and play logfiles.
* The messages are played in the same time sequence as they were recorded.
*
* @author Martin L�tzsch
*/
class LogPlayer : public MessageQueue, public TimeControlled
{
public:
  /**
  * Constructor
  * @param targetQueue The queue into that messages from played logfiles shall be stored.
  */
  LogPlayer(MessageQueue& targetQueue);

  /** Deletes all messages from the queue */
  void init();

  /**
  * Opens a log file and reads all messages into the queue.
  * @param fileName the name of the file to open
  * @return if the reading was successful
  */
  bool open(const char* fileName);

  /**
  * Playes the queue.
  * Note that you have to call replay() regularely if you want to use that function
  */
  void play();

  /** Pauses playing the queue. */
  void pause();

  /** Stops playing the queue, resets the position in the queue to the first message */
  void stop();

  /** Stops recording, resets the position to the first message */
  void recordStop();

  /** Starts recording.
   * Note that you have to notify the queue on new messages with handleMessage().
   */
  void recordStart();

  /** Plays the next message in the queue */
  void stepForward();

  /** Plays messages in the queue until a frame that contains an image got copied. */
  void stepImageForward();

  /** Plays the previous message in the queue */
  void stepBackward();

  /** Plays previous messages in the queue until a frame that contains an image got copied. */
  void stepImageBackward();

  /** repeats the current message in the queue */
  void stepRepeat();

  /** jumps to given message-number in the queue */
  void gotoFrame(int frame);

  /** Set loop mode. If disabled the logfile is played only once. */
  void setLoop(bool);

  /**
  * Writes all messages in the log player queue to a log file.
  * @param fileName the name of the file to write
  * @return if the writing was successful
  */
  bool save(const char* fileName);

  /**
  * Writes all images in the log player queue to a bunch of files (*.bmp or *.jpg).
  * @param raw Savecolor unconverted
  * @param fileName the name of one file to write, all files will be enumerated by appending a 3 digit number to the filename.
  * @return if the writing of all files was successful
  */
  bool saveImages(const bool raw, const char* fileName);

  /**
  * Save an image to a file.
  * @param image The image to save.
  * @param fileName The intended file name of the image.
  * @param imageNumber A number that will be integrated into the file name. -1: ignore.
  * @param YUV2RGB Convert from YUV to RGB?
  * @return Was writing successful?
  */
  static bool saveImage(const Image& image, const char* fileName, int imageNumber = -1, bool YUV2RGB = true);
    
  /**
  * Adds the message to the queue depending on isRecording.
  * That function should be called for every message in the queue that the
  * log player shall work on.
  */
  void handleMessage(InMessage& message);

  /**
  * If playing a log file, that function checks if it is time to release the next
  * message dependend on the time stamp. Call that function whenever there is some
  * processing time left.
  * @return Was log data replayed?
  */
  bool replay(bool realtime);

  /**
  * The functions filters the message queue.
  * @param messageIDs An null-terminated array of message ids that should be kept.
  */
  void keep(MessageID* messageIDs);

  /**
  * The functions filters the message queue.
  * @param messageIDs An null-terminated array of message ids that should be removed.
  */
  void remove(MessageID* messageIDs);

  /**
  * The function creates a histogram on the message ids contained in the log file.
  * @param frequency An array that is filled with the frequency of message ids.
  */
  void statistics(int frequency[numOfDataMessageIDs]);

  /** different states of the logplayer */
  ENUM(LogPlayerState, initial, recording, paused, playing);

  LogPlayerState state; /**< The state of the log player. */
  int currentFrameNumber; /**< The number of the current frame. */
  int numberOfFrames; /**< The overall number of frames available. */

private:
  MessageQueue& targetQueue; /**< The queue into that messages from played logfiles shall be stored. */
  int currentMessageNumber; /**< The current message number in the message queue. */
  int numberOfMessagesWithinCompleteFrames; /**< The number of messages within complete frames. Messages behind that number will be skipped. */
  bool loop;
  int lastImageFrameNumber; /**< The number of the last frame that contained an image. */
  unsigned logBeginTimestamp;
  unsigned logEndTimestamp;
  unsigned currentLogTimestamp;
  unsigned lastRealTimestamp;
  int replayOffset;
  bool fallbackTimings;

  /**
  * The method counts the number of frames.
  */
  void countFrames();

  unsigned getTimeInformation();
  
  /**
  * The method expands image file name to its full path and integrates
  * an image number if desired.
  * @param fileName The short file name of the image.
  * @param imageNumber A number that will be integrated into the file name. -1: ignore.
  * @return The full path of the image file.
  */
  static std::string expandImageFileName(const char* fileName, int imageNumber);

public:
  virtual void setTime(int time);
  virtual int getTime();
  virtual int getLength();
  virtual void setPause(bool pause);
  virtual int getResyncThreshold() {return 200;}
};
