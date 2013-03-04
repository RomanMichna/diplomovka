/*
 * Logger.cpp
 *
 *  Created on: Jan 27, 2012
 *      Author: arne
 */

#include "Logger.h"
#include "Tools/Streams/OutStreams.h"
#include "Platform/Win32Linux/File.h"
#include "Tools/Configuration/ConfigMap.h"
#include <algorithm>
#include "Tools/Module/ModuleManager.h"
#include "Platform/SoundPlayer.h"
#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;

Logger::Logger(const string& filenameExtension) :
		      theConfig(string(File::getBHDir()) + "/Config/logger.cfg"),
		      theState(Init),
		      theFilenameExtension(filenameExtension),
		      theFrameCounter(0),
		      theFileHasBeenCreated(false),
		      maximumBlockSizeStatistics(0),
		      maximumBlockSizeStatisticsLastFrame(0)
{

  #ifdef TARGET_SIM
  theConfig.setEnabled(false);
  #endif

  initMessageIds();
  loadRepresentations();
}

Logger::~Logger()
{
  if(NULL != thepOutFile)
  {

    //store everything before destruction
    while(!isBufferEmpty())
    {
      save();
    }

    for(unsigned int i = 0; i < theConfig.getMaxBufferSize(); i++)
    {
      delete theBuffer[i];
    }

    delete thepOutFile;
  }
}

void
Logger::save()
{

  if(!theFileHasBeenCreated)
  {
    createFileOnDisk();
  }


  unsigned int blockSize = theConfig.getWriteBlockSize();

  while(!isBufferEmpty() &&
        blockSize > 0)
  {
    if(theBuffer[theOldestUsedSlot]->getNumberOfMessages() > 0)
    {

      theBuffer[theOldestUsedSlot]->append(*thepOutFile);//write the frame to disk.
      theBuffer[theOldestUsedSlot]->clear();

      theOldestUsedSlot = (theOldestUsedSlot+1)%theConfig.getMaxBufferSize();
      blockSize--;
    }
  }

  EXECUTE_ONLY_IN_DEBUG(
    if(isBufferEmpty())
    {
      SoundPlayer::play("log_written.wav");
    }
  );



}

void
Logger::logFrame(ProcessName name)
{
  //Do nothing if the logger is disabled
  if(!theConfig.getEnabled())
  {
    return;
  }

  /**
   * Logfile format:
   * | File size (4 byte) | Num of Messages (4 byte) | Frame 1 | Frame 2 | ... | Frame n|
   *
   * file size and number of messages should be set to -1 meaning that the file size is unknown and has to be determined while reading
   *
   * Frame format:
   * | ProcessBegin | Log data 1 | Log data 2 | ... | Log data n | ProcessFinished |
   *
   * Log data format:
   * | ID ( 1 byte) | Message size (3 byte) | Message |
   */

  //write processBegin for the specified process
  theBuffer[theCurrentSlot]->out.bin << name;
  theBuffer[theCurrentSlot]->out.finishMessage(idProcessBegin);


  //stream all provided representations to the queue
  list<pair<MessageID,const Streamable*> >::iterator it;
  for(it = provideRepresentations().begin(); it != provideRepresentations().end(); it++)
  {
    /*
     * it->first  = MessageId of the representation
     * it->second = The representation
     */
    //Some representations do not have message ids. Those cannot be saved.
    if(undefined == it->first)
    {
      continue;
    }

    //Message ids above numOfDataMessageIds do no belong to representations.
    ASSERT(it->first < numOfDataMessageIDs);
    if(numOfDataMessageIDs <= it->first )
    {
      continue;
    }


    unsigned int size = getStreamableSize(*(it->second));

    maximumBlockSizeStatistics += size;

    //some streamables do not stream anything.
    //If no data is streamed the queue crashes.
    if (size > 0)
    {
      //stream the message to the buffer
      theBuffer[theCurrentSlot]->out.bin << *(it->second);

      //the buffer has a limited size. Thus it might be full.
      if(theBuffer[theCurrentSlot]->writeErrorOccurred())
      {
        OUTPUT_WARNING("Logging of " << ::getName(it->first) << " has failed. The buffer is full.");
      }

      theBuffer[theCurrentSlot]->out.finishMessage(it->first);
    }
  }

  //write processFinished for the specified process
  theBuffer[theCurrentSlot]->out.bin << name;
  theBuffer[theCurrentSlot]->out.finishMessage(idProcessFinished);

  //cognition runs at 60 fps. Therefore use a new block every 60 frames.
  //Thus one block is used per second.
  if(theFrameCounter >= 60)
  {
    if(theConfig.getOutputStatistics())
    {
      if(maximumBlockSizeStatistics > maximumBlockSizeStatisticsLastFrame)
      {
        maximumBlockSizeStatisticsLastFrame = maximumBlockSizeStatistics;
        OUTPUT(idText,text,"Logger: maximum block size: " << maximumBlockSizeStatistics);
      }

    }

    maximumBlockSizeStatistics = 0; //reset block size statistics
    //the next call to logFrame will use a new block.
    theFrameCounter = 0;
    ensureNextSlotFree();
    theCurrentSlot = (theCurrentSlot+1) % theConfig.getMaxBufferSize();

  }

  theFrameCounter++;

}

void Logger::loadRepresentations()
{
  vector<RepresentationName> reps(theConfig.getCognitionRepresentations());
  //for each configured representation: add an entry to m_activeRepresentations
  vector<RepresentationName>::iterator rep;
  for(rep = reps.begin(); rep != reps.end(); rep++)
  {
    //TODO remove hard coded c
    m_activeRepresentations['c'].push_back(*rep);
  }
}

map<Logger::ProcessName, list<Logger::RepresentationName> > const&
Logger::getActiveRepresentations() const
{
  return m_activeRepresentations;
}


void Logger::update(unsigned char gameInfoState, unsigned short penaltyState)
{

  MODIFY("module:Logger:config",theConfig);

  theGameInfostate = gameInfoState;
  thePenaltyState = penaltyState;
  //a very simple state machine
  DEBUG_RESPONSE("module:Logger:currentState", OUTPUT(idText, text,"state: " << getName(theState)););

  switch(theState)
  {
    case Init:
      enterInit();
      break;

    case Setup:
      enterSetup();
      break;

    case Idle:
      enterIdle();

      break;

    case Recording:
      enterRecording();
      break;

   default:
     //If the default  case has been reached there is an
     //implementation bug...
     ASSERT(true);
   }
}

/**
 * OnEnter function for state Init();
 * Checks whether the Logger is enabled or not.
 * Changes state to Setup if the logger is enabled. Remains in Init otherwise.
 */
void Logger::enterInit()
{
  if(theConfig.getEnabled())
  {
    changeState(Setup);
  }
}

/**
 * OnEnter function for state Setup.
 * Initializes the ringbuffer.
 */
void Logger::enterSetup()
{
  //initialize the buffer
  //FIXME calculate frameSize dynamically
  for(unsigned int i= 0; i < theConfig.getMaxBufferSize(); i++)
  {
    theBuffer.push_back(new MessageQueue());
    theBuffer.back()->setSize(theConfig.getFrameSize());
  }
  theCurrentSlot = 0;
  theOldestUsedSlot = 0;

  changeState(Idle);

}

/**
 * OnEnter function for state Idle.
 * Checks if there is anything to write in the buffer.
 * If yes writes it to the file.
 * Chages the state to Recording upon game.state change.
 */
void Logger::enterIdle()
{
  if(!isBufferEmpty())
  {
    save();
  }


  //only consider leaving this state if we are not penalized and not in init/finished
  if( thePenaltyState  == PENALTY_NONE   &&
      theGameInfostate != STATE_FINISHED &&
      theGameInfostate != STATE_INITIAL)
  {
    logFrame(getProcessName()); //log this frame, it is the first playing frame.
    changeState(Recording);
  }

}

void Logger::enterRecording()
{
  //switch back to Idle if the is finished/initial or the we have been penalized
  if(theGameInfostate == STATE_FINISHED ||
     thePenaltyState  != PENALTY_NONE ||
     theGameInfostate == STATE_INITIAL)
  {
    changeState(Idle);
  }
  else
  {
    logFrame(getProcessName());
  }

}

void
Logger::initMessageIds()
{
  for(int i = 0; i < numOfMessageIDs; ++i)
  {
    string name(::getName((MessageID)i));
    //TODO this is a veeeery dangerous quick hack. fix it!
    name = name.substr(2);
    //all names start with id followed by the representationName. The substr removes the id
    m_messageIDs[name] = (MessageID)i;
  }
}


MessageID
Logger::getMessageId(RepresentationName name) const
{

  map<RepresentationName, MessageID>::const_iterator it = m_messageIDs.find(name);
  if(it != m_messageIDs.end())
  {
    return it->second;
  }
  else
  {
    OUTPUT_WARNING("Logger: No MessageId exists for representation " << name << "! The representation will not be logged.");
    return undefined;
  }

}


inline void Logger::changeState(LoggerState newState)
{
  theState = newState;
}

unsigned int Logger::getStreamableSize(const Streamable& s)
{

  theSizeCounter << s;
  unsigned int size = theSizeCounter.getSize();
  theSizeCounter.reset();
  return size;
}


void Logger::ensureNextSlotFree()
{

  /**
   * This is a typical ringbuffer.
   * Note that there is one buffer element between
   * first and oldest that will never be filled.
   * Without this free element the calculation would be more
   * complicated.
   */

  if((theCurrentSlot+1)%theConfig.getMaxBufferSize() ==
      theOldestUsedSlot%theConfig.getMaxBufferSize())
  {//no free frame found
    //drop the oldest frame.
    theBuffer[theOldestUsedSlot]->clear();
    theOldestUsedSlot = (theOldestUsedSlot+1) % theConfig.getMaxBufferSize();
    OUTPUT_WARNING("Logger: Buffer full, discarding block!");
  }
}

bool Logger::fileExists(const std::string & path)
{
	std::ifstream f;
	f.open(path.c_str());
	if(f.is_open())
	{
		f.close();
		return true;
	}
	return false;
}

void Logger::createFileOnDisk()
{

  //abort if the file has already been created.
  if(theFileHasBeenCreated)
  {
    OUTPUT_WARNING("Logger: tried to create file twice. This should not happen.");
    return;
  }

  //check if the file exists.
  //If it does exist the name is changed to prevent overwriting.

  std::stringstream playerNo;
  playerNo << theConfig.getPlayerNumber();

  std::string filename(theConfig.getLogFilePath() + "_player" + playerNo.str());

  bool first = true;
  for(int i = 1; fileExists(filename + theFilenameExtension); i++)
  {
    OUTPUT_WARNING("Logfile '" << (filename + theFilenameExtension) << "' already exists. Searching for alternative name");
    std::stringstream converter;
    converter << i;

    if (first)
    {
      filename += "_1";
      first = false;
    }
    else
    {
      filename.replace(filename.end()-1,filename.end(),converter.str());
    }

  }
  //If we arrive at this line the we have found filename that does not exist, yet.

  thepOutFile = new OutBinaryFile(filename + theFilenameExtension);


  /**
    * The first 8 bytes of a logfile contain the size and the frame count of the log file.
    * Since we append log data to the file it is impossible to know the size in advance.
    * Therefore size and length are -1. If the simulator encounters -1 instead of size/length
    * it will calculate them automatically.
    */
   theBuffer.back()->writeAppendableHeader(*thepOutFile); //it doesn't matter which queue writes the appendable header
   theFileHasBeenCreated = true;

}

bool Logger::isBufferEmpty()
{
  return theCurrentSlot == theOldestUsedSlot;
}






















