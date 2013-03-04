/*
 * Logger.h
 *
 *  Created on: Jan 27, 2012
 *      Author: arne
 */

#ifndef LOGGER_H_logger
#define LOGGER_H_

#include "Tools/Module/Module.h"

#include "LoggerConfig.h"
#include <map>
#include <string>
#include "Representations/Infrastructure/RoboCupGameControlData.h"
#include "Tools/MessageQueue/MessageQueue.h"
#include "Tools/Enum.h"
#include "Tools/Debugging/Modify.h"
using namespace std;


/**
 * Provides basic logger functionality.
 */
class Logger
{
  public:

    /**
     * This code uses severval maps to  map between representations, providers and processes.
     * These typedefs increase the readability of the mappings.
     */
    typedef string RepresentationName;
    typedef string ProviderName;
    typedef char ProcessName;

  private:
    ENUM(LoggerState,
        Init, //This is the initial state.
        Setup, //If the logger enters Setup it will try to create a logfile etc.
        Idle,  //Logger is running but not recording.
        Recording //Logger is recording
        );

    /**
     * Guess what... its the configuration.
     */
    LoggerConfig theConfig;


    /**
     * The logger is a tiny state machine.
     * This enum keeps track of its internal state
     */
    LoggerState theState;

    /**
     *
     */
    std::string theFilenameExtension;

    /**
     * Path to the config file.
     */
    const string m_configPath;

    /**
     * The log file :)
     * This is a pointer because otherwise I would have to initialize
     * the file in the initializer list. Upon initialization the file is
     * created on disc immediately. However the file should only be created
     * if the logger is enabled.
     */
    OutBinaryFile* thepOutFile;


    /**
     * Contains the names of all representations that should be logged ordered by
     * their corresponding process name.
     */
    map<ProcessName, list<RepresentationName> > m_activeRepresentations;

    /**
     * Contains a mapping from representation name to the corresponding message id.
     * The id is used to identify the representation during streaming.
     */
    map<RepresentationName, MessageID> m_messageIDs;


    /**
     * Buffer of MessageQueues. Eeach queue holds one frame.
     * This Buffer is used as RingBuffer.
     */
    vector<MessageQueue*> theBuffer;

    /**
     * Points to the currently used slot in theBuffer.
     */
    unsigned int theCurrentSlot;

    /**
     * Points to the oldest slot that is not free.
     */
    unsigned int theOldestUsedSlot;

    /**
     * Holds the current state of the game.
     */
    unsigned char theGameInfostate;

    /**
     * Holds the current penalty state.
     */
    unsigned short thePenaltyState;

    /**
     * Counts the current frame (from 0 to 30)
     */
    unsigned int theFrameCounter;


    /**
     * This stream is used by getStreamableSize to determine the size of
     * a Streamable.
     * Do not access it from anywhere except getStreamableSize()!!!
     */
    OutBinarySize theSizeCounter;

    /**
     * true if the log file has been created on disk.
     */
    bool theFileHasBeenCreated;

    /**
     * Is used to record statistics about the maximum block size.
     */
    unsigned int maximumBlockSizeStatistics;
    unsigned int maximumBlockSizeStatisticsLastFrame;


  public:

    /**
     * @param filenameExtension This string is appended to the log-filename.
     */
    Logger(const string& filenameExtension);
    virtual
    ~Logger();//TODO check if detor is correct.


    /**
     * returns the activeRepresentations ordered by process name.
     */
    map<ProcessName, list<RepresentationName> > const&
    getActiveRepresentations()const;

    /**
     * Returns the messageId for the specified representation name.
     * Returns unknown if the representation name was unknown.
     */
    MessageID
    getMessageId(RepresentationName name) const;

  protected:

    /**
     * Logs all representations provided by provideRepresentations().
     * It is the callers responsibility to ensure that the representations are valid
     * at the time this method is called.
     */
    void
    logFrame(ProcessName name);

    /**
     * Writes the buffer content to the files.
     */
    void
    save();

    /**
     * Creates the log file on disk.
     * If the file already exists another file name is generated.
     * If the file has already been created, the call will return without any effect.
     */
    void createFileOnDisk();

    /**
     * This is the main worker functiont.
     * It should be called once per frame by the base class.
     * Call this method once per frame.
     * @param gameInfoState The current state of the game. See "Representations/Infrastructure/RoboCupGameControlData.h"
     * @param penaltyState The current penalty state of this robot. See "Representations/Infrastructure/RoboCupGameControlData.h"
     */
    virtual void update(unsigned char gameInfoState, unsigned short penaltyState);

    /**
     * This method should provide the representations that should be logged.
     */
    virtual list<pair<MessageID,const Streamable*> >&
    provideRepresentations() = 0;

    /**
     * This method should provide the process name.
     * It is called every frame.
     */
    virtual ProcessName getProcessName() = 0;



  private:

    /**
     * Checks if the specified file exists.
     */
    bool fileExists(const std::string& path);


    /**
     * OnEnter function for state Init.
     * Checks whether the Logger is enabled or not.
     * Changes state to Setup if the logger is enabled. Remains in Init otherwise.
     */
    void enterInit();

    /**
     * OnEnter function for state Setup.
     * creates the log file and sets the state to idle.
     */
    void enterSetup();

    /**
     * OnEnter function for state Idle.
     * Checks if there is anything to write in the buffer.
     * If yes writes it to the file.
     * Chages the state to Recording upon game.state change.
     */
    void enterIdle();

    /**
     * OnEnter function for state Recording.
     * Records the current frame.
     * Changes to state Idle if upon game.state change.
     */
    void enterRecording();

    /**
     * Loads activated representations into m_activeRepresentations
     */
    void
    loadRepresentations();

    /**
     * Initializes the mapping from representation names to message ids.
     * This is done automatically by parsing the MessageID enum.
     */
    void
    initMessageIds();

    /**
     * changes the internal logger state;
     */
    inline void changeState(LoggerState newState);

    /**
     * Returns the size needed to stream the specified Streamable
     * @note This method is more expensive than the usual size() functions.
     *       Use with care :)
     */
    unsigned int getStreamableSize(const Streamable& queue);

    /**
     * Ensures that the slot at theFirstFreeSlot is really free.
     * Drops the last slot if the buffer is full to free up space.
     */
    void ensureNextSlotFree();

    /**
     * Returns true if the buffer is empty right now.
     */
    bool isBufferEmpty();



};

#endif /* LOGGER_H_ */
