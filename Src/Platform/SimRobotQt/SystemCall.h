/**
* @file  Platform/SimRobotQt/SystemCall.h
* @brief static class for system calls from the non NDA classes
* @attention the implementation is system specific! Only definitions here
*/

#pragma once

/**
 * All process-local global variable declarations have to be preceeded
 * by this macro. Only variables of simple types can be defined, i.e.
 * no class instantiations are allowed.
 */
#if defined(WIN32) || defined(MACOSX)
#define PROCESS_WIDE_STORAGE(type) TLS<type>
#define PROCESS_WIDE_STORAGE_STATIC(type) static PROCESS_WIDE_STORAGE(type)

/**
* @class TLS
* The class encapsulates thread local storage.
*/
class TLSHandler
{
private:
  unsigned id;

public:
  TLSHandler();
  ~TLSHandler();
  void* get() const;
  void set(void* value);
};

/**
* @class TLS
* The class encapsulates typed thread local storage.
* It is meant as an replacement for __declspec(thread) that does not
* work in DLLs on Windows XP.
*/
template<class T> class TLS
{
private:
  TLSHandler handler;

public:
  TLS() {handler.set((void*) 0);}
  TLS(T* value) {handler.set((void*) value);}
  T* operator=(T* value) {handler.set((void*) value); return value;}
  T& operator*() const {return *(T*) handler.get();}
  T* operator->() const {return (T*) handler.get();}
  operator T* () const {return (T*) handler.get();}
};
#else
#define PROCESS_WIDE_STORAGE(type) __thread type*
#define PROCESS_WIDE_STORAGE_STATIC(type) static PROCESS_WIDE_STORAGE(type)
#endif

/**
* static class for system calls
* @attention the implementation is system specific!
*/
class SystemCall
{
public:
  enum Mode
  {
    physicalRobot,
    remoteRobot,
    simulatedRobot,
    logfileReplay,
    teamRobot,
  };

  /** returns the current system time in milliseconds*/
  static unsigned getCurrentSystemTime();

  /** returns the real system time in milliseconds (never the simulated one)*/
  static unsigned getRealSystemTime();

  /**
  * The function returns the thread cpu time of the calling thread in microseconds.
  * return thread cpu time of the calling thread
  */
  static unsigned long long getCurrentThreadTime();

  /** returns the time since aTime*/
  inline static int getTimeSince(unsigned aTime)
  {
    return (int)(getCurrentSystemTime() - aTime);
  }

  /** returns the real time since aTime*/
  inline static int getRealTimeSince(unsigned aTime)
  {
    return (int)(getRealSystemTime() - aTime);
  }

  /** returns the name of the local machine*/
  static const char* getHostName();

  /** returns the first ip address of the local machine*/
  static const char* getHostAddr();

  /** Sleeps for some milliseconds.
  * \param ms The amout of milliseconds.
  */
  static void sleep(unsigned int ms);

  /** returns the current execution mode */
  static Mode getMode();

  /** Returns the load and the physical memory usage in percent */
  static void getLoad(float& mem, float load[3]);
};
