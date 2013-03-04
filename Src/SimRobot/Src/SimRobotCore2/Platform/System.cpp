/**
* @file System.cpp
* Implementation of class System
*/

#ifdef WIN32
#define NOMINMAX
#include <windows.h>
#elif defined(MACOSX)
#include <mach/mach_time.h>
#include <unistd.h>
#else
#include <time.h>
#include <unistd.h>
#endif

#include "System.h"

unsigned int System::getTime()
{
#ifdef WIN32
  return GetTickCount();
#elif defined(MACOSX)
  static mach_timebase_info_data_t info = {0, 0};
  if(info.denom == 0)
    mach_timebase_info(&info);
  return unsigned(mach_absolute_time() * (info.numer / info.denom) / 1000000);
#else
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (unsigned int) (ts.tv_sec * 1000 + ts.tv_nsec / 1000000l);
#endif
}
