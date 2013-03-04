/**
* @file Tools/ProcessFramework/ProcessFramework.cpp
*
* This file implements classes corresponding to the process framework.
*
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#include "ProcessFramework.h"

void PlatformProcess::setPriority(int priority)
{
  this->priority = priority;
  if(processBase)
    processBase->setPriority(priority);
}

/*
 * ProcessCreatorBase::list must be initialized before any code generated
 * by MAKE_PROCESS is executed. Therefore, early initialization is forced.
 */
#ifdef WIN32
#pragma warning(disable:4073)
#pragma init_seg(lib)
std::list<ProcessCreatorBase*> ProcessCreatorBase::list;
#else
std::list<ProcessCreatorBase*> ProcessCreatorBase::list
                               __attribute__((init_priority(60000)));
// Note: the default priority for initialization of static objects
// using 'gcc' is 65535 (lowest), the highest is 101.
#endif
