/**
* @file  Platform/SystemCall.h
* @brief static class for system calls from the non NDA classes
*
* Inclusion of platform dependend definitions of system calls.
*
* @author <a href="mailto:martin@martin-loetzsch.de">Martin Lötzsch</a>
*/

#pragma once
#ifdef TARGET_ROBOT

#ifdef LINUX
#include "Linux/SystemCall.h"
#define SYSTEMCALL_INCLUDED
#endif

#endif


#ifdef TARGET_SIM

#include "SimRobotQt/SystemCall.h"
#define SYSTEMCALL_INCLUDED
#else

#endif


#ifdef TARGET_TOOL

#ifdef LINUX
#include "Linux/SystemCall.h"
#else
#include "SimRobotQt/SystemCall.h"
#endif  // LINUX
#define SYSTEMCALL_INCLUDED

#endif // TARGET_TOOL


#ifndef SYSTEMCALL_INCLUDED
#error "Unknown platform or target"
#endif
