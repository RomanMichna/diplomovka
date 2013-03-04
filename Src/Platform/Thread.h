/**
* @file Platform/Thread.h
*
* Inclusion of platform dependend definitions for thread usage.
*
* @author <a href="mailto:ingsie@informatik.uni-bremen.de">Ingo Sieverdingbeck</a>
*/

#pragma once
#ifdef TARGET_ROBOT

#ifdef LINUX
#include "Linux/Thread.h"
#define THREAD_INCLUDED
#endif

#endif


#if defined(TARGET_SIM) || defined(TARGET_TOOL)

#ifdef WIN32
#include "Win32/Thread.h"
#define THREAD_INCLUDED
#endif

#ifdef LINUX
#include "Linux/Thread.h"
#define THREAD_INCLUDED
#endif

#ifdef MACOSX
#include "linux/Thread.h"
#define THREAD_INCLUDED
#endif

#endif


#ifndef THREAD_INCLUDED
#error Unknown platform
#endif
