/**
* @file Platform/BHAssert.h
*
* Inclusion of platform dependend definitions for low level debugging.
*
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#pragma once
#ifdef TARGET_ROBOT

#ifdef LINUX
#include "Linux/BHAssert.h"
#define BHASSERT_INCLUDED
#endif

#endif


#ifdef TARGET_SIM

#ifdef WIN32
#include "Win32/BHAssert.h"
#define BHASSERT_INCLUDED
#endif

#ifdef LINUX
#include "Linux/BHAssert.h"
#define BHASSERT_INCLUDED
#endif

#ifdef MACOSX
#include "linux/BHAssert.h"
#define BHASSERT_INCLUDED
#endif

#endif


#ifdef TARGET_TOOL

#ifdef WIN32
#include "Win32/BHAssert.h"
#define BHASSERT_INCLUDED
#endif

#ifdef LINUX
#include "Linux/BHAssert.h"
#define BHASSERT_INCLUDED
#endif

#ifdef MACOSX
#include "linux/BHAssert.h"
#define BHASSERT_INCLUDED
#endif

#endif


#ifndef BHASSERT_INCLUDED
#error "Unknown platform or target"
#endif
