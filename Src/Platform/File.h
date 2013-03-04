/**
* @file Platform/File.h
*
* Inclusion of platform dependend definitions of simple file handling.
*
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#pragma once
#ifdef WIN32
#include "Win32Linux/File.h"
#define FILE_INCLUDED
#endif

#ifdef LINUX
#include "Win32Linux/File.h"
#define FILE_INCLUDED
#endif

#ifdef MACOSX
#include "Win32Linux/File.h"
#define FILE_INCLUDED
#endif

#ifndef FILE_INCLUDED
#error "Unknown platform or target"
#endif
