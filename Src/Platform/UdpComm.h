/**
* @file Platform/UdpComm.h
*
* Inclusion of platform dependent definitions of simple UDP communication handling.
*
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#ifdef WIN32
#include "Win32Linux/UdpComm.h"
#define UDPCOMM_INCLUDED
#endif

#ifdef LINUX
#include "Win32Linux/UdpComm.h"
#define UDPCOMM_INCLUDED
#endif

#ifdef MACOSX
#include "Win32Linux/UdpComm.h"
#define UDPCOMM_INCLUDED
#endif

#ifdef CYGWIN
#include "Win32Linux/UdpComm.h"
#define UDPCOMM_INCLUDED
#endif

#ifndef UDPCOMM_INCLUDED
#error Unknown platform
#endif
