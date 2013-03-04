/**
* @file Platform/TcpComm.h
*
* Inclusion of platform dependent definitions of simple TCP/IP communication handling.
*
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#ifdef WIN32
#include "Win32Linux/TcpComm.h"
#define TCPCOMM_INCLUDED
#endif

#ifdef LINUX
#include "Win32Linux/TcpComm.h"
#define TCPCOMM_INCLUDED
#endif

#ifdef MACOSX
#include "Win32Linux/TcpComm.h"
#define TCPCOMM_INCLUDED
#endif

#ifdef CYGWIN
#include "Win32Linux/TcpComm.h"
#define TCPCOMM_INCLUDED
#endif

#ifndef TCPCOMM_INCLUDED
#error Unknown platform
#endif
