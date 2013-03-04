/**
* \file Platform/Camera.h
* Inclusion of platform dependend camera interface.
* \author Colin Graf
*/

#pragma once
#ifdef TARGET_ROBOT

#ifdef LINUX
#include "Linux/NaoCamera.h"
#define CAMERA_INCLUDED
#endif

#endif
