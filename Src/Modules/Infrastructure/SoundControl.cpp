/**
* @file Modules/Infrastructure/SoundControl.cpp
* The file implements a module that plays back sounds.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#include "SoundControl.h"
#include "Platform/SoundPlayer.h"

void SoundControl::update(SoundOutput&)
{
  if(theSoundRequest.sound != SoundRequest::none && lastSound != theSoundRequest.sound)
  {
    std::string fileName = SoundRequest::getName(theSoundRequest.sound);
    fileName += ".wav";
#ifdef TARGET_ROBOT
    SoundPlayer::play(fileName);
#else
    OUTPUT(idText, text, "Playing sound: " << fileName);
#endif
  }
  lastSound = theSoundRequest.sound;
}

MAKE_MODULE(SoundControl, Infrastructure)
