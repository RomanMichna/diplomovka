/**
* @file Controller/VideoCtrl.cpp
*
* Implementation of VideoCtrl.
*
* @author <a href="mailto:matroc@tzi.de">Max Trocha</a>
*/

#include "VideoCtrl.h"



VideoCtrl::VideoCtrl(const char* name, const char* fileName)
{
  setCurrentSource(QString(fileName));
  ctrl = (ConsoleRoboCupCtrl*) RoboCupCtrl::controller;
  this->name = QString(name);
  QObject::connect((MediaObject*)this, SIGNAL(tick(qint64)),
                   (TimeControlled*)this, SLOT(doTick(void)));
  setTickInterval(10);
}

void VideoCtrl::setTimeCtrl(TimeCtrl* tc)
{
  timeCtrl = tc;
}

VideoCtrl::~VideoCtrl() {}

void VideoCtrl::addViews()
{
  SimRobot::Object* category = ctrl->addCategory(name);

  ctrl->addView(new VideoView(name + ".video", *this), category);
}

void VideoCtrl::setTime(int time)
{
  seek(time);
  //HACK: since play() on MediaObjects is asynchronous
  // we just start the video and let the log sync to it
  // this may cause problems with multiple videos though
  setPause(false);
}

int VideoCtrl::getTime()
{
  return (int)currentTime();
}

int VideoCtrl::getLength()
{
  return (int)totalTime();
}

void VideoCtrl::setPause(bool pause)
{
  if(pause)
    this->pause();
  else
    this->play();
}

bool VideoCtrl::ready()
{
  switch(state())
  {
  case Phonon::PlayingState:
    return true;
  default:
    return false;
  }
}