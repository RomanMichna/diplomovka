/**
* @file Controller/TimeCtrl.cpp
*
* Implementation of TimeCtrl.
*
* @author <a href="mailto:matroc@tzi.de">Max Trocha</a>
*/

#include "TimeCtrl.h"
#include "ConsoleRoboCupCtrl.h"



TimeCtrl::TimeCtrl(const char* name, std::list<TimeControlled*>& controlled)
  : controlled(controlled),
    playing(true),
    seeking(false)
{
  lastTick = SystemCall::getCurrentSystemTime();
  this->name = QString(name);
  pos = 0;
  registerTimeCtrl();
}

void TimeCtrl::registerTimeCtrl()
{
  for(std::list<TimeControlled*>::iterator i = controlled.begin(); i != controlled.end(); ++i)
  {
    (*i)->setTimeCtrl(this);
  }
}

void TimeCtrl::seek(int time)
{
  pos = time;
  for(std::list<TimeControlled*>::iterator i = controlled.begin(); i != controlled.end(); ++i)
  {
    int len = (*i)->getLength();
    int offset = (*i)->offset;
    if(offset > time || len + offset < time)
    {
      (*i)->setPause(true);
      (*i)->setTime(0);
    }
    else
    {
      (*i)->setTime(time - offset);
    }
  }
}

void TimeCtrl::tick(TimeControlled& self)
{
  int tDiff = self.getTime() + self.offset - pos;
  if(abs(tDiff - timeSinceLastTick()) < self.getResyncThreshold() && !seeking)
  {
    if(tDiff >= 0)
    {
      pos = self.getTime() + self.offset;
      lastTick = SystemCall::getCurrentSystemTime();
    }
    setPlay(playing);
  }
  else
    self.setTime(max(pos - self.offset, 0));
  emit tick();
}

TimeCtrl::~TimeCtrl()
{
  for(std::list<TimeControlled*>::iterator i = controlled.begin(); i != controlled.end(); ++i)
  {
    (*i)->setTimeCtrl(0);
  }
}

void TimeCtrl::addViews()
{
  ConsoleRoboCupCtrl* ctrl = (ConsoleRoboCupCtrl*)RoboCupCtrl::controller;
  SimRobot::Object* category = ctrl->addCategory(name);

  ctrl->addView(new TimeCtrlView(name + ".timeCtrl", *this), category);
}

QString TimeCtrl::msecToQString(int msec)
{
  int sec = msec / 1000;
  QString res;
  QTextStream(&res) << sec / 60 << ':' << ((sec % 60 < 10) ? "0" : "") << sec % 60;
  return res;
}

int TimeCtrl::getLength()
{
  int maxLen = 0;
  for(std::list<TimeControlled*>::iterator i = controlled.begin(); i != controlled.end(); ++i)
  {
    maxLen = std::max(maxLen, (*i)->getLength() + (*i)->offset);
  }
  return maxLen;
}

QString TimeCtrl::getLengthStr()
{
  return msecToQString(getLength());
}

int TimeCtrl::getPos()
{
  return pos;
}

QString TimeCtrl::getPosStr()
{
  return msecToQString(getPos());
}

void TimeCtrl::startSeeking()
{
  seeking = true;
}

void TimeCtrl::stopSeeking()
{
  seeking = false;
}

void TimeCtrl::setPause(bool pause)
{
  if(!playing && !pause)
    lastTick = SystemCall::getCurrentSystemTime();
  playing = !pause;
  for(std::list<TimeControlled*>::iterator i = controlled.begin(); i != controlled.end(); ++i)
  {
    if((*i)->offset > pos || (*i)->getLength() + (*i)->offset < pos)
    {
      (*i)->setPause(true);
    }
    else
    {
      (*i)->setPause(pause);
    }
  }
}

void TimeCtrl::setPlay(bool play)
{
  setPause(!play);
}

bool TimeCtrl::allReady()
{
  for(std::list<TimeControlled*>::iterator i = controlled.begin(); i != controlled.end(); ++i)
  {
    if(!(*i)->ready())
      return false;
  }
  return true;
}

bool TimeCtrl::isPlaying()
{
  return playing;
}

int TimeCtrl::timeSinceLastTick()
{
  return (int)(SystemCall::getCurrentSystemTime() - lastTick);
}
