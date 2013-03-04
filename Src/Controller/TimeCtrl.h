/*
* @file Controller/TimeCtrl.h
*
* Declaration of TimeCtrl.
*
* @author <a href="mailto:matroc@tzi.de">Max Trocha</a>
*/

#pragma once

#include "Views/TimeCtrlView.h"

#include <QString>
#include <phonon/MediaObject>

#include "Tools/ProcessFramework/Process.h"

class ConsoleRoboCupCtrl;
class TimeControlled;



/**
* @class TimeCtrl
*/
class TimeCtrl : public QObject
{
  Q_OBJECT

protected:
  std::list<TimeControlled*>& controlled; /**< Console lines buffered because the process is currently waiting. */
  QString name;
  int pos;
  bool playing;
  bool wasPlaying;
  QString msecToQString(int msec);
  void registerTimeCtrl();

public:

  /**
  * Constructor.
  */
  TimeCtrl(const char* name, std::list<TimeControlled*>& controlled);

  /**
  * Destructor.
  */
  ~TimeCtrl();

  /**
  * The function adds all views.
  */
  void addViews();

  void setPause(bool pause);

  int getLength();
  QString getLengthStr();

  int getPos();
  QString getPosStr();
  bool seeking;
  bool seekingFinished;
  bool allReady();

  bool isPlaying();

public slots:
  void setPlay(bool play);
  void startSeeking();
  void seek(int time);
  void stopSeeking();
  void tick(TimeControlled& self);

signals:
  void tick(void);

private:
  int timeSinceLastTick();
  unsigned long long lastTick;
};

class TimeControlled : public QObject
{
  Q_OBJECT
public:
  TimeControlled(): offset(0), timeCtrl(0) {}
  ~TimeControlled() {}

  virtual void setTime(int time) = 0;
  virtual int getTime() = 0;
  virtual int getLength() = 0;
  virtual void setPause(bool pause) = 0;
  virtual int getResyncThreshold() = 0;

  virtual bool ready() {return true;}

  virtual void setTimeCtrl(TimeCtrl* tc) {timeCtrl = tc;}

  int offset;

protected:
  TimeCtrl* timeCtrl;

public slots:
  void doTick() {if(timeCtrl) timeCtrl->tick(*this);}
};
