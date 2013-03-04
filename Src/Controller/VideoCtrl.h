/*
* @file Controller/VideoCtrl.h
*
* Declaration of VideoCtrl.
*
* @author <a href="mailto:matroc@tzi.de">Max Trocha</a>
*/

#pragma once

#include "ConsoleRoboCupCtrl.h"
#include "Views/VideoView.h"

#include <QString>
#include <phonon/MediaObject>

#include "Tools/ProcessFramework/Process.h"

class ConsoleRoboCupCtrl;

/**
* @class VideoCtrl
*
* A process that simulates the Aperios VideoCtrl.
*/
class VideoCtrl : public Phonon::MediaObject, public TimeControlled
{
private:
  ConsoleRoboCupCtrl* ctrl; /** A pointer to the controller object. */
  std::list<std::string> lines; /**< Console lines buffered because the process is currently waiting. */
  QString name;

public:

  /**
  * Constructor.
  */
  VideoCtrl(const char* name, const char* fileName);

  /**
  * Destructor.
  */
  ~VideoCtrl();

  /**
  * The function adds all views.
  */
  void addViews();

  virtual void setTime(int time);
  virtual int getTime();
  virtual int getLength();
  virtual void setPause(bool pause);
  virtual int getResyncThreshold() {return 5000;}
  virtual bool ready();
  virtual void setTimeCtrl(TimeCtrl* tc);
};
