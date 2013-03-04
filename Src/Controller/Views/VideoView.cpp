/**
* @file Controller/Views/VideoView.cpp
*
* Implementation of class VideoView
*
* @author <a href="mailto:matroc@tzi.de">Max Trocha</a>
*/

#include <QWidget>
#include <QPainter>
#include <QApplication>
#include <QMouseEvent>
#include <QResizeEvent>
#include <QSettings>

#include <phonon/VideoWidget>
#include <phonon/AudioOutput>
#include <phonon/MediaObject>

#include "Controller/RobotConsole.h"
#include "Controller/RoboCupCtrl.h"
#include "Controller/Visualization/PaintMethods.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Platform/Thread.h"
#include "VideoView.h"
#include "Controller/VideoCtrl.h"

class VideoWidget : public Phonon::VideoWidget, public SimRobot::Widget
{
public:
  VideoWidget(VideoView& videoView) :
    videoView(videoView)
  {
    Phonon::AudioOutput* audioOutput = new Phonon::AudioOutput(Phonon::VideoCategory, this);
    Phonon::Path path = Phonon::createPath(&videoView.console, audioOutput);
    Phonon::createPath(&videoView.console, this);
  }

  virtual ~VideoWidget() {}

private:
  VideoView& videoView;

  virtual QWidget* getWidget() {return this;}

  QSize sizeHint() const { return QSize(); }

  virtual operator QWidget* () {return this;}

  friend class VideoView;

};

VideoView::VideoView(const QString& fullName, VideoCtrl& console) :
  fullName(fullName), icon(":/Icons/tag_green.png"), console(console) {}

SimRobot::Widget* VideoView::createWidget()
{
  return new VideoWidget(*this);
}
