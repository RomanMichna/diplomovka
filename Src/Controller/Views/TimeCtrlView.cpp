/**
* @file Controller/Views/TimeCtrlView.cpp
*
* Implementation of class TimeCtrlView
*
* @author <a href="mailto:matroc@tzi.de">Max Trocha</a>
*/

#include <QWidget>
#include <QSlider>
#include <QApplication>
#include <QLabel>
#include <QBoxLayout>
#include <QSettings>
#include <QPushButton>

#include "Controller/RobotConsole.h"
#include "Controller/RoboCupCtrl.h"
#include "Controller/Visualization/PaintMethods.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Platform/Thread.h"
#include "TimeCtrlView.h"
#include "Controller/TimeCtrl.h"

class TimeCtrlWidget : public QWidget, public SimRobot::Widget
{
public:
  TimeCtrlWidget(TimeCtrl& timeCtrl) :
    timeCtrl(timeCtrl)
  {
    playButton = new QPushButton(QIcon(":/Icons/control_play_blue.png"), "", this);
    playButton->setCheckable(true);
    playButton->setChecked(timeCtrl.isPlaying());
    positionSlider = new QSlider(Qt::Horizontal);
    positionSlider->setRange(0, timeCtrl.getLength());
    positionSlider->setValue(timeCtrl.getPos());
    label = new QLabel(timeCtrl.getPosStr() + "/" + timeCtrl.getLengthStr());

    QBoxLayout* layout = new QHBoxLayout;
    layout->setMargin(0);
    layout->addWidget(playButton);
    layout->addWidget(positionSlider);
    layout->addWidget(label);
    setLayout(layout);

    QObject::connect(playButton, SIGNAL(clicked(bool)),
                     &timeCtrl, SLOT(setPlay(bool)));

    QObject::connect(positionSlider, SIGNAL(sliderPressed(void)),
                     &timeCtrl, SLOT(startSeeking(void)));

    QObject::connect(positionSlider, SIGNAL(sliderMoved(int)),
                     &timeCtrl, SLOT(seek(int)));

    QObject::connect(positionSlider, SIGNAL(sliderReleased(void)),
                     &timeCtrl, SLOT(stopSeeking(void)));

    QObject::connect(&timeCtrl, SIGNAL(tick(void)),
                     this, SLOT(update(void)));
  }

  virtual ~TimeCtrlWidget() {}

public slots:
  void update(void)
  {
    if(!timeCtrl.seeking)
    {
      positionSlider->setRange(0, timeCtrl.getLength());
      positionSlider->setValue(timeCtrl.getPos());
      label->setText(timeCtrl.getPosStr() + "/" + timeCtrl.getLengthStr());
    }
  }

private:
  TimeCtrl& timeCtrl;
  QSlider* positionSlider;
  QLabel* label;
  QPushButton* playButton;

  virtual QWidget* getWidget() {return this;}

  QSize sizeHint() const { return QSize(); }

  virtual operator QWidget* () {return this;}

  friend class TimeCtrl;

};

TimeCtrlView::TimeCtrlView(const QString& fullName, TimeCtrl& timeCtrl) :
  fullName(fullName), icon(":/Icons/tag_green.png"), timeCtrl(timeCtrl) {}

SimRobot::Widget* TimeCtrlView::createWidget()
{
  return new TimeCtrlWidget(timeCtrl);
}
