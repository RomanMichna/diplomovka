/**
* @file Controller/Views/TimeView.cpp
*
* Implementation of class TimeView
*
* @author Colin Graf
*/

#include <QHeaderView>
#include <QApplication>
#include <QPainter>
#include <QFontMetrics>
#include <QSettings>

#include "TimeView.h"
#include "Platform/Thread.h"
#include "Controller/RobotConsole.h"
#include "Controller/RoboCupCtrl.h"
#include "Controller/Visualization/HeaderedWidget.h"

class TimeWidget : public QWidget
{
public:
  TimeWidget(TimeView& timeView, QHeaderView* headerView, QWidget* parent) : QWidget(parent),
    timeView(timeView), headerView(headerView), noPen(Qt::NoPen)
  {
    setFocusPolicy(Qt::StrongFocus);
    setBackgroundRole(QPalette::Base);
    const QFontMetrics& fontMetrics(QApplication::fontMetrics());
    lineSpacing = fontMetrics.lineSpacing() + 2;
    textOffset = fontMetrics.descent() + 1;

    font = QApplication::font();

    const QPalette& pal(QApplication::palette());
    altBrush = pal.alternateBase();
    fontPen.setColor(pal.text().color());

    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
    settings.beginGroup(timeView.fullName);
    headerView->restoreState(settings.value("HeaderState").toByteArray());
    settings.endGroup();
  }

  virtual ~TimeWidget()
  {
    QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
    settings.beginGroup(timeView.fullName);
    settings.setValue("HeaderState", headerView->saveState());
    settings.endGroup();
  }

  void update()
  {
    {
      SYNC_WITH(timeView.console);
      if(timeView.info.timeStamp == lastTimeInfoTimeStamp)
        return;
      lastTimeInfoTimeStamp = timeView.info.timeStamp;
    }

    QWidget::update();
  }

  void paintEvent(QPaintEvent* event)
  {
    painter.begin(this);
    painter.setFont(font);
    painter.setBrush(altBrush);
    painter.setPen(fontPen);
    fillBackground = false;

    paintRect = painter.window();
    paintRectField0 = QRect(headerView->sectionViewportPosition(0) + textOffset, 0, headerView->sectionSize(0) - textOffset * 2, lineSpacing);
    paintRectField1 = QRect(headerView->sectionViewportPosition(1) + textOffset, 0, headerView->sectionSize(1) - textOffset * 2, lineSpacing);
    paintRectField2 = QRect(headerView->sectionViewportPosition(2) + textOffset, 0, headerView->sectionSize(2) - textOffset * 2, lineSpacing);
    paintRectField3 = QRect(headerView->sectionViewportPosition(3) + textOffset, 0, headerView->sectionSize(3) - textOffset * 2, lineSpacing);
    paintRectField4 = QRect(headerView->sectionViewportPosition(4) + textOffset, 0, headerView->sectionSize(4) - textOffset * 2, lineSpacing);
    paintRectField5 = QRect(headerView->sectionViewportPosition(5) + textOffset, 0, headerView->sectionSize(5) - textOffset * 2, lineSpacing);
    paintRectField6 = QRect(headerView->sectionViewportPosition(6) + textOffset, 0, headerView->sectionSize(6) - textOffset * 2, lineSpacing);
    {
      SYNC_WITH(timeView.console);

      for(TimeInfo::Infos::const_iterator i = timeView.info.infos.begin(), end = timeView.info.infos.end(); i != end; ++i)
      {
        float minTime = -1, maxTime = -1, avgTime = -1, minDelta = -1, maxDelta = -1, avgFreq = -1;
        char fminTime[100], fmaxTime[100], favgTime[100], fminDelta[100], fmaxDelta[100], favgFreq[100];
        if(timeView.info.getStatistics(i->second, minTime, maxTime, avgTime, minDelta, maxDelta, avgFreq))
        {
          sprintf(fminTime, "%.03f", minTime / 1000);
          sprintf(fmaxTime, "%.03f", maxTime / 1000);
          sprintf(favgTime, "%.03f", avgTime / 1000);
          sprintf(fminDelta, "%.03f", minDelta);
          sprintf(fmaxDelta, "%.03f", maxDelta);
          sprintf(favgFreq, "%.03f", avgFreq);
          print(i->first.c_str(), fminTime, fmaxTime, favgTime, fminDelta, fmaxDelta, favgFreq);
          newBlock();
        }
      }
    }
    painter.end();
    setMinimumHeight(paintRectField1.top());
  }

private:
  TimeView& timeView;
  unsigned int lastTimeInfoTimeStamp; /**< Timestamp of the last painted info. */
  QHeaderView* headerView;
  QPainter painter;
  int lineSpacing;
  int textOffset;

  QFont font;
  QBrush altBrush;
  QPen fontPen;
  QPen noPen;
  bool fillBackground;

  QRect paintRect;
  QRect paintRectField0;
  QRect paintRectField1;
  QRect paintRectField2;
  QRect paintRectField3;
  QRect paintRectField4;
  QRect paintRectField5;
  QRect paintRectField6;

  void print(const char* name, const char* value1, const char* value2, const char* value3, const char* value4, const char* value5, const char* value6)
  {
    if(fillBackground)
    {
      painter.setPen(noPen);
      painter.drawRect(paintRect.left(), paintRectField1.top(), paintRect.width(), paintRectField1.height());
      painter.setPen(fontPen);
    }
    painter.drawText(paintRectField0, Qt::TextSingleLine | Qt::AlignVCenter, name);
    painter.drawText(paintRectField1, Qt::TextSingleLine | Qt::AlignVCenter | Qt::AlignRight, value1);
    painter.drawText(paintRectField2, Qt::TextSingleLine | Qt::AlignVCenter | Qt::AlignRight, value2);
    painter.drawText(paintRectField3, Qt::TextSingleLine | Qt::AlignVCenter | Qt::AlignRight, value3);
    painter.drawText(paintRectField4, Qt::TextSingleLine | Qt::AlignVCenter | Qt::AlignRight, value4);
    painter.drawText(paintRectField5, Qt::TextSingleLine | Qt::AlignVCenter | Qt::AlignRight, value5);
    painter.drawText(paintRectField6, Qt::TextSingleLine | Qt::AlignVCenter | Qt::AlignRight, value6);
    paintRectField0.moveTop(paintRectField0.top() + lineSpacing);
    paintRectField1.moveTop(paintRectField1.top() + lineSpacing);
    paintRectField2.moveTop(paintRectField2.top() + lineSpacing);
    paintRectField3.moveTop(paintRectField3.top() + lineSpacing);
    paintRectField4.moveTop(paintRectField4.top() + lineSpacing);
    paintRectField5.moveTop(paintRectField5.top() + lineSpacing);
    paintRectField6.moveTop(paintRectField6.top() + lineSpacing);
  }

  void newBlock()
  {
    fillBackground = fillBackground ? false : true;
  }
};

class TimeHeaderedWidget : public HeaderedWidget, public SimRobot::Widget
{
public:
  TimeHeaderedWidget(TimeView& timeView)
  {
    QStringList headerLabels;
    headerLabels << "Module" << "Min" << "Max" << "Avg" << "MinDelta" << "MaxDelta" << "AvgFreq";
    setHeaderLabels(headerLabels, "lrrrrrr");
    QHeaderView* headerView = getHeaderView();
    headerView->setMinimumSectionSize(30);
    headerView->setDefaultSectionSize(30);
    headerView->resizeSection(0, 100);
    headerView->resizeSection(1, 50);
    headerView->resizeSection(2, 50);
    headerView->resizeSection(3, 50);
    headerView->resizeSection(4, 50);
    headerView->resizeSection(5, 50);
    headerView->resizeSection(6, 50);
    timeWidget = new TimeWidget(timeView, headerView, this);
    setWidget(timeWidget);
  }

private:
  TimeWidget* timeWidget;
  virtual QWidget* getWidget() {return this;}
  virtual void update() {timeWidget->update();}

};

TimeView::TimeView(const QString& fullName, RobotConsole& console, const TimeInfo& info) :
  fullName(fullName), icon(":/Icons/tag_green.png"), console(console), info(info) {}

SimRobot::Widget* TimeView::createWidget()
{
  return new TimeHeaderedWidget(*this);
}
