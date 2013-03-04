/**
* @file Controller/Views/StateMachineBehaviorView.cpp
* Implementation of class StateMachineBehaviorView
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
* @author Colin Graf
*/

#include <QHeaderView>
#include <QApplication>
#include <QPainter>
#include <QFontMetrics>
#include <QKeyEvent>
#include <QSettings>

#include "StateMachineBehaviorView.h"
#include "Platform/Thread.h"
#include "Controller/RobotConsole.h"
#include "Controller/RoboCupCtrl.h"
#include "Controller/Visualization/HeaderedWidget.h"

class StateMachineBehaviorWidget : public QWidget
{
public:
  StateMachineBehaviorWidget(StateMachineBehaviorView& stateMachineBehaviorView/*, QHeaderView* headerView*/, QWidget* parent) : QWidget(parent),
    stateMachineBehaviorView(stateMachineBehaviorView)/*, headerView(headerView)*/, noPen(Qt::NoPen)
  {
    setFocusPolicy(Qt::StrongFocus);
    setBackgroundRole(QPalette::Base);

    font = QApplication::font();
    boldFont = font;
    boldFont.setBold(true);
    setFontPointSize(getFontPointSize());
    
    const QPalette& pal(QApplication::palette());
    altBrush = pal.alternateBase();
    fontPen.setColor(pal.text().color());
    
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);    
    /*
    QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
    settings.beginGroup(StateMachineBehaviorView.fullName);
    headerView->restoreState(settings.value("HeaderState").toByteArray());
    settings.endGroup();
    */
  }

  virtual ~StateMachineBehaviorWidget()
  {
    /*
    QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
    settings.beginGroup(StateMachineBehaviorView.fullName);
    settings.setValue("HeaderState", headerView->saveState());
    settings.endGroup();
    */
  }

  void update()
  {
    {
      SYNC_WITH(stateMachineBehaviorView.console);
      if(stateMachineBehaviorView.info.timeStamp == lastStateMachineBehaviorDebugInfoTimeStamp)
        return;
      lastStateMachineBehaviorDebugInfoTimeStamp = stateMachineBehaviorView.info.timeStamp;
    }
    QWidget::update();
  }
  
  int getFontPointSize()
  {
    return font.pointSize();
  }
  
  void setFontPointSize(int size)
  {
    font.setPointSize(size);
    boldFont.setPointSize(size);
    QFontMetrics me(font);
    lineSpacing = me.lineSpacing() + 2;
    textOffset = me.descent() + 1;
  }

  void paintEvent(QPaintEvent *event)
  {
    painter.begin(this);
    painter.setFont(font);
    painter.setBrush(altBrush);
    painter.setPen(fontPen);
    fillBackground = false;

    char formatedTime[65];

    paintRect = painter.window();
    int defaultLeft = /*headerView->sectionViewportPosition(0) + */textOffset;
    paintRectField0 = QRect(defaultLeft, 0, /*headerView->sectionSize(0)*/ paintRect.right() - textOffset * 2, lineSpacing);
    //paintRectField1 = QRect(headerView->sectionViewportPosition(1) + textOffset, 0, headerView->sectionSize(1) - textOffset * 2, lineSpacing);
    {
      SYNC_WITH(stateMachineBehaviorView.console);
      const StateMachineBehaviorInfo& info(stateMachineBehaviorView.info);

      for(std::vector<StateMachineBehaviorInfo::ActiveOption>::const_iterator i = info.activeOptions.begin(), end = info.activeOptions.end(); i != end; ++i)
      {
        const StateMachineBehaviorInfo::ActiveOption& activeOption = *i;
        const StateMachineBehaviorInfo::Option& option = info.options[activeOption.option];
        const std::string* state = activeOption.state < option.states.size() ? &option.states[activeOption.state] : 0;

        paintRectField0.setLeft(defaultLeft + 10 * (activeOption.depth /*+ 1*/));

        sprintf(formatedTime, "%.02f", float(activeOption.optionTime) / 1000.f);
        print(option.name.c_str(), formatedTime, true, true);

        paintRectField0.setLeft(paintRectField0.left() + 5);
        sprintf(formatedTime, "%.02f", float(activeOption.stateTime) / 1000.f);
        print(state ? state->c_str() : "", formatedTime, false, true);
        newBlock();
      }
    }
    painter.end();
    int minHeight = paintRectField0.top();
    if(minHeight > 0)
      setMinimumHeight(minHeight);
  }

private:
  StateMachineBehaviorView& stateMachineBehaviorView;
  unsigned int lastStateMachineBehaviorDebugInfoTimeStamp; /**< Timestamp of the last painted info. */
  //QHeaderView* headerView;
  QPainter painter;
  int lineSpacing;
  int textOffset;

  QFont font;
  QFont boldFont;
  QBrush altBrush;
  QPen fontPen;
  QPen noPen;
  bool fillBackground;

  QRect paintRect;
  QRect paintRectField0;
  //QRect paintRectField1;

  void print(const char* name, const char* value, bool bold = false, bool rightAlign = false)
  {
    if(fillBackground)
    {
      painter.setPen(noPen);
      painter.drawRect(paintRect.left(), paintRectField0.top(), paintRect.width(), paintRectField0.height());
      painter.setPen(fontPen);
    }
    if(name)
    {
      if(bold)
        painter.setFont(boldFont);
      painter.drawText(paintRectField0, ((!value /*|| rightAlign*/) ? Qt::TextDontClip : 0) | Qt::TextSingleLine | Qt::AlignVCenter, name);
      if(bold)
        painter.setFont(font);
    }
    if(value)
      painter.drawText(paintRectField0, (rightAlign ? Qt::AlignRight : Qt::AlignLeft) | Qt::TextSingleLine | Qt::AlignVCenter, value);
    paintRectField0.moveTop(paintRectField0.top() + lineSpacing);
    //paintRectField1.moveTop(paintRectField1.top() + lineSpacing);
  }
  
  void newBlock()
  {
    fillBackground = fillBackground ? false : true;
  }

  void newSection()
  {
    painter.drawLine(paintRect.left(), paintRectField0.top(), paintRect.width(), paintRectField0.top());
    paintRectField0.moveTop(paintRectField0.top() + 1);
    //paintRectField1.moveTop(paintRectField1.top() + 1);
    fillBackground = false;
  }

  void keyPressEvent(QKeyEvent* event)
  {
    switch(event->key())
    {
    case Qt::Key_PageUp:
    case Qt::Key_Plus:
      event->accept();
      if(getFontPointSize() < 48)
        setFontPointSize(getFontPointSize() + 1);
      QWidget::update();
      break;
    case Qt::Key_PageDown:
    case Qt::Key_Minus:
      event->accept();
      if(getFontPointSize() > 3)
        setFontPointSize(getFontPointSize() - 1);
      QWidget::update();
      break;
    default:
      QWidget::keyPressEvent(event);
      break;
    }
  }

  QSize sizeHint () const { return QSize(200, 500); }
};

class StateMachineBehaviorHeaderedWidget : public /*HeaderedWidget*/QScrollArea, public SimRobot::Widget
{
public:
  StateMachineBehaviorHeaderedWidget(StateMachineBehaviorView& xabslView)
  {
    /*QStringList headerLabels;
    headerLabels << "Name" << "Value";
    setHeaderLabels(headerLabels);
    QHeaderView* headerView = getHeaderView();
    headerView->setMinimumSectionSize(30);
    headerView->resizeSection(0, 100);
    headerView->resizeSection(1, 100);
    */

    setFrameStyle(QFrame::NoFrame);
    setWidgetResizable(true);

    stateMachineBehaviorWidget = new StateMachineBehaviorWidget(xabslView/*, headerView*/, this);
    setWidget(stateMachineBehaviorWidget);
    setFocusProxy(stateMachineBehaviorWidget);
  }

private:
  StateMachineBehaviorWidget* stateMachineBehaviorWidget;
  virtual QWidget* getWidget() {return this;}
  virtual void update() {stateMachineBehaviorWidget->update();}
};

StateMachineBehaviorView::StateMachineBehaviorView(const QString& fullName, RobotConsole& console, const StateMachineBehaviorInfo& info) : 
  fullName(fullName), icon(":/Icons/tag_green.png"), console(console), info(info) {}

SimRobot::Widget* StateMachineBehaviorView::createWidget()
{
  return new StateMachineBehaviorHeaderedWidget(*this);
  //return new StateMachineBehaviorWidget(this/*, headerView*/, this);
}
