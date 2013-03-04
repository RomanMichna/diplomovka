/**
* @file Controller/Views/ModuleView.cpp
*
* Implementation of a class to represent a view displaying the module layout of the process.
*
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
* @author Colin Graf
*/

#include <algorithm>
#include <QPixmap>
#include <QFile>
#include <QDir>
#include <QProcess>
#include <QWidget>
#include <QPainter>
#include <QMouseEvent>
#include <QTextStream>
#include <QSettings>
#include <QApplication>
#include <QClipboard>
#include <QFileDialog>
#include <QMenu>

#include "Platform/Thread.h"
#ifdef WIN32
#include "Platform/File.h"
#endif

#include "ModuleView.h"
#include "Controller/RobotConsole.h"
#include "Controller/RoboCupCtrl.h"

ModuleWidget::ModuleWidget(ModuleView& moduleView) : QWidget(),
  scale(1.0f), moduleView(moduleView), dragStart(-1, -1)
{
  setFocusPolicy(Qt::StrongFocus);
  setBackgroundRole(QPalette::Base);

  QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
  settings.beginGroup(moduleView.fullName);
  scale = (float)settings.value("Scale", 1.).toDouble();
  offset = settings.value("Offset", QPoint()).toPoint();
  settings.endGroup();
}

ModuleWidget::~ModuleWidget()
{
  QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
  settings.beginGroup(moduleView.fullName);
  settings.setValue("Scale", (double)scale);
  settings.setValue("Offset", offset);
  settings.endGroup();
}

bool ModuleWidget::changeScale(float change)
{
  if((scale <= 0.1f && change < 0) || (scale >= 2.0f && change > 0))
    return false;
  scale += change;
  if(scale < 0.1f)
    scale = 0.1f;
  if(scale > 2.0f)
    scale = 2.0f;
  return true;
}

void ModuleWidget::paintEvent(QPaintEvent* event)
{
  if(!moduleView.image)
    return;

  painter.begin(this);
  const QRect& windowRect(painter.window());
  QSizeF maxSize(windowRect.size());
  maxSize *= scale;
  QSizeF size(moduleView.image->size());
  size -= QSizeF(offset.x(), offset.y());
  if(size.width() > maxSize.width())
    size.setWidth(maxSize.width());
  if(size.height() > maxSize.height())
    size.setHeight(maxSize.height());
  painter.drawPixmap(QRectF(QPointF(0, 0), size / scale), *moduleView.image, QRectF(offset, size));
  painter.end();
}

void ModuleWidget::mousePressEvent(QMouseEvent* event)
{
  QWidget::mousePressEvent(event);

  if((event->button() == Qt::LeftButton || event->button() == Qt::MidButton) && moduleView.image)
  {
    dragStart = event->pos();
    dragStartOffset = offset;
  }
}

void ModuleWidget::mouseReleaseEvent(QMouseEvent* event)
{
  QWidget::mouseReleaseEvent(event);

  dragStart = QPoint(-1, -1);
}

void ModuleWidget::mouseMoveEvent(QMouseEvent* event)
{
  QWidget::mouseMoveEvent(event);

  if(dragStart.x() >= 0 && moduleView.image)
  {
    const QPoint& dragPos(event->pos());
    offset = dragStartOffset + (dragStart - dragPos);
    QWidget::update();
  }
}

void ModuleWidget::keyPressEvent(QKeyEvent* event)
{
  if(!moduleView.image)
  {
    QWidget::keyPressEvent(event);
    return;
  }

  switch(event->key())
  {
  case Qt::Key_PageUp:
  case Qt::Key_Plus:
    event->accept();
    changeScale(0.1f);
    QWidget::update();
    break;
  case Qt::Key_PageDown:
  case Qt::Key_Minus:
    event->accept();
    changeScale(-0.1f);
    QWidget::update();
    break;
  default:
    QWidget::keyPressEvent(event);
    break;
  }
}

void ModuleWidget::wheelEvent(QWheelEvent* event)
{
  QWidget::wheelEvent(event);

  if(!moduleView.image)
    return;

  changeScale(0.1 * event->delta() / 160);
  QWidget::update();
}

void ModuleWidget::mouseDoubleClickEvent(QMouseEvent* event)
{
  QWidget::mouseDoubleClickEvent(event);

  if(!moduleView.image)
    return;

  scale = 1.0f;
  offset = QPoint(0, 0);
  QWidget::update();
}

void ModuleWidget::update()
{
  {
    SYNC_WITH(moduleView.console);
    if(moduleView.lastModulInfoTimeStamp == moduleView.console.moduleInfo.timeStamp)
      return;
  }
  moduleView.generateImage();
  QWidget::update();
}

QMenu* ModuleWidget::createEditMenu()
{
  QMenu* menu = new QMenu(tr("&Edit"));
  QAction* action = menu->addAction(QIcon(":/Icons/page_copy.png"), tr("&Copy"));
  action->setShortcut(QKeySequence(QKeySequence::Copy));
  action->setStatusTip(tr("Copy the window drawing to the clipboard"));
  connect(action, SIGNAL(triggered()), this, SLOT(copy()));
  return menu;
}

QMenu* ModuleWidget::createUserMenu()
{
  QMenu* menu = new QMenu(tr("&Graph"));
  QAction* action = menu->addAction(tr("&Export as SVG..."));
  action->setStatusTip(tr("Export the window drawing as svg"));
  connect(action, SIGNAL(triggered()), this, SLOT(exportAsSvg()));
  action = menu->addAction(tr("Export as &DOT..."));
  action->setStatusTip(tr("Export the window drawing as dot"));
  connect(action, SIGNAL(triggered()), this, SLOT(exportAsDot()));
  return menu;
}

void ModuleWidget::copy()
{
  if(!moduleView.image)
    return;
  QApplication::clipboard()->clear();
  QApplication::clipboard()->setPixmap(*moduleView.image);
}

void ModuleWidget::exportAsSvg()
{
  QSettings& settings = RoboCupCtrl::application->getSettings();
  QString fileName = QFileDialog::getSaveFileName(this,
                     tr("Export as SVG"), settings.value("ExportDirectory", "").toString(), tr("Scalable Vector Graphics (*.svg)"));
  if(fileName.isEmpty())
    return;
  settings.setValue("ExportDirectory", QFileInfo(fileName).dir().path());

  const QString& dotName(QDir::temp().filePath("ModuleView.dot"));
  if(moduleView.generateDotFile(dotName.toAscii().constData()))
  {
    QString cmd = moduleView.generateDotCommand("svg", dotName, fileName);
    if(QProcess::execute(cmd) == 0)
    {
      // success
    }
  }
  QFile::remove(dotName);
}

void ModuleWidget::exportAsDot()
{
  QSettings& settings = RoboCupCtrl::application->getSettings();
  QString fileName = QFileDialog::getSaveFileName(this,
                     tr("Export as DOT"), settings.value("ExportDirectory", "").toString(), tr("Graphviz Graph File (*.dot)"));
  if(fileName.isEmpty())
    return;
  settings.setValue("ExportDirectory", QFileInfo(fileName).dir().path());

  moduleView.generateDotFile(fileName.toAscii().constData());
}

ModuleView::ModuleView(const QString& fullName, RobotConsole& console, char processIdentifier, const std::string& category) :
  fullName(fullName), icon(":/Icons/tag_green.png"), console(console), processIdentifier(processIdentifier), category(category),
  image(0), lastModulInfoTimeStamp(0) {}

ModuleView::~ModuleView()
{
  if(image)
    delete image;
}

SimRobot::Widget* ModuleView::createWidget()
{
  ModuleWidget* widget = new ModuleWidget(*this);
  return widget;
}

QString ModuleView::generateDotCommand(const char* fmt, const QString& src, const QString& dest) const
{
#ifdef WIN32
  return QString("\"%1\\Util\\dot-1.9.0\\dot.exe\" -T%2 -o \"%3\" \"%4\"").arg(File::getBHDir(), QString(fmt), dest, src);
#elif defined(MACOSX) // search path is ignored
  return QString("/usr/local/bin/dot -T%1 -o \"%2\" \"%3\"").arg(QString(fmt), dest, src);
#else
  return QString("dot -T%1 -o \"%2\" \"%3\"").arg(QString(fmt), dest, src);
#endif
}

bool ModuleView::generateDotFile(const char* fileName)
{
  SYNC_WITH(console);
  const ModuleInfo& m = console.moduleInfo;
  bool success = false;
  std::ofstream stream(fileName);
  stream << "digraph G {" << std::endl;
  stream << "node [style=filled,fillcolor=lightyellow,fontname=Arial,fontsize=9,height=0.2];" << std::endl;
  stream << "concentrate = true;" << std::endl;

  for(std::list<ModuleInfo::Module>::const_iterator i = m.modules.begin(); i != m.modules.end(); ++i)
    if(i->processIdentifier == processIdentifier && (category == "" || i->category == category))
    {
      bool used = false;
      for(std::vector<std::string>::const_iterator j = i->representations.begin(); j != i->representations.end(); ++j)
      {
        std::list<ModuleInfo::Provider>::const_iterator k = std::find(m.providers.begin(), m.providers.end(), *j);
        while(k != m.providers.end() && k->selected != i->name)
          k = std::find(++k, m.providers.end(), *j);
        if(k != m.providers.end())
        {
          used = true;
          stream << j->c_str() << " [style=filled,fillcolor=\"lightblue\"];" << std::endl;
          stream << i->name << "->" << j->c_str() << " [len=2];" << std::endl;
        }
      }
      if(used)
      {
        for(std::vector<std::string>::const_iterator j = i->requirements.begin(); j != i->requirements.end(); ++j)
        {
          std::list<ModuleInfo::Provider>::const_iterator k = std::find(m.providers.begin(), m.providers.end(), *j);
          while(k != m.providers.end() && (k->selected == "" || k->processIdentifier != processIdentifier))
            k = std::find(++k, m.providers.end(), *j);
          if(k == m.providers.end())
          {
            // no provider in same process
            k = std::find(m.providers.begin(), m.providers.end(), *j);
            while(k != m.providers.end() && k->selected == "")
              k = std::find(++k, m.providers.end(), *j);
            if(k == m.providers.end())
              stream << j->c_str() << " [style=\"filled,dashed\",color=red,fontcolor=red];" << std::endl;
            else
              stream << j->c_str() << " [style=\"filled,dashed\",fillcolor=\"#ffdec4\"];" << std::endl;
          }
          stream << j->c_str() << "->" << i->name << " [len=2];" << std::endl;
        }
        if(i->name == "default")
          stream << i->name.c_str() << "[shape=box,fontcolor=red];" << std::endl;
        else
          stream << i->name.c_str() << "[shape=box];" << std::endl;
        success = true;
      }
    }

  stream << "}" << std::endl;
  return success;
}

std::string ModuleView::compress(const std::string& s) const
{
  std::string s2(s);
  if(!isalpha(s2[0]))
    s2[0] = '_';
  for(unsigned i = 1; i < s2.size(); ++i)
    if(!isalpha(s2[i]) && !isdigit(s2[i]))
      s2[i] = '_';
  return s2;
}

void ModuleView::generateImage()
{
  if(image)
  {
    delete image;
    image = 0;
  }

  const QString& dotName(QDir::temp().filePath("ModuleView.dot"));
  if(generateDotFile(dotName.toAscii().constData()))
  {
    const QString& pngName(QDir::temp().filePath("ModuleView.png"));
    QString cmd = generateDotCommand("png", dotName, pngName);
    if(QProcess::execute(cmd) == 0)
    {
      image = new QPixmap(pngName);
      if(image->isNull())
      {
        delete image;
        image = 0;
      }
      QFile::remove(pngName);
    }
    SYNC_WITH(console);
    lastModulInfoTimeStamp = console.moduleInfo.timeStamp;
  }

  QFile::remove(dotName);
}
