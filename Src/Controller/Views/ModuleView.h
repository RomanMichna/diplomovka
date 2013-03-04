/**
* @file Controller/Views/ModuleView.h
* Declaration of a class to represent a view displaying the module layout of the process.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#pragma once

#include <QString>
#include <QIcon>
#include <QWidget>
#include <QPainter>
#include <string>

#include "SimRobot.h"

class RobotConsole;
class QPixmap;

/**
* @class ModuleView
* A class to represent a view displaying the module layout of the process.
*/
class ModuleView : public SimRobot::Object
{
public:
  /**
  * Constructor.
  * @param fullName The path to this view in the scene graph
  * @param console The console object.
  * @param processIdentifier The identifier of the process the modules of which are displayed.
  * @param category The category of the modules of this view. If "", show all categories.
  */
  ModuleView(const QString& fullName, RobotConsole& console, char processIdentifier, const std::string& category);

  /**
  * Destructor.
  */
  ~ModuleView();

private:
  const QString fullName; /**< The path to this view in the scene graph */
  const QIcon icon; /**< The icon used for listing this view in the scene graph */
  RobotConsole& console; /**< A reference to the console object. */
  char processIdentifier; /**< The name of the view. */
  std::string category; /**< The category of the modules of this view. If "", show all categories. */
  QPixmap* image; /**< The module graph image. */
  unsigned int lastModulInfoTimeStamp; /**< Module Info timestamp when the image was created. */

  /**
  * The method returns a new instance of a widget for this direct view.
  * The caller has to delete this instance. (Qt handles this)
  * @return The widget.
  */
  virtual SimRobot::Widget* createWidget();

  QString generateDotCommand(const char* fmt, const QString& src, const QString& dest) const;

  /**
  * The method creates the dot file of the module graph.
  * @param fileName The name of the file created.
  * @return Were any nodes generated?
  */
  bool generateDotFile(const char* fileName);

  /**
  * The method replaces all ' ' by '_'.
  * @param s The input string.
  * @return The string in which spaces were replaced.
  */
  std::string compress(const std::string& s) const;

  /**
  * The method creates the image of the module graph.
  */
  void generateImage();

  virtual const QString& getFullName() const {return fullName;}
  virtual const QIcon* getIcon() const {return &icon;}

  friend class ModuleWidget;
};

class ModuleWidget : public QWidget, public SimRobot::Widget
{
  Q_OBJECT

public:
  ModuleWidget(ModuleView& modelView);

private:
  float scale;
  QPoint offset;

  ModuleView& moduleView;

  QPainter painter;
  QPoint dragStart;
  QPoint dragStartOffset;

  virtual ~ModuleWidget();

  bool changeScale(float change);

  virtual void paintEvent(QPaintEvent* event);
  virtual void mousePressEvent(QMouseEvent* event);
  virtual void mouseReleaseEvent(QMouseEvent* event);
  virtual void mouseMoveEvent(QMouseEvent* event);
  virtual void keyPressEvent(QKeyEvent* event);
  virtual void wheelEvent(QWheelEvent* event);
  virtual void mouseDoubleClickEvent(QMouseEvent* event);

  virtual QSize sizeHint() const { return QSize(640, 480); }

  virtual QWidget* getWidget() {return this;}
  virtual void update();
  virtual QMenu* createEditMenu();
  virtual QMenu* createUserMenu();

private slots:
  void copy();
  void exportAsSvg();
  void exportAsDot();
};
