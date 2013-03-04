/**
* @file Controller/Views/ImageView.h
*
* Declaration of class ImageView
*
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
* @author Colin Graf
*/

#pragma once

#include <QString>
#include <QIcon>
#include <QPainter>
#include <QApplication>
#include <QMouseEvent>
#include <QWidget>
#include <QSettings>
#include <QMenu>

#include "SimRobot.h"
#include "Tools/Math/Vector2.h"
#include "Controller/RobotConsole.h"
#include "Controller/RoboCupCtrl.h"
#include "Controller/Visualization/PaintMethods.h"
#include "Controller/ImageViewAdapter.h"
#include "Representations/Infrastructure/Image.h"
#include "Platform/Thread.h"

#include "Controller/Representations/AutoColorTableCreator.h"


class RobotConsole;

/**
* @class ImageView
*
* A class to represent a view displaying camera images and overlaid debug drawings.
*
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/
class ImageView : public SimRobot::Object
{
public:
  /**
  * Constructor.
  * @param fullName The path to this view in the scene graph.
  * @param console The console object.
  * @param background The name of the background image.
  * @param name The name of the view.
  * @param segmented The image will be segmented.
  * @param gain The intensity is multiplied with this factor.
  */
  ImageView(const QString& fullName, RobotConsole& console, const std::string& background, const std::string& name, bool segmented, float gain = 1.0f);

private:
  const QString fullName; /**< The path to this view in the scene graph */
  const QIcon icon; /**< The icon used for listing this view in the scene graph */
  RobotConsole& console; /**< A reference to the console object. */
  const std::string background; /**< The name of the background image. */
  const std::string name; /**< The name of the view. */
  bool segmented;  /**< The image will be segmented. */
  float gain; /**< The intensity is multiplied with this factor. */

  /**
  * The method returns a new instance of a widget for this direct view.
  * The caller has to delete this instance. (Qt handles this)
  * @return The widget.
  */
  virtual SimRobot::Widget* createWidget();

  virtual const QString& getFullName() const {return fullName;}
  virtual const QIcon* getIcon() const {return &icon;}

  friend class ImageWidget;
};


class ImageWidget : public QWidget, public SimRobot::Widget
{
  Q_OBJECT
public:
  ImageWidget(ImageView& imageView);
  virtual ~ImageWidget();

private:
  ImageView& imageView;
  QImage* imageData;
  int imageWidth;
  int imageHeight;
  unsigned int lastImageTimeStamp;
  unsigned int lastColorTableTimeStamp;
  unsigned int lastDrawingsTimeStamp;
  QPainter painter;
  QPoint dragStart;
  QPoint dragPos;
  float zoom;
  QPoint offset;

  void paintEvent(QPaintEvent* event);
  virtual void paint(QPainter& painter);
  void paintDrawings(QPainter& painter);
  void copyImage(const Image& srcImage);
  void copyImageSegmented(const Image& srcImage);
  void paintImage(QPainter& painter, const Image& srcImage);
  bool needsRepaint() const;
  void window2viewport(QPoint& point);
  void mousePressEvent(QMouseEvent* event);
  void mouseMoveEvent(QMouseEvent* event);
  void mouseReleaseEvent(QMouseEvent* event);
  void keyPressEvent(QKeyEvent* event);
  void wheelEvent(QWheelEvent* event);
  void mouseDoubleClickEvent(QMouseEvent* event);
  QSize sizeHint() const { return QSize(imageWidth, imageHeight); }
  virtual QWidget* getWidget() {return this;}
  virtual void update()
  {
    if(needsRepaint())
      QWidget::update();
  }

  virtual QMenu* createUserMenu();

  friend class ImageView;

private slots:

  void kdTraining()
  {
    imageView.console.colorTableHandler.enableTraining();
    imageView.console.colorTableHandler.active = true;
  }

  void kdUndo()
  {
    imageView.console.colorTableHandler.undoLastHandData();
  }
  void kdClear()
  {
    imageView.console.colorTableHandler.resetKDAuto();
    imageView.console.colorTableHandler.resetKDHand();
  }

  void kdReclassify()
  {
    imageView.console.colorTableHandler.reclassify();
  }

  void kdSelectGreen()
  {
    kdTraining();
    imageView.console.colorTableHandler.selectedColorClass = ColorClasses::green;
  }
  void kdSelectBlue()
  {
    kdTraining();
    imageView.console.colorTableHandler.selectedColorClass = ColorClasses::blue;
  }
  void kdSelectWhite()
  {
    kdTraining();
    imageView.console.colorTableHandler.selectedColorClass = ColorClasses::white;
  }
  void kdSelectNone()
  {
    kdTraining();
    imageView.console.colorTableHandler.selectedColorClass = ColorClasses::none;
  }
  void kdSelectYellow()
  {
    kdTraining();
    imageView.console.colorTableHandler.selectedColorClass = ColorClasses::yellow;
  }
  void kdSelectOrange()
  {
    kdTraining();
    imageView.console.colorTableHandler.selectedColorClass = ColorClasses::orange;
  }
  void kdSelectRed()
  {
    kdTraining();
    imageView.console.colorTableHandler.selectedColorClass = ColorClasses::red;
  }
  void kdSelectRobotBlue()
  {
    kdTraining();
    imageView.console.colorTableHandler.selectedColorClass = ColorClasses::robotBlue;
  }
  
  void waistbandsOn()
  {
    imageView.console.handleConsole("vid raw module:RobotPerceptor:scanlines");
  }
  
  void headAngle()
  {
    imageView.console.handleConsole("set representation:HeadAngleRequest pan = 1000; tilt = 1000; speed = 2.61799;");
  }
  
  void camUpOn()
  {
    if(!imageView.console.imageInfo.toggling)
      imageView.console.handleConsole("set representation:ImageRequest requestedCamera = upperCamera;");
    imageView.console.handleConsole("ac upper");
  }
  
  void camDownOn()
  {
    if(!imageView.console.imageInfo.toggling)
      imageView.console.handleConsole("set representation:ImageRequest requestedCamera = lowerCamera;");
    imageView.console.handleConsole("ac lower");
  }
};
