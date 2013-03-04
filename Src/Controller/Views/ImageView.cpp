/**
 * @file Controller/Views/ImageView.cpp
 *
 * Implementation of class ImageView
 *
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas R�fer</a>
 * @author Colin Graf
 */

#include <QPainter>
#include <QApplication>
#include <QMouseEvent>
#include <QWidget>
#include <QSettings>
#include <QMenu>

#include "ImageView.h"
#include "Controller/RobotConsole.h"
#include "Controller/RoboCupCtrl.h"
#include "Controller/Visualization/PaintMethods.h"
#include "Controller/ImageViewAdapter.h"
#include "Representations/Infrastructure/Image.h"
#include "Platform/Thread.h"


ImageWidget::ImageWidget(ImageView& imageView) :
  imageView(imageView),
  imageData(0), imageWidth(cameraResolutionWidth), imageHeight(cameraResolutionHeight),
  lastImageTimeStamp(0), lastColorTableTimeStamp(0), lastDrawingsTimeStamp(0),
  dragStart(-1, -1), zoom(1.f), offset(0, 0)
{
  setFocusPolicy(Qt::StrongFocus);
  setMouseTracking(true);

  QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
  settings.beginGroup(imageView.fullName);
  zoom = (float)settings.value("Zoom", 1.).toDouble();
  offset = settings.value("Offset", QPoint()).toPoint();
  settings.endGroup();
}

ImageWidget::~ImageWidget()
{
  QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
  settings.beginGroup(imageView.fullName);
  settings.setValue("Zoom", (double)zoom);
  settings.setValue("Offset", offset);
  settings.endGroup();

  if(imageData)
    delete imageData;
}


void ImageWidget::paintEvent(QPaintEvent* event)
{
  painter.begin(this);
  paint(painter);

 /* if(dragStart.x() >= 0)
  {
    QPen pen(QColor(128, 128, 128, 128));
    pen.setWidth(1);
    painter.setPen(pen);
    painter.setBrush(QBrush(Qt::NoBrush));
    painter.drawRect(QRect(dragStart, dragPos));
  }*/

  painter.end();
}

void ImageWidget::paint(QPainter& painter)
{
  {
    SYNC_WITH(imageView.console);

    const Image* image = 0;
    RobotConsole::Images::const_iterator i = imageView.console.images.find(imageView.background);
    if(i != imageView.console.images.end())
    {
      image = i->second.image;
      imageWidth = image->resolutionWidth;
      imageHeight = image->resolutionHeight;
    }

    float scale, xOffset, yOffset;
    {
      const QSize& size = painter.window().size();
      float xScale = float(size.width()) / float(imageWidth);
      float yScale = float(size.height()) / float(imageHeight);
      scale = xScale < yScale ? xScale : yScale;
      scale *= zoom;
      xOffset = (float(size.width()) - float(imageWidth) * scale) * 0.5f + offset.x() * scale;
      yOffset = (float(size.height()) - float(imageHeight) * scale) * 0.5f + offset.y() * scale;
    }

    painter.setTransform(QTransform(scale, 0, 0, scale, xOffset, yOffset));

    if(image)
      paintImage(painter, *image);
    else
      lastImageTimeStamp = 0;

    paintDrawings(painter);
  }
}

void ImageWidget::paintDrawings(QPainter& painter)
{
  const QTransform baseTrans(painter.transform());
  const std::list<std::string>& drawings(imageView.console.imageViews[imageView.name]);
  for(std::list<std::string>::const_iterator i = drawings.begin(), end = drawings.end(); i != end; ++i)
  {
    const DebugDrawing& debugDrawing(imageView.console.imageDrawings[*i]);
    PaintMethods::paintDebugDrawing(painter, debugDrawing, baseTrans);
    if(debugDrawing.timeStamp > lastDrawingsTimeStamp)
      lastDrawingsTimeStamp = debugDrawing.timeStamp;
  }
  painter.setTransform(baseTrans);
}

void ImageWidget::copyImage(const Image& srcImage)
{
  int width = srcImage.resolutionWidth;
  int height = srcImage.resolutionHeight;

  static const int factor1 = 29016;
  static const int factor2 = 5662;
  static const int factor3 = 22972;
  static const int factor4 = 11706;
  unsigned* p = (unsigned*) imageData->bits();
  int r, g, b;
  int yImage, uImage, vImage;
  for(int y = 0; y < height; ++y)
  {
    const Image::Pixel* cur = &srcImage.image[y][0];
    const Image::Pixel* end = cur + width;
    for(; cur < end; ++cur)
    {
      yImage = int(cur->y) << 14;
      uImage = int(cur->cr) - 128;
      vImage = int(cur->cb) - 128;

      r = (yImage + factor3 * uImage) >> 14;
      g = (yImage - factor2 * vImage - factor4 * uImage) >> 14;
      b = (yImage + factor1 * vImage) >> 14;

      r = (int)(imageView.gain * (float) r);
      g = (int)(imageView.gain * (float) g);
      b = (int)(imageView.gain * (float) b);

      *p++ = (r < 0 ? 0 : r > 255 ? 255 : r) << 16 |
             (g < 0 ? 0 : g > 255 ? 255 : g) << 8 |
             (b < 0 ? 0 : b > 255 ? 255 : b) |
             0xff000000;
    }
  }
}

void ImageWidget::copyImageSegmented(const Image& srcImage)
{
  int width = srcImage.resolutionWidth;
  int height = srcImage.resolutionHeight;
  
  unsigned colorClassColors[256] =
  {
    0xff7f7f7f, // none
    0xffff7f00, // orange
    0xffffff00, // yellow
    0xff0000ff, // blue
    0xffffffff, // white
    0xff00ff00, // green
    0xff000000, // black
    0xffff0000, // red
    0xff00ffff, // robotBlue
  };
  
  if (imageView.name == "segmentedSolo") {
    if (imageView.console.colorTableHandler.selectedColorClass != ColorClasses::orange) {
      colorClassColors[1] = 0x00000000;
    }
    if (imageView.console.colorTableHandler.selectedColorClass != ColorClasses::yellow) {
      colorClassColors[2] = 0x00000000;
    }
    if (imageView.console.colorTableHandler.selectedColorClass != ColorClasses::blue) {
      colorClassColors[3] = 0x00000000;
    }
    if (imageView.console.colorTableHandler.selectedColorClass != ColorClasses::white) {
      colorClassColors[4] = 0x00000000;
    }
    if (imageView.console.colorTableHandler.selectedColorClass != ColorClasses::green) {
      colorClassColors[5] = 0x00000000;
    }
    if (imageView.console.colorTableHandler.selectedColorClass != ColorClasses::black) {
      colorClassColors[6] = 0x00000000;
    }
    if (imageView.console.colorTableHandler.selectedColorClass != ColorClasses::red) {
      colorClassColors[7] = 0x00000000;
    }
    if (imageView.console.colorTableHandler.selectedColorClass != ColorClasses::robotBlue) {
      colorClassColors[8] = 0x00000000;
    }
  }
  
  
  if(!colorClassColors[129])
  {
    for(int i = 129; i < 256; ++i)
    {
      int rSum = 0;
      int gSum = 0;
      int bSum = 0;
      int count = 0;
      for(int j = 0; j < 7; ++j)
        if(i & 1 << j)
        {
          unsigned color = colorClassColors[j + 1];
          bSum += color & 0xff;
          gSum += color >> 8 & 0xff;
          rSum += color >> 16 & 0xff;
          ++count;
        }
      colorClassColors[i] = (rSum / count) << 16 | (gSum / count) << 8 | (bSum / count);
    }
  }
  unsigned* p = (unsigned*) imageData->bits();
  unsigned char(*colorClasses)[64][64](imageView.console.colorTableHandler.colorTable.colorClasses);
  for(int y = 0; y < height; ++y)
  {
    const Image::Pixel* cur = &srcImage.image[y][0];
    const Image::Pixel* end = cur + width;
    for(; cur < end; ++cur)
    {
      unsigned char clrclass = colorClasses[cur->y >> 2][cur->cb >> 2][cur->cr >> 2];
      *p++ = colorClassColors[clrclass];
    }
  }
}

void ImageWidget::paintImage(QPainter& painter, const Image& srcImage)
{
  
  // make sure we have a buffer
  if(!imageData || imageWidth != imageData->width() || imageHeight != imageData->height())
  {
    if(imageData)
      delete imageData;
    imageData = new QImage(imageWidth, imageHeight, QImage::Format_RGB32);
  }

  if(srcImage.timeStamp != lastImageTimeStamp || (imageView.segmented && imageView.console.colorTableHandler.timeStamp != lastColorTableTimeStamp))
  {
    if(imageView.segmented)
      copyImageSegmented(srcImage);
    else
      copyImage(srcImage);

    lastImageTimeStamp = srcImage.timeStamp;
    if(imageView.segmented)
      lastColorTableTimeStamp = imageView.console.colorTableHandler.timeStamp;
  }

  if (imageView.name == "act") {
    AutoColorTableCreator act;
    copyImage(act.detectGreen(srcImage));
  }
  
  painter.drawImage(QRectF(0, 0, imageWidth, imageHeight), *imageData);
}

bool ImageWidget::needsRepaint() const
{
  SYNC_WITH(imageView.console);
  const std::list<std::string>& drawings(imageView.console.imageViews[imageView.name]);
  for(std::list<std::string>::const_iterator i = drawings.begin(), end = drawings.end(); i != end; ++i)
  {
    const DebugDrawing& debugDrawing(imageView.console.imageDrawings[*i]);
    if(debugDrawing.timeStamp > lastDrawingsTimeStamp)
      return true;
  }
  Image* image = 0;
  RobotConsole::Images::const_iterator j = imageView.console.images.find(imageView.background);
  if(j != imageView.console.images.end())
    image = j->second.image;
  return (image &&
          (image->timeStamp != lastImageTimeStamp ||
           (imageView.segmented && imageView.console.colorTableHandler.timeStamp != lastColorTableTimeStamp))) ||
         (!image && lastImageTimeStamp);
}

void ImageWidget::window2viewport(QPoint& point)
{
  float scale, xOffset, yOffset;
  {
    const QSize& size(this->size());
    float xScale = float(size.width()) / float(imageWidth);
    float yScale = float(size.height()) / float(imageHeight);
    scale = xScale < yScale ? xScale : yScale;
    scale *= zoom;
    xOffset = (float(size.width()) - float(imageWidth) * scale) * 0.5f + offset.x() * scale;
    yOffset = (float(size.height()) - float(imageHeight) * scale) * 0.5f + offset.y() * scale;
  }
  point = QPoint(static_cast<int>((point.x() - xOffset) / scale),
                 static_cast<int>((point.y() - yOffset) / scale));
}

void ImageWidget::mousePressEvent(QMouseEvent* event)
{
  QWidget::mousePressEvent(event);

  if(event->button() == Qt::LeftButton || event->button() == Qt::MidButton)
  {
    bool active;
    {
      SYNC_WITH(imageView.console);
      active = imageView.console.colorTableHandler.active;
    }
    if(!active)
    {
      //QApplication::beep();
      return;
    }
    dragStart = event->pos();
    window2viewport(dragStart);
    dragPos = dragStart;
    QWidget::update();
  }
}

void ImageWidget::mouseMoveEvent(QMouseEvent* event)
{
  QWidget::mouseMoveEvent(event);
/*  if(dragStart.x() >= 0)
  {
    dragPos = event->pos();
    window2viewport(dragPos);
    QWidget::update();
    return;
  }*/

  {
    
    
    
    
    
    SYNC_WITH(imageView.console);
    
    //New by Robin but not ready
    //There are Bugs with the undo
    /*
    Image* image = 0;
    QPoint pos(event->pos());
    window2viewport(pos);
    RobotConsole::Images::const_iterator i = imageView.console.images.find("raw image");
    if(i != imageView.console.images.end())
      image = i->second.image;
    imageView.console.colorTableHandler.disableTraining();
    imageView.console.colorTableHandler.undo();
    imageView.console.colorTableHandler.addPixel(*image, Vector2<int>(pos.rx(), pos.ry()));
    imageView.console.colorTableHandler.enableTraining();
    QWidget::update();
    */
    
    
    if(!imageView.console.colorTableHandler.active)
    {
      QPoint pos(event->pos());
      window2viewport(pos);
      const char* text = 0;
      const std::list<std::string>& drawings(imageView.console.imageViews[imageView.name]);
      for(std::list<std::string>::const_iterator i = drawings.begin(), end = drawings.end(); i != end; ++i)
      {
        text = imageView.console.imageDrawings[*i].getTip(pos.rx(), pos.ry());
        if(text)
          break;
      }
      
      
      
      
      
      
      if(text)
        setToolTip(QString(text));
      else
      {
        Image* image = 0;
        RobotConsole::Images::const_iterator i = imageView.console.images.find(imageView.background);
        if(i != imageView.console.images.end())
          image = i->second.image;
        if(image && pos.rx() >= 0 && pos.ry() >= 0 && pos.rx() < image->resolutionWidth && pos.ry() < image->resolutionHeight)
        {
          Image::Pixel& pixel = image->image[pos.ry()][pos.rx()];
          char color[128];

          static const int factor1 = 29016;
          static const int factor2 = 5662;
          static const int factor3 = 22972;
          static const int factor4 = 11706;

          int yImage = int(pixel.y) << 14;
          int uImage = int(pixel.cr) - 128;
          int vImage = int(pixel.cb) - 128;

          int r = (yImage + factor3 * uImage) >> 14;
          int g = (yImage - factor2 * vImage - factor4 * uImage) >> 14;
          int b = (yImage + factor1 * vImage) >> 14;

          sprintf(color, "x=%d, y=%d\ny=%d, cb=%d, cr=%d\nr=%d, g=%d, b=%d", pos.rx(), pos.ry(), pixel.y, pixel.cb, pixel.cr, r, g, b);
          setToolTip(QString(color));
        }
        else
          setToolTip(QString());
      }
    }
    else
      setToolTip(QString());
  }
}


void ImageWidget::mouseReleaseEvent(QMouseEvent* event)
{
  QWidget::mouseReleaseEvent(event);
  QPoint posInViewPort = QPoint(event->pos());
  window2viewport(posInViewPort);
  Vector2<int> v = Vector2<int>(posInViewPort.x(), posInViewPort.y());
  ImageViewAdapter::fireClick(imageView.name, v);

  if(dragStart.x() < 0)
    return;

  Qt::KeyboardModifiers keys = QApplication::keyboardModifiers();
  {
    SYNC_WITH(imageView.console);

    Image* image = 0;
    RobotConsole::Images::const_iterator i = imageView.console.images.find("raw image");
    if(i != imageView.console.images.end())
      image = i->second.image;

    if(image)
    {
      if(keys & Qt::ControlModifier && !(keys & Qt::ShiftModifier))
        imageView.console.colorTableHandler.undo();
      else
      {
        if(dragStart == dragPos) // just a click
        {
          if((keys & Qt::ShiftModifier) && (keys & Qt::ControlModifier))
            imageView.console.colorTableHandler.clearPixel(*image, Vector2<int>(dragStart.x(), dragStart.y()));
          else if(keys & Qt::ShiftModifier)
            imageView.console.colorTableHandler.selectedColorClass =
              imageView.console.colorTableHandler.colorTable.getColorClass(image->image[dragStart.y()][dragStart.x()].y,
                  image->image[dragStart.y()][dragStart.x()].cb,
                  image->image[dragStart.y()][dragStart.x()].cr);
          else
          {
            if(imageView.console.colorTableHandler.autoct)
              imageView.console.colorTableHandler.autoCT(*image);
            else
              imageView.console.colorTableHandler.addPixel(*image, Vector2<int>(dragStart.x(), dragStart.y()));
          }
        }
      /*  else if((keys & Qt::ShiftModifier) && (keys & Qt::ControlModifier))
          imageView.console.colorTableHandler.clearRectangle(*image, Vector2<int>(std::min<>(dragStart.x(), dragPos.x()), std::min<>(dragStart.y(), dragPos.y())),
              Vector2<int>(std::max<>(dragStart.x(), dragPos.x()), std::max<>(dragStart.y(), dragPos.y())));
        else if(keys & Qt::ShiftModifier)
          imageView.console.colorTableHandler.fillRectangle(*image, Vector2<int>(std::min<>(dragStart.x(), dragPos.x()), std::min<>(dragStart.y(), dragPos.y())),
              Vector2<int>(std::max<>(dragStart.x(), dragPos.x()), std::max<>(dragStart.y(), dragPos.y())));
        else if(!(keys & Qt::ShiftModifier))
          imageView.console.colorTableHandler.addRectangle(*image, Vector2<int>(std::min<>(dragStart.x(), dragPos.x()), std::min<>(dragStart.y(), dragPos.y())),
              Vector2<int>(std::max<>(dragStart.x(), dragPos.x()), std::max<>(dragStart.y(), dragPos.y())));*/
      }
    }
  }

  dragStart = QPoint(-1, -1);
  QWidget::update();
}

void ImageWidget::keyPressEvent(QKeyEvent* event)
{
  switch(event->key())
  {
  case Qt::Key_PageUp:
  case Qt::Key_Plus:
    event->accept();
    if(zoom < 3.f)
      zoom += 0.1f;
    if(zoom > 3.f)
      zoom = 3.f;
    QWidget::update();
    break;
  case Qt::Key_PageDown:
  case Qt::Key_Minus:
    event->accept();
    if(zoom > 0.1f)
      zoom -= 0.1f;
    QWidget::update();
    break;
  case Qt::Key_Up:
    offset += QPoint(0, 20);
    QWidget::update();
    break;
  case Qt::Key_Down:
    offset += QPoint(0, -20);
    QWidget::update();
    break;
  case Qt::Key_Left:
    offset += QPoint(20, 0);
    QWidget::update();
    break;
  case Qt::Key_Right:
    offset += QPoint(-20, 0);
    QWidget::update();
    break;
  default:
    QWidget::keyPressEvent(event);
    break;
  }
}

void ImageWidget::wheelEvent(QWheelEvent* event)
{
  QWidget::wheelEvent(event);

  zoom += 0.1 * event->delta() / 120;
  if(zoom > 3.f)
    zoom = 3.f;
  else if(zoom < 0.1f)
    zoom = 0.1f;
  QWidget::update();
}

void ImageWidget::mouseDoubleClickEvent(QMouseEvent* event)
{
  zoom = 1.f;
  offset.setX(0);
  offset.setY(0);
  QWidget::update();
}

QMenu* ImageWidget::createUserMenu()
{

  QMenu* menu = new QMenu(tr("&Image"));
  QAction* clearAct      = new QAction(QIcon(":/Icons/delete.png"), tr("&Clear"), menu);
  QAction* undoAct       = new QAction(QIcon(":/Icons/arrow_undo.png"), tr("&Undo"), menu);
  QAction* reclassifyAct = new QAction(QIcon(":/Icons/arrow_refresh.png"), tr("Rec&lassify"), menu);

  QAction* noneAct       = new QAction(QIcon(":/Icons/none.png"), tr("&None"), menu);
  QAction* whiteAct      = new QAction(QIcon(":/Icons/white.png"), tr("&White"), menu);
  QAction* yellowAct     = new QAction(QIcon(":/Icons/yellow.png"), tr("&Yellow"), menu);
  QAction* orangeAct     = new QAction(QIcon(":/Icons/orange.png"), tr("&Orange"), menu);

  QAction* redAct        = new QAction(QIcon(":/Icons/red.png"), tr("&Red"), menu);
  QAction* greenAct      = new QAction(QIcon(":/Icons/green.png"), tr("&Green"), menu);
  QAction* robotBlueAct  = new QAction(QIcon(":/Icons/robotBlue.png"), tr("Robo&t Blue"), menu);
  QAction* blueAct       = new QAction(QIcon(":/Icons/blue.png"), tr("&Blue"), menu);
  
  QAction* waistbandsAct = new QAction(QIcon(":/Icons/waistbands.png"), tr("&Waistband Scanlines On"), menu);
  QAction* headAngleAct  = new QAction(QIcon(":/Icons/headAngle.png"), tr("&HeadAngleRequest Free Head"), menu);
  QAction* camUpAct      = new QAction(QIcon(":/Icons/camUp.png"), tr("&Upper Cam View"), menu);
  QAction* camDownAct    = new QAction(QIcon(":/Icons/camDown.png"), tr("&Down Cam View"), menu);

  connect(clearAct     , SIGNAL(triggered()), this, SLOT(kdClear()));
  connect(undoAct      , SIGNAL(triggered()), this, SLOT(kdUndo()));
  connect(reclassifyAct, SIGNAL(triggered()), this, SLOT(kdReclassify()));

  connect(noneAct      , SIGNAL(triggered()), this, SLOT(kdSelectNone()));
  connect(whiteAct     , SIGNAL(triggered()), this, SLOT(kdSelectWhite()));
  connect(yellowAct    , SIGNAL(triggered()), this, SLOT(kdSelectYellow()));
  connect(orangeAct    , SIGNAL(triggered()), this, SLOT(kdSelectOrange()));

  connect(redAct       , SIGNAL(triggered()), this, SLOT(kdSelectRed()));
  connect(greenAct     , SIGNAL(triggered()), this, SLOT(kdSelectGreen()));
  connect(robotBlueAct , SIGNAL(triggered()), this, SLOT(kdSelectRobotBlue()));
  connect(blueAct      , SIGNAL(triggered()), this, SLOT(kdSelectBlue()));
  
  connect(waistbandsAct, SIGNAL(triggered()), this, SLOT(waistbandsOn()));
  connect(headAngleAct , SIGNAL(triggered()), this, SLOT(headAngle()));
  connect(camUpAct     , SIGNAL(triggered()), this, SLOT(camUpOn()));
  connect(camDownAct   , SIGNAL(triggered()), this, SLOT(camDownOn()));
  
  
  menu->addAction(clearAct);
  menu->addAction(undoAct);
  menu->addAction(reclassifyAct);

  menu->addAction(noneAct);
  menu->addAction(whiteAct);
  menu->addAction(yellowAct);
  menu->addAction(orangeAct);

  menu->addAction(redAct);
  menu->addAction(greenAct);
  menu->addAction(robotBlueAct);
  menu->addAction(blueAct);

  menu->addAction(waistbandsAct);
  menu->addAction(headAngleAct);
  menu->addAction(camUpAct);
  menu->addAction(camDownAct);
  
  return menu;
}

ImageView::ImageView(const QString& fullName, RobotConsole& console, const std::string& background, const std::string& name, bool segmented, float gain) :
  fullName(fullName), icon(":/Icons/tag_green.png"), console(console),
  background(background), name(name),
  segmented(segmented),
  gain(gain) {}

SimRobot::Widget* ImageView::createWidget()
{
  return new ImageWidget(*this);
}
