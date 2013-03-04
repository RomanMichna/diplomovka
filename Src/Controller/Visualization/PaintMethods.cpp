/**
* @file Controller/Visualization/PaintMethods.cpp
* Implementation of class PaintMethods.
*
* @author <A href="mailto:juengel@informatik.hu-berlin.de">Matthias J�ngel</A>
* @author Colin Graf
*/

#include <QPainter>

#include "PaintMethods.h"
#include "DebugDrawing.h"

void PaintMethods::paintDebugDrawing(QPainter& painter, const DebugDrawing& debugDrawing, const QTransform& baseTrans)
{
  static QBrush brush(Qt::SolidPattern);
  static QBrush noBrush(Qt::NoBrush);
  static QPen pen;
  static QPen noPen(Qt::NoPen);

  for(const DebugDrawing::Element* e = debugDrawing.getFirst(); e; e = debugDrawing.getNext(e))
    switch(e->type)
    {
    case DebugDrawing::Element::POLYGON:
    {
      const DebugDrawing::Polygon& element = *(const DebugDrawing::Polygon*) e;

      // select brush
      if(element.fillStyle == Drawings::bs_solid)
      {
        brush.setColor(QColor(element.fillColor.r, element.fillColor.g, element.fillColor.b, element.fillColor.a));
        painter.setBrush(brush);
      }
      else
        painter.setBrush(noBrush);

      // select pen
      if(element.penStyle != Drawings::ps_null)
      {
        pen.setColor(QColor(element.penColor.r, element.penColor.g, element.penColor.b, element.penColor.a));
        // A line width of zero indicates a cosmetic pen. This means that the pen width is always drawn one pixel wide, independent of the transformation set on the painter.
        pen.setWidth(element.width);
        switch(element.penStyle)
        {
        case Drawings::ps_dash:
          pen.setStyle(Qt::DashLine);
          break;
        case Drawings::ps_dot:
          pen.setStyle(Qt::DotLine);
          break;
        case Drawings::ps_solid:
        default:
          pen.setStyle(Qt::SolidLine);
        }
        painter.setPen(pen);
      }
      else
        painter.setPen(noPen);

      // copy vector2 to QPoints
      static QPoint points[16];
      for(int n = element.nCount - 1; n >= 0 ; --n)
        points[n] = QPoint(element.points[n].x, element.points[n].y);

      painter.drawPolygon(points, element.nCount);
      break;
    }
    case DebugDrawing::Element::GRID_RGBA:
    {
      const DebugDrawing::GridRGBA& element = *(const DebugDrawing::GridRGBA*) e;
      QImage image(element.cellsX, element.cellsY, QImage::Format_ARGB32);

      unsigned* pDst = (unsigned*) image.bits();
      unsigned* pEnd = pDst + element.cellsX * element.cellsY;
      const ColorRGBA* pSrc = (const ColorRGBA*) (&element + 1);
      while(pDst != pEnd)
      {
        *pDst++ = pSrc->a << 24 | pSrc->r << 16 | pSrc->g << 8 | pSrc->b;
        ++pSrc;
      }

      float totalWidth = float(element.cellsX * element.cellSize);
      float totalHeight = float(element.cellsY * element.cellSize);
      painter.drawImage(QRectF(-totalWidth / 2.f, -totalHeight / 2.f, totalWidth, totalHeight), image);
      break;
    }
    case DebugDrawing::Element::GRID_MONO:
    {
      const DebugDrawing::GridMono& element = *(const DebugDrawing::GridMono*) e;
      QImage image(element.cellsX, element.cellsY, QImage::Format_ARGB32);

      unsigned colors[256];
      for(int i = 0; i < 256; ++i)
      {
        ColorRGBA col(element.baseColor * (1.0f - (static_cast<float>(i) / 255.0f)));
        colors[i] = element.baseColor.a << 24 | col.r << 16 | col.g << 8 | col.b;
      }
      
      unsigned* pDst = (unsigned*) image.bits();
      unsigned* pEnd = pDst + element.cellsX * element.cellsY;
      const unsigned char* pSrc = (const unsigned char*) (&element + 1);
      while(pDst != pEnd)
        *pDst++ = colors[*pSrc++];

      float totalWidth = float(element.cellsX * element.cellSize);
      float totalHeight = float(element.cellsY * element.cellSize);
      painter.drawImage(QRectF(-totalWidth / 2.f, -totalHeight / 2.f, totalWidth, totalHeight), image);
      break;
    }
    case DebugDrawing::Element::ELLIPSE:
    {
      const DebugDrawing::Ellipse& element = *(const DebugDrawing::Ellipse*) e;

      // select brush
      if(element.fillStyle == Drawings::bs_solid)
      {
        brush.setColor(QColor(element.fillColor.r, element.fillColor.g, element.fillColor.b, element.fillColor.a));
        painter.setBrush(brush);
      }
      else
        painter.setBrush(noBrush);

      // select pen
      if(element.penStyle != Drawings::ps_null)
      {
        pen.setColor(QColor(element.penColor.r, element.penColor.g, element.penColor.b, element.penColor.a));
        // A line width of zero indicates a cosmetic pen. This means that the pen width is always drawn one pixel wide, independent of the transformation set on the painter.
        pen.setWidth(element.width);
        switch(element.penStyle)
        {
        case Drawings::ps_dash:
          pen.setStyle(Qt::DashLine);
          break;
        case Drawings::ps_dot:
          pen.setStyle(Qt::DotLine);
          break;
        case Drawings::ps_solid:
        default:
          pen.setStyle(Qt::SolidLine);
        }
        painter.setPen(pen);
      }
      else
        painter.setPen(noPen);

      if(element.rotation != 0.0f)
      {
        QTransform trans(painter.transform());
        QTransform transBack(painter.transform());
        trans.translate(qreal(element.x), qreal(element.y));
        trans.rotateRadians(qreal(element.rotation));
        painter.setTransform(trans);
        painter.drawEllipse(-element.radiusX, -element.radiusY, 2 * element.radiusX, 2 * element.radiusY);
        painter.setTransform(transBack);
      }
      else
      {
        painter.drawEllipse(element.x - element.radiusX, element.y - element.radiusY, 2 * element.radiusX, 2 * element.radiusY);
      }
      break;
    }
    case DebugDrawing::Element::RECTANGLE:
    {
      const DebugDrawing::Rectangle& element = *(const DebugDrawing::Rectangle*) e;

      // select brush
      if(element.fillStyle == Drawings::bs_solid)
      {
        brush.setColor(QColor(element.fillColor.r, element.fillColor.g, element.fillColor.b, element.fillColor.a));
        painter.setBrush(brush);
      }
      else
        painter.setBrush(noBrush);

      // select pen
      if(element.penStyle != Drawings::ps_null)
      {
        pen.setColor(QColor(element.penColor.r, element.penColor.g, element.penColor.b, element.penColor.a));
        // A line width of zero indicates a cosmetic pen. This means that the pen width is always drawn one pixel wide, independent of the transformation set on the painter.
        pen.setWidth(element.width);
        switch(element.penStyle)
        {
        case Drawings::ps_dash:
          pen.setStyle(Qt::DashLine);
          break;
        case Drawings::ps_dot:
          pen.setStyle(Qt::DotLine);
          break;
        case Drawings::ps_solid:
        default:
          pen.setStyle(Qt::SolidLine);
        }
        painter.setPen(pen);
      }
      else
        painter.setPen(noPen);

      QRect dRect(element.topLX, element.topLY, element.w, element.h);

      if(element.rotation != 0.0f)
      {
        QPoint center = dRect.center();
        QTransform trans(painter.transform());
        QTransform transBack(painter.transform());
        trans.translate(center.x(), center.y());
        trans.rotateRadians(qreal(element.rotation));
        painter.setTransform(trans);
        painter.drawRect(QRect(element.topLX - center.x(), element.topLY - center.y(), dRect.width(), dRect.height()));
        painter.setTransform(transBack);
      }
      else
      {
        painter.drawRect(dRect);
      }

      break;
    }
    case DebugDrawing::Element::LINE:
    {
      const DebugDrawing::Line& element = *(const DebugDrawing::Line*) e;

      if(element.penStyle != Drawings::ps_null)
      {
        pen.setColor(QColor(element.penColor.r, element.penColor.g, element.penColor.b, element.penColor.a));
        // A line width of zero indicates a cosmetic pen. This means that the pen width is always drawn one pixel wide, independent of the transformation set on the painter.
        pen.setWidth(element.width);
        switch(element.penStyle)
        {
        case Drawings::ps_dash:
          pen.setStyle(Qt::DashLine);
          break;
        case Drawings::ps_dot:
          pen.setStyle(Qt::DotLine);
          break;
        case Drawings::ps_solid:
        default:
          pen.setStyle(Qt::SolidLine);
        }
        painter.setPen(pen);

        painter.drawLine(element.xStart, element.yStart, element.xEnd, element.yEnd);
      }
      break;
    }
    case DebugDrawing::Element::ORIGIN:
    {
      const DebugDrawing::Origin& element = *(const DebugDrawing::Origin*) e;
      QTransform trans(baseTrans);
      trans.translate(qreal(element.x), qreal(element.y));
      trans.rotateRadians(qreal(element.angle));
      painter.setTransform(trans);
      break;
    }

    case DebugDrawing::Element::TEXT:
    {
      const DebugDrawing::Text& element = *(const DebugDrawing::Text*) e;

      pen.setColor(QColor(element.penColor.r, element.penColor.g, element.penColor.b, element.penColor.a));
      painter.setPen(pen);

      QTransform trans(painter.transform());
      const QPoint& pos(trans.map(QPoint(element.x, element.y)));
      painter.resetTransform();
      painter.drawText(pos, (const char*)(&element + 1));
      painter.setTransform(trans);

      break;
    }
    default:
      break;
    }
}
