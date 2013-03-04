/**
* @file Controller/Views/ViewBike/ViewBikeGLWidget.cpp
*
* Implementation of class ViewBikeGLWidget.
*
* @author <a href="mailto:judy@tzi.de">Judith Müller</a>
*/

#include "Platform/OpenGL.h"
#include "ViewBikeWidget.h"
#include "Tools/Streams/InStreams.h"
#include "Platform/BHAssert.h"
#include "Tools/Math/Vector2.h"

#include "Controller/RobotConsole.h"
#include "ViewBike.h"
#include "ViewBikeMath.h"

#include <vector>
#include <sstream>

#include <QMouseEvent>
#include <QApplication>
#include <QKeyEvent>
#include <QContextMenuEvent>

ViewBikeGLWidget::ViewBikeGLWidget(ViewBike& viewBike, BIKEParameters& parameters, ViewBikeWidget* parent) :
  QGLWidget(parent),
  widget(*parent),
  viewBike(viewBike),
  renderer(*viewBike.robot->createRenderer()),
  parameters(parameters),
  moveDrag(false),
  moveViewOfViewDragXY(false),
  moveViewOfViewDragXZ(false),
  moveViewOfViewDragYZ(false),
  moveViewOfViewDragXP(false),
  moveViewOfViewDragYP(false),
  moveViewOfViewDragZP(false)
{
  GLubyte test[17][128] =
  {
    {
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    },

    {
      0x88, 0x88, 0x88, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x88, 0x88, 0x88, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x88, 0x88, 0x88, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x88, 0x88, 0x88, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x88, 0x88, 0x88, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x88, 0x88, 0x88, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x88, 0x88, 0x88, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x88, 0x88, 0x88, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    },

    {
      0x88, 0x88, 0x88, 0x88, 0x00, 0x00, 0x00, 0x00, 0x22, 0x22, 0x22, 0x22, 0x00, 0x00, 0x00, 0x00,
      0x88, 0x88, 0x88, 0x88, 0x00, 0x00, 0x00, 0x00, 0x22, 0x22, 0x22, 0x22, 0x00, 0x00, 0x00, 0x00,
      0x88, 0x88, 0x88, 0x88, 0x00, 0x00, 0x00, 0x00, 0x22, 0x22, 0x22, 0x22, 0x00, 0x00, 0x00, 0x00,
      0x88, 0x88, 0x88, 0x88, 0x00, 0x00, 0x00, 0x00, 0x22, 0x22, 0x22, 0x22, 0x00, 0x00, 0x00, 0x00,
      0x88, 0x88, 0x88, 0x88, 0x00, 0x00, 0x00, 0x00, 0x22, 0x22, 0x22, 0x22, 0x00, 0x00, 0x00, 0x00,
      0x88, 0x88, 0x88, 0x88, 0x00, 0x00, 0x00, 0x00, 0x22, 0x22, 0x22, 0x22, 0x00, 0x00, 0x00, 0x00,
      0x88, 0x88, 0x88, 0x88, 0x00, 0x00, 0x00, 0x00, 0x22, 0x22, 0x22, 0x22, 0x00, 0x00, 0x00, 0x00,
      0x88, 0x88, 0x88, 0x88, 0x00, 0x00, 0x00, 0x00, 0x22, 0x22, 0x22, 0x22, 0x00, 0x00, 0x00, 0x00
    },

    {
      0xaa, 0xaa, 0xaa, 0xaa, 0x00, 0x00, 0x00, 0x00, 0x22, 0x22, 0x22, 0x22, 0x00, 0x00, 0x00, 0x00,
      0xaa, 0xaa, 0xaa, 0xaa, 0x00, 0x00, 0x00, 0x00, 0x22, 0x22, 0x22, 0x22, 0x00, 0x00, 0x00, 0x00,
      0xaa, 0xaa, 0xaa, 0xaa, 0x00, 0x00, 0x00, 0x00, 0x22, 0x22, 0x22, 0x22, 0x00, 0x00, 0x00, 0x00,
      0xaa, 0xaa, 0xaa, 0xaa, 0x00, 0x00, 0x00, 0x00, 0x22, 0x22, 0x22, 0x22, 0x00, 0x00, 0x00, 0x00,
      0xaa, 0xaa, 0xaa, 0xaa, 0x00, 0x00, 0x00, 0x00, 0x22, 0x22, 0x22, 0x22, 0x00, 0x00, 0x00, 0x00,
      0xaa, 0xaa, 0xaa, 0xaa, 0x00, 0x00, 0x00, 0x00, 0x22, 0x22, 0x22, 0x22, 0x00, 0x00, 0x00, 0x00,
      0xaa, 0xaa, 0xaa, 0xaa, 0x00, 0x00, 0x00, 0x00, 0x22, 0x22, 0x22, 0x22, 0x00, 0x00, 0x00, 0x00,
      0xaa, 0xaa, 0xaa, 0xaa, 0x00, 0x00, 0x00, 0x00, 0x22, 0x22, 0x22, 0x22, 0x00, 0x00, 0x00, 0x00
    },

    {
      0xaa, 0xaa, 0xaa, 0xaa, 0x00, 0x00, 0x00, 0x00, 0xaa, 0xaa, 0xaa, 0xaa, 0x00, 0x00, 0x00, 0x00,
      0xaa, 0xaa, 0xaa, 0xaa, 0x00, 0x00, 0x00, 0x00, 0xaa, 0xaa, 0xaa, 0xaa, 0x00, 0x00, 0x00, 0x00,
      0xaa, 0xaa, 0xaa, 0xaa, 0x00, 0x00, 0x00, 0x00, 0xaa, 0xaa, 0xaa, 0xaa, 0x00, 0x00, 0x00, 0x00,
      0xaa, 0xaa, 0xaa, 0xaa, 0x00, 0x00, 0x00, 0x00, 0xaa, 0xaa, 0xaa, 0xaa, 0x00, 0x00, 0x00, 0x00,
      0xaa, 0xaa, 0xaa, 0xaa, 0x00, 0x00, 0x00, 0x00, 0xaa, 0xaa, 0xaa, 0xaa, 0x00, 0x00, 0x00, 0x00,
      0xaa, 0xaa, 0xaa, 0xaa, 0x00, 0x00, 0x00, 0x00, 0xaa, 0xaa, 0xaa, 0xaa, 0x00, 0x00, 0x00, 0x00,
      0xaa, 0xaa, 0xaa, 0xaa, 0x00, 0x00, 0x00, 0x00, 0xaa, 0xaa, 0xaa, 0xaa, 0x00, 0x00, 0x00, 0x00,
      0xaa, 0xaa, 0xaa, 0xaa, 0x00, 0x00, 0x00, 0x00, 0xaa, 0xaa, 0xaa, 0xaa, 0x00, 0x00, 0x00, 0x00
    },

    {
      0xaa, 0xaa, 0xaa, 0xaa, 0x44, 0x44, 0x44, 0x44, 0xaa, 0xaa, 0xaa, 0xaa, 0x00, 0x00, 0x00, 0x00,
      0xaa, 0xaa, 0xaa, 0xaa, 0x44, 0x44, 0x44, 0x44, 0xaa, 0xaa, 0xaa, 0xaa, 0x00, 0x00, 0x00, 0x00,
      0xaa, 0xaa, 0xaa, 0xaa, 0x44, 0x44, 0x44, 0x44, 0xaa, 0xaa, 0xaa, 0xaa, 0x00, 0x00, 0x00, 0x00,
      0xaa, 0xaa, 0xaa, 0xaa, 0x44, 0x44, 0x44, 0x44, 0xaa, 0xaa, 0xaa, 0xaa, 0x00, 0x00, 0x00, 0x00,
      0xaa, 0xaa, 0xaa, 0xaa, 0x44, 0x44, 0x44, 0x44, 0xaa, 0xaa, 0xaa, 0xaa, 0x00, 0x00, 0x00, 0x00,
      0xaa, 0xaa, 0xaa, 0xaa, 0x44, 0x44, 0x44, 0x44, 0xaa, 0xaa, 0xaa, 0xaa, 0x00, 0x00, 0x00, 0x00,
      0xaa, 0xaa, 0xaa, 0xaa, 0x44, 0x44, 0x44, 0x44, 0xaa, 0xaa, 0xaa, 0xaa, 0x00, 0x00, 0x00, 0x00,
      0xaa, 0xaa, 0xaa, 0xaa, 0x44, 0x44, 0x44, 0x44, 0xaa, 0xaa, 0xaa, 0xaa, 0x00, 0x00, 0x00, 0x00
    },

    {
      0xaa, 0xaa, 0xaa, 0xaa, 0x44, 0x44, 0x44, 0x44, 0xaa, 0xaa, 0xaa, 0xaa, 0x11, 0x11, 0x11, 0x11,
      0xaa, 0xaa, 0xaa, 0xaa, 0x44, 0x44, 0x44, 0x44, 0xaa, 0xaa, 0xaa, 0xaa, 0x11, 0x11, 0x11, 0x11,
      0xaa, 0xaa, 0xaa, 0xaa, 0x44, 0x44, 0x44, 0x44, 0xaa, 0xaa, 0xaa, 0xaa, 0x11, 0x11, 0x11, 0x11,
      0xaa, 0xaa, 0xaa, 0xaa, 0x44, 0x44, 0x44, 0x44, 0xaa, 0xaa, 0xaa, 0xaa, 0x11, 0x11, 0x11, 0x11,
      0xaa, 0xaa, 0xaa, 0xaa, 0x44, 0x44, 0x44, 0x44, 0xaa, 0xaa, 0xaa, 0xaa, 0x11, 0x11, 0x11, 0x11,
      0xaa, 0xaa, 0xaa, 0xaa, 0x44, 0x44, 0x44, 0x44, 0xaa, 0xaa, 0xaa, 0xaa, 0x11, 0x11, 0x11, 0x11,
      0xaa, 0xaa, 0xaa, 0xaa, 0x44, 0x44, 0x44, 0x44, 0xaa, 0xaa, 0xaa, 0xaa, 0x11, 0x11, 0x11, 0x11,
      0xaa, 0xaa, 0xaa, 0xaa, 0x44, 0x44, 0x44, 0x44, 0xaa, 0xaa, 0xaa, 0xaa, 0x11, 0x11, 0x11, 0x11
    },

    {
      0xaa, 0xaa, 0xaa, 0xaa, 0x55, 0x55, 0x55, 0x55, 0xaa, 0xaa, 0xaa, 0xaa, 0x11, 0x11, 0x11, 0x11,
      0xaa, 0xaa, 0xaa, 0xaa, 0x55, 0x55, 0x55, 0x55, 0xaa, 0xaa, 0xaa, 0xaa, 0x11, 0x11, 0x11, 0x11,
      0xaa, 0xaa, 0xaa, 0xaa, 0x55, 0x55, 0x55, 0x55, 0xaa, 0xaa, 0xaa, 0xaa, 0x11, 0x11, 0x11, 0x11,
      0xaa, 0xaa, 0xaa, 0xaa, 0x55, 0x55, 0x55, 0x55, 0xaa, 0xaa, 0xaa, 0xaa, 0x11, 0x11, 0x11, 0x11,
      0xaa, 0xaa, 0xaa, 0xaa, 0x55, 0x55, 0x55, 0x55, 0xaa, 0xaa, 0xaa, 0xaa, 0x11, 0x11, 0x11, 0x11,
      0xaa, 0xaa, 0xaa, 0xaa, 0x55, 0x55, 0x55, 0x55, 0xaa, 0xaa, 0xaa, 0xaa, 0x11, 0x11, 0x11, 0x11,
      0xaa, 0xaa, 0xaa, 0xaa, 0x55, 0x55, 0x55, 0x55, 0xaa, 0xaa, 0xaa, 0xaa, 0x11, 0x11, 0x11, 0x11,
      0xaa, 0xaa, 0xaa, 0xaa, 0x55, 0x55, 0x55, 0x55, 0xaa, 0xaa, 0xaa, 0xaa, 0x11, 0x11, 0x11, 0x11
    },

    {
      0xaa, 0xaa, 0xaa, 0xaa, 0x55, 0x55, 0x55, 0x55, 0xaa, 0xaa, 0xaa, 0xaa, 0x55, 0x55, 0x55, 0x55,
      0xaa, 0xaa, 0xaa, 0xaa, 0x55, 0x55, 0x55, 0x55, 0xaa, 0xaa, 0xaa, 0xaa, 0x55, 0x55, 0x55, 0x55,
      0xaa, 0xaa, 0xaa, 0xaa, 0x55, 0x55, 0x55, 0x55, 0xaa, 0xaa, 0xaa, 0xaa, 0x55, 0x55, 0x55, 0x55,
      0xaa, 0xaa, 0xaa, 0xaa, 0x55, 0x55, 0x55, 0x55, 0xaa, 0xaa, 0xaa, 0xaa, 0x55, 0x55, 0x55, 0x55,
      0xaa, 0xaa, 0xaa, 0xaa, 0x55, 0x55, 0x55, 0x55, 0xaa, 0xaa, 0xaa, 0xaa, 0x55, 0x55, 0x55, 0x55,
      0xaa, 0xaa, 0xaa, 0xaa, 0x55, 0x55, 0x55, 0x55, 0xaa, 0xaa, 0xaa, 0xaa, 0x55, 0x55, 0x55, 0x55,
      0xaa, 0xaa, 0xaa, 0xaa, 0x55, 0x55, 0x55, 0x55, 0xaa, 0xaa, 0xaa, 0xaa, 0x55, 0x55, 0x55, 0x55,
      0xaa, 0xaa, 0xaa, 0xaa, 0x55, 0x55, 0x55, 0x55, 0xaa, 0xaa, 0xaa, 0xaa, 0x55, 0x55, 0x55, 0x55
    },

    {
      0xee, 0xee, 0xee, 0xee, 0x55, 0x55, 0x55, 0x55, 0xaa, 0xaa, 0xaa, 0xaa, 0x55, 0x55, 0x55, 0x55,
      0xee, 0xee, 0xee, 0xee, 0x55, 0x55, 0x55, 0x55, 0xaa, 0xaa, 0xaa, 0xaa, 0x55, 0x55, 0x55, 0x55,
      0xee, 0xee, 0xee, 0xee, 0x55, 0x55, 0x55, 0x55, 0xaa, 0xaa, 0xaa, 0xaa, 0x55, 0x55, 0x55, 0x55,
      0xee, 0xee, 0xee, 0xee, 0x55, 0x55, 0x55, 0x55, 0xaa, 0xaa, 0xaa, 0xaa, 0x55, 0x55, 0x55, 0x55,
      0xee, 0xee, 0xee, 0xee, 0x55, 0x55, 0x55, 0x55, 0xaa, 0xaa, 0xaa, 0xaa, 0x55, 0x55, 0x55, 0x55,
      0xee, 0xee, 0xee, 0xee, 0x55, 0x55, 0x55, 0x55, 0xaa, 0xaa, 0xaa, 0xaa, 0x55, 0x55, 0x55, 0x55,
      0xee, 0xee, 0xee, 0xee, 0x55, 0x55, 0x55, 0x55, 0xaa, 0xaa, 0xaa, 0xaa, 0x55, 0x55, 0x55, 0x55,
      0xee, 0xee, 0xee, 0xee, 0x55, 0x55, 0x55, 0x55, 0xaa, 0xaa, 0xaa, 0xaa, 0x55, 0x55, 0x55, 0x55
    },

    {
      0xee, 0xee, 0xee, 0xee, 0x55, 0x55, 0x55, 0x55, 0xbb, 0xbb, 0xbb, 0xbb, 0x55, 0x55, 0x55, 0x55,
      0xee, 0xee, 0xee, 0xee, 0x55, 0x55, 0x55, 0x55, 0xbb, 0xbb, 0xbb, 0xbb, 0x55, 0x55, 0x55, 0x55,
      0xee, 0xee, 0xee, 0xee, 0x55, 0x55, 0x55, 0x55, 0xbb, 0xbb, 0xbb, 0xbb, 0x55, 0x55, 0x55, 0x55,
      0xee, 0xee, 0xee, 0xee, 0x55, 0x55, 0x55, 0x55, 0xbb, 0xbb, 0xbb, 0xbb, 0x55, 0x55, 0x55, 0x55,
      0xee, 0xee, 0xee, 0xee, 0x55, 0x55, 0x55, 0x55, 0xbb, 0xbb, 0xbb, 0xbb, 0x55, 0x55, 0x55, 0x55,
      0xee, 0xee, 0xee, 0xee, 0x55, 0x55, 0x55, 0x55, 0xbb, 0xbb, 0xbb, 0xbb, 0x55, 0x55, 0x55, 0x55,
      0xee, 0xee, 0xee, 0xee, 0x55, 0x55, 0x55, 0x55, 0xbb, 0xbb, 0xbb, 0xbb, 0x55, 0x55, 0x55, 0x55,
      0xee, 0xee, 0xee, 0xee, 0x55, 0x55, 0x55, 0x55, 0xbb, 0xbb, 0xbb, 0xbb, 0x55, 0x55, 0x55, 0x55
    },

    {
      0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55, 0xbb, 0xbb, 0xbb, 0xbb, 0x55, 0x55, 0x55, 0x55,
      0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55, 0xbb, 0xbb, 0xbb, 0xbb, 0x55, 0x55, 0x55, 0x55,
      0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55, 0xbb, 0xbb, 0xbb, 0xbb, 0x55, 0x55, 0x55, 0x55,
      0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55, 0xbb, 0xbb, 0xbb, 0xbb, 0x55, 0x55, 0x55, 0x55,
      0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55, 0xbb, 0xbb, 0xbb, 0xbb, 0x55, 0x55, 0x55, 0x55,
      0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55, 0xbb, 0xbb, 0xbb, 0xbb, 0x55, 0x55, 0x55, 0x55,
      0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55, 0xbb, 0xbb, 0xbb, 0xbb, 0x55, 0x55, 0x55, 0x55,
      0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55, 0xbb, 0xbb, 0xbb, 0xbb, 0x55, 0x55, 0x55, 0x55
    },

    {
      0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55, 0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55,
      0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55, 0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55,
      0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55, 0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55,
      0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55, 0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55,
      0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55, 0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55,
      0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55, 0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55,
      0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55, 0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55,
      0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55, 0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55
    },

    {
      0xff, 0xff, 0xff, 0xff, 0xdd, 0xdd, 0xdd, 0xdd, 0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55,
      0xff, 0xff, 0xff, 0xff, 0xdd, 0xdd, 0xdd, 0xdd, 0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55,
      0xff, 0xff, 0xff, 0xff, 0xdd, 0xdd, 0xdd, 0xdd, 0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55,
      0xff, 0xff, 0xff, 0xff, 0xdd, 0xdd, 0xdd, 0xdd, 0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55,
      0xff, 0xff, 0xff, 0xff, 0xdd, 0xdd, 0xdd, 0xdd, 0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55,
      0xff, 0xff, 0xff, 0xff, 0xdd, 0xdd, 0xdd, 0xdd, 0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55,
      0xff, 0xff, 0xff, 0xff, 0xdd, 0xdd, 0xdd, 0xdd, 0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55,
      0xff, 0xff, 0xff, 0xff, 0xdd, 0xdd, 0xdd, 0xdd, 0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55
    },

    {
      0xff, 0xff, 0xff, 0xff, 0xdd, 0xdd, 0xdd, 0xdd, 0xff, 0xff, 0xff, 0xff, 0x77, 0x77, 0x77, 0x77,
      0xff, 0xff, 0xff, 0xff, 0xdd, 0xdd, 0xdd, 0xdd, 0xff, 0xff, 0xff, 0xff, 0x77, 0x77, 0x77, 0x77,
      0xff, 0xff, 0xff, 0xff, 0xdd, 0xdd, 0xdd, 0xdd, 0xff, 0xff, 0xff, 0xff, 0x77, 0x77, 0x77, 0x77,
      0xff, 0xff, 0xff, 0xff, 0xdd, 0xdd, 0xdd, 0xdd, 0xff, 0xff, 0xff, 0xff, 0x77, 0x77, 0x77, 0x77,
      0xff, 0xff, 0xff, 0xff, 0xdd, 0xdd, 0xdd, 0xdd, 0xff, 0xff, 0xff, 0xff, 0x77, 0x77, 0x77, 0x77,
      0xff, 0xff, 0xff, 0xff, 0xdd, 0xdd, 0xdd, 0xdd, 0xff, 0xff, 0xff, 0xff, 0x77, 0x77, 0x77, 0x77,
      0xff, 0xff, 0xff, 0xff, 0xdd, 0xdd, 0xdd, 0xdd, 0xff, 0xff, 0xff, 0xff, 0x77, 0x77, 0x77, 0x77,
      0xff, 0xff, 0xff, 0xff, 0xdd, 0xdd, 0xdd, 0xdd, 0xff, 0xff, 0xff, 0xff, 0x77, 0x77, 0x77, 0x77
    },

    {
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x77, 0x77, 0x77, 0x77,
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x77, 0x77, 0x77, 0x77,
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x77, 0x77, 0x77, 0x77,
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x77, 0x77, 0x77, 0x77,
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x77, 0x77, 0x77, 0x77,
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x77, 0x77, 0x77, 0x77,
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x77, 0x77, 0x77, 0x77,
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x77, 0x77, 0x77, 0x77
    },

    {
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
    }
  };
  for(int i = 0; i < 17; i++)
  {
    for(int k = 0; k < 128; k++)
    {
      stippleMask[i][k] = test[i][k];
    }
  }

  setFocusPolicy(Qt::StrongFocus);
}

ViewBikeGLWidget::~ViewBikeGLWidget()
{
  delete &renderer;
}

void ViewBikeGLWidget::initializeGL()
{
  renderer.init(isSharing());
  renderer.setCameraMode(SimRobotCore2::Renderer::targetCam);
  renderer.setSurfaceShadeMode(SimRobotCore2::Renderer::smoothShading);
  renderer.setRenderFlags((renderer.getRenderFlags() & ~SimRobotCore2::Renderer::showCoordinateSystem) | SimRobotCore2::Renderer::showAsGlobalView);
  renderer.resetCamera();
  Vector3<> cameraPos(0, -1.f, 0.f),
          targetPos;
  renderer.setCamera(&cameraPos.x, &targetPos.x);
}

void ViewBikeGLWidget::resizeGL(int newWidth, int newHeight)
{
  renderer.resize(renderer.getFovY(), newWidth, newHeight);
}

bool ViewBikeGLWidget::clickControlPoint(const int& x, const int& y)
{
  Vector3<> vecNear, vecFar;

  gluUnProjectClick(x, y, vecFar, vecNear);

  for(int phaseNumber = 0; phaseNumber < parameters.numberOfPhases; phaseNumber++)
  {
    if(!moveDrag)
    {
      if(!widget.singleDraw || phaseNumber == widget.selectedPoint.phaseNumber)
      {
        for(int limb = 0; limb < Phase::numOfLimbs; limb++)
        {
          for(unsigned int i = 0; i < NUM_OF_POINTS; i++)
          {
            Vector3<> cubePoint;
            cubePoint = Vector3<>((double)(parameters.phaseParameters[phaseNumber].controlPoints[limb][i].x),
                                  (double)(parameters.phaseParameters[phaseNumber].controlPoints[limb][i].y),
                                  (double)(parameters.phaseParameters[phaseNumber].controlPoints[limb][i].z));


            Vector3<> intersection;
            if(ViewBikeMath::intersectRayAndBox(vecNear, vecFar, cubePoint, originRot, 15.0, 15.0, 15.0, intersection) && limb != Phase::leftFootRot && limb != Phase::rightFootRot && limb != Phase::rightHandRot && limb != Phase::leftHandRot)
            {
              widget.tabber->setCurrentIndex(phaseNumber + 1);
              widget.selectedPoint.phaseNumber = phaseNumber;
              widget.selectedPoint.pointNumber = i;
              widget.selectedPoint.limb = limb;
              widget.selectedPoint.xzRot = false;
              moveDrag = true;
              return true;
            }
          }
        }
      }
    }

    if((widget.tra2dWindows || widget.tra1dWindows) && !moveViewOfViewDragXY && !moveViewOfViewDragXZ && !moveViewOfViewDragYZ
       && !moveViewOfViewDragXP && !moveViewOfViewDragYP && !moveViewOfViewDragZP)
    {
      if(widget.selectedPoint.phaseNumber > -1 && widget.selectedPoint.limb > -1)
      {
        unsigned int width, height;
        renderer.getSize(width, height);

        float mini_window_width = width / 3.0f;
        float mini_window_height = (height - 150.0f) / 3.0f;

        float maxXrange = 100, minXrange = -100, maxYrange = 150, minYrange = -150, maxZrange = 100, minZrange = -250;

        std::vector<Vector2<> > cpWindow1, cpWindow2, cpWindow3;
        cpWindow1.resize(NUM_OF_POINTS);
        cpWindow2.resize(NUM_OF_POINTS);
        cpWindow3.resize(NUM_OF_POINTS);

        for(unsigned int k = 0; k < NUM_OF_POINTS; k++)
        {
          Vector3<> controlPoint = parameters.phaseParameters[widget.selectedPoint.phaseNumber].controlPoints[widget.selectedPoint.limb][k];

          minXrange = std::min<float>(minXrange, controlPoint.x);
          maxXrange = std::max<float> (maxXrange, controlPoint.x);
          minYrange = std::min<float>(minYrange, controlPoint.y);
          maxYrange = std::max<float>(maxYrange, controlPoint.y);
          minZrange = std::min<float>(minZrange, controlPoint.z);
          maxZrange = std::max<float>(maxZrange, controlPoint.z);
          if(widget.tra2dWindows)
          {
            cpWindow1[k] = (Vector2<>(controlPoint.x, controlPoint.y));
            cpWindow2[k] = (Vector2<>(controlPoint.x, controlPoint.z));
            cpWindow3[k] = (Vector2<>(controlPoint.y, controlPoint.z));
          }
          else
          {
            cpWindow1[k] = (Vector2<>((1.0f / 3.0f) * (float)(k + 1), controlPoint.x));
            cpWindow2[k] = (Vector2<>((1.0f / 3.0f) * (float)(k + 1), controlPoint.y));
            cpWindow3[k] = (Vector2<>((1.0f / 3.0f) * (float)(k + 1), controlPoint.z));
          }
        }
        //calculate zoomfactor
        float
        scaleFactorX = mini_window_width / std::max<>(maxXrange - minXrange, 1.0f),
        scaleFactorX1 = mini_window_height / std::max<>(maxXrange - minXrange, 1.0f),
        scaleFactorY = mini_window_height / std::max<>(maxYrange - minYrange, 1.0f),
        scaleFactorY1 = mini_window_width / std::max<>(maxYrange - minYrange, 1.0f),
        scaleFactorZ = mini_window_height / std::max<>(maxZrange - minZrange, 1.0f);


        for(unsigned int k = 0; k < cpWindow1.size(); k++)
        {
          if(widget.tra2dWindows)
          {

            //XY-Window
            float win1, win2;
            win1 = (float)width - 20.f - (float)width / 3.0f + ((cpWindow1[k].x - minXrange) * scaleFactorX);
            win2 = mini_window_height + 20.0f - ((cpWindow1[k].y - minYrange) * scaleFactorY);
            float localDist = sqrt(((float)x - win1) * ((float)x - win1) + ((float)y - win2) * ((float)y - win2));
            if(localDist < 10.f)
            {
              widget.selectedPoint.pointNumber = k;
              moveViewOfViewDragXY = true;
              return true;
            }
            //XZ-Window
            win1 = (float)width - 20.f - (float)width / 3.0f + ((cpWindow2[k].x - minXrange) * scaleFactorX);
            win2 = mini_window_height * 2.f + 55.0f - ((cpWindow2[k].y - minZrange) * scaleFactorZ);
            localDist = sqrt(((float)x - win1) * ((float)x - win1) + ((float)y - win2) * ((float)y - win2));
            if(localDist < 10.f)
            {
              widget.selectedPoint.pointNumber = k;
              moveViewOfViewDragXZ = true;
              return true;
            }
            //XZ-Window
            win1 = (float)width - 20.f - (float)width / 3.0f + ((cpWindow3[k].x - minYrange) * scaleFactorY1);
            win2 = mini_window_height * 3.f + 90 - ((cpWindow3[k].y - minZrange) * scaleFactorZ);
            localDist = sqrt(((float)x - win1) * ((float)x - win1) + ((float)y - win2) * ((float)y - win2));
            if(localDist < 10.f)
            {
              widget.selectedPoint.pointNumber = k;
              moveViewOfViewDragYZ = true;
              return true;
            }
          }
          else
          {
            //XP-Window
            float win1, win2;
            win1 = (float)width - 20.f - (float)width / 3.0f + (cpWindow1[k].x * mini_window_width);
            win2 = mini_window_height + 20.0f - ((cpWindow1[k].y - minXrange) * scaleFactorX1);
            float localDist = sqrt(((float)x - win1) * ((float)x - win1) + ((float)y - win2) * ((float)y - win2));
            if(localDist < 10.f)
            {
              widget.selectedPoint.pointNumber = k;
              moveViewOfViewDragXP = true;
              return true;
            }
            //YP-Window
            win1 = (float)width - 20.f - (float)width / 3.0f + (cpWindow1[k].x * mini_window_width);
            win2 = mini_window_height * 2.f + 55.0f - ((cpWindow2[k].y - minYrange) * scaleFactorY);
            localDist = sqrt(((float)x - win1) * ((float)x - win1) + ((float)y - win2) * ((float)y - win2));
            if(localDist < 10.0)
            {
              widget.selectedPoint.pointNumber = k;
              moveViewOfViewDragYP = true;
              return true;
            }
            //ZP-Window
            win1 = (float)width - 20.f - (float)width / 3.0f + (cpWindow1[k].x * mini_window_width);
            win2 = mini_window_height * 3.f + 90.f - ((cpWindow3[k].y - minZrange) * scaleFactorZ);
            localDist = sqrt(((float)x - win1) * ((float)x - win1) + ((float)y - win2) * ((float)y - win2));
            if(localDist < 10.f)
            {
              widget.selectedPoint.pointNumber = k;
              moveViewOfViewDragZP = true;
              return true;
            }

          }
        }
      }
    }
  }
  return false;
}

void ViewBikeGLWidget::gluUnProjectClick(int x, int y, Vector3<>& vecFar, Vector3<>& vecNear)
{
  glPushMatrix();
  setMatrix(viewBike.robot->getPosition(), originRot);
  glTranslatef(0.f, 0.f, -0.085f);
  glScaled(0.001, 0.001, 0.001);

  GLint viewport[4];
  glGetIntegerv(GL_VIEWPORT, viewport);
  GLdouble modelview[16];
  glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
  GLdouble projection[16];
  glGetDoublev(GL_PROJECTION_MATRIX, projection);
  GLfloat winX(0), winY(0);
  winX = (float) x;
  winY = (float) y;
  winY = (float)viewport[3] - winY;

  GLdouble tx, ty, tz;

  gluUnProject(winX, winY, 1.0, modelview, projection, viewport, &tx, &ty, &tz);

  vecFar.x = tx;
  vecFar.y = ty;
  vecFar.z = tz;

  gluUnProject(winX, winY, 0.0, modelview, projection, viewport, &tx, &ty, &tz);

  vecNear.x = tx;
  vecNear.y = ty;
  vecNear.z = tz;

  glPopMatrix();
}

void ViewBikeGLWidget::drawPhases()
{
  for(int phaseNumber = 0; phaseNumber < parameters.numberOfPhases; phaseNumber++)
  {
    if(!widget.singleDraw || phaseNumber == widget.selectedPoint.phaseNumber)
    {
      drawBezierCurves(phaseNumber);
      //Draw Bezierlines
      Vector3<> oPoint;

      for(int j = 0; j < Phase::numOfLimbs; j++)
      {
        if(j != Phase::leftFootRot && j != Phase::rightFootRot && j != Phase::rightHandRot && j != Phase::leftHandRot)
        {

          oPoint = parameters.getPositionBlame(0.f, phaseNumber, j);

          for(unsigned int k = 0; k < NUM_OF_POINTS; k++)
          {
            Vector3<> point1, point2;
            if(k == 0)
            {
              point1 = oPoint;
              point2 = parameters.phaseParameters[phaseNumber].controlPoints[j][k];
            }
            else
            {
              point1 = parameters.phaseParameters[phaseNumber].controlPoints[j][k - 1];
              point2 = parameters.phaseParameters[phaseNumber].controlPoints[j][k];
            }
            glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
            glBegin(GL_LINES);
            glVertex3f(point1.x, point1.y, point1.z);
            glVertex3f(point2.x, point2.y, point2.z);
            glEnd();
          }
        }
      }
    }
  }
}

void ViewBikeGLWidget::drawBezierCurves(const int& phaseNumber)
{
  //draw controlPoints
  Vector3 <> cubePoint[NUM_OF_POINTS][Phase::numOfLimbs], cubePoint2[NUM_OF_POINTS][Phase::numOfLimbs];

  for(int j = 0; j < Phase::numOfLimbs; j++)
  {
    for(unsigned int i = 0; i < NUM_OF_POINTS; i++)
    {
      cubePoint[i][j] = parameters.phaseParameters[phaseNumber].controlPoints[j][i];

      glColor4f(1.0f, 0.0f, 0.0f, 1.0f);

      if(widget.selectedPoint.phaseNumber == phaseNumber) glColor4f(0.0f, 1.0f, 0.0f, 1.0f);  //green selected Phase
      if(widget.selectedPoint.limb == j) glColor4f(1.0f, 1.0f, 0.0f, 1.0f);  //selected Limb yellow

      if((unsigned int) widget.selectedPoint.pointNumber == i)
      {
        if(moveDrag || moveViewOfViewDragYP || moveViewOfViewDragZP || moveViewOfViewDragXP
           || moveViewOfViewDragXY || moveViewOfViewDragXZ || moveViewOfViewDragYZ)
        {
          glColor4f(1.0f, 0.0f, 1.0f, 1.0f); //magenta moved selected point
        }
      }

      if(j == Phase::leftFootRot)
      {
        Vector3<> drawPos(0.f, 150.f, 0.f);
        drawPos = RotationMatrix::fromRotationZ(cubePoint[i][j].z) *
                  RotationMatrix::fromRotationX(cubePoint[i][j].x) * drawPos;
        cubePoint2[i][j] = Vector3<>(cubePoint[NUM_OF_POINTS - 1][Phase::leftFootTra].x + drawPos.x,
                                     cubePoint[NUM_OF_POINTS - 1][Phase::leftFootTra].y + drawPos.y,
                                     cubePoint[NUM_OF_POINTS - 1][Phase::leftFootTra].z + drawPos.z);

        Vector3<> drawPos2(150.f, 0.f, 0.f);
        drawPos2 = RotationMatrix::fromRotationZ(cubePoint[i][j].z) *
                   RotationMatrix::fromRotationY(cubePoint[i][j].y) * drawPos2;
        cubePoint[i][j] = Vector3<>(cubePoint[NUM_OF_POINTS - 1][Phase::leftFootTra].x + drawPos2.x,
                                    cubePoint[NUM_OF_POINTS - 1][Phase::leftFootTra].y + drawPos2.y,
                                    cubePoint[NUM_OF_POINTS - 1][Phase::leftFootTra].z + drawPos2.z);
      }

      if(j == Phase::rightFootRot)
      {
        Vector3<> drawPos(0.f, -150.f, 0.f);
        drawPos = RotationMatrix::fromRotationZ(cubePoint[i][j].z) *
                  RotationMatrix::fromRotationX(cubePoint[i][j].x) * drawPos;
        cubePoint2[i][j] = Vector3<>(cubePoint[NUM_OF_POINTS - 1][Phase::rightFootTra].x + drawPos.x,
                                     cubePoint[NUM_OF_POINTS - 1][Phase::rightFootTra].y + drawPos.y,
                                     cubePoint[NUM_OF_POINTS - 1][Phase::rightFootTra].z + drawPos.z);

        Vector3<> drawPos2(150.f, 0.f, 0.f);
        drawPos2 = RotationMatrix::fromRotationZ(cubePoint[i][j].z) *
                   RotationMatrix::fromRotationY(cubePoint[i][j].y) * drawPos2;
        cubePoint[i][j] = Vector3<>(cubePoint[NUM_OF_POINTS - 1][Phase::rightFootTra].x + drawPos2.x,
                                    cubePoint[NUM_OF_POINTS - 1][Phase::rightFootTra].y + drawPos2.y,
                                    cubePoint[NUM_OF_POINTS - 1][Phase::rightFootTra].z + drawPos2.z);
      }

      if(j == Phase::leftHandRot)
      {
        Vector3<> drawPos2(0.f, viewBike.robotDimensions.lowerArmLength, 0.f);
        drawPos2 = RotationMatrix::fromRotationX(cubePoint[i][j].x) * drawPos2;
        cubePoint2[i][j] = Vector3<>(cubePoint[NUM_OF_POINTS - 1][Phase::leftArmTra].x + drawPos2.x,
                                     cubePoint[NUM_OF_POINTS - 1][Phase::leftArmTra].y + drawPos2.y,
                                     cubePoint[NUM_OF_POINTS - 1][Phase::leftArmTra].z + drawPos2.z);

        Vector3<> lHDrawPos(-viewBike.robotDimensions.lowerArmLength, 0., 0.);
        lHDrawPos = RotationMatrix::fromRotationZ(cubePoint[i][j].z) *
                    RotationMatrix::fromRotationY(cubePoint[i][j].y) *
                    RotationMatrix::fromRotationX(cubePoint[i][j].x) * lHDrawPos;
        cubePoint[i][j] = Vector3<>(cubePoint[NUM_OF_POINTS - 1][Phase::leftArmTra].x + lHDrawPos.x,
                                    cubePoint[NUM_OF_POINTS - 1][Phase::leftArmTra].y + lHDrawPos.y,
                                    cubePoint[NUM_OF_POINTS - 1][Phase::leftArmTra].z + lHDrawPos.z);
      }

      if(j == Phase::rightHandRot)
      {
        Vector3<> drawPos2(0.f, -viewBike.robotDimensions.lowerArmLength, 0.f);
        drawPos2 = RotationMatrix::fromRotationX(cubePoint[i][j].x) * drawPos2;
        cubePoint2[i][j] = Vector3<>(cubePoint[NUM_OF_POINTS - 1][Phase::rightArmTra].x + drawPos2.x,
                                     cubePoint[NUM_OF_POINTS - 1][Phase::rightArmTra].y + drawPos2.y,
                                     cubePoint[NUM_OF_POINTS - 1][Phase::rightArmTra].z + drawPos2.z);

        Vector3<> rHDrawPos(-viewBike.robotDimensions.lowerArmLength, 0., 0.);
        rHDrawPos = RotationMatrix::fromRotationZ(cubePoint[i][j].z) *
                    RotationMatrix::fromRotationY(cubePoint[i][j].y) *
                    RotationMatrix::fromRotationX(cubePoint[i][j].x) * rHDrawPos;
        cubePoint[i][j] = Vector3<>(cubePoint[NUM_OF_POINTS - 1][Phase::rightArmTra].x + rHDrawPos.x,
                                    cubePoint[NUM_OF_POINTS - 1][Phase::rightArmTra].y + rHDrawPos.y,
                                    cubePoint[NUM_OF_POINTS - 1][Phase::rightArmTra].z + rHDrawPos.z);
      }

      if(i == NUM_OF_POINTS - 1)
      {
        Vector3<> rotation(0, 0, 0);

        glColor4f(0.0f, 1.0f, 1.0f, 1.0f);

        if((unsigned int) widget.selectedPoint.pointNumber == i && widget.selectedPoint.limb == j && widget.selectedPoint.phaseNumber == phaseNumber)
        {
          if(moveDrag || moveViewOfViewDragYP || moveViewOfViewDragZP || moveViewOfViewDragXP
             || moveViewOfViewDragXY || moveViewOfViewDragXZ || moveViewOfViewDragYZ)
          {
            glColor4f(1.0f, 0.0f, 1.0f, 1.0f); //magenta moved selected point
          }
        }

        if(j == Phase::leftFootRot || j == Phase::rightFootRot || j == Phase::rightHandRot  || j == Phase::leftHandRot)
        {
          glColor4f(0.84f, 0.84f, 0.85f, 1.f);
          calculateControlPointRot(cubePoint2[i - 2][j], cubePoint2[i - 1][j], cubePoint2[i][j], rotation);
          drawArrow(cubePoint2[i][j], rotation);
          glBegin(GL_LINES);
          glVertex3d(cubePoint2[i][j].x, cubePoint2[i][j].y, cubePoint2[i][j].z);
          glVertex3d(cubePoint[i][j - 1].x, cubePoint[i][j - 1].y, cubePoint[i][j - 1].z);

          glVertex3d(cubePoint[i][j].x, cubePoint[i][j].y, cubePoint[i][j].z);
          glVertex3d(cubePoint[i][j - 1].x, cubePoint[i][j - 1].y, cubePoint[i][j - 1].z);

          glEnd();
        }

        calculateControlPointRot(cubePoint[i - 2][j], cubePoint[i - 1][j], cubePoint[i][j], rotation);
        drawArrow(cubePoint[i][j], rotation);
      }
      else
      {
        if(j != Phase::leftFootRot && j != Phase::rightFootRot && j != Phase::rightHandRot && j != Phase::leftHandRot)
        {
          drawControlPoint(cubePoint[i][j], 5.0f);
        }
      }
    }
  }

  Vector3<> positions[Phase::numOfLimbs];
  Vector3<> rotation[Phase::numOfLimbs];

  float ps[1];
  glGetFloatv(GL_POINT_SIZE, ps);

  for(double phase = 0.0; phase < 1.0; phase += 0.02)
  {
    for(int j = 0; j < Phase::numOfLimbs; j++)
    {
      positions[j] = parameters.getPositionBlame(phase, phaseNumber, j);
      rotation[j] = Vector3<>(0.f, 0.f, 0.f);
    }

    if(widget.selectedPoint.phaseNumber == phaseNumber)
    {
      glColor4f(0.0f, 1.0f, 0.0f, 1.0f); //green selected Phase
    }
    else
    {
      glColor4f(0.0f, 0.0f, 1.0f, 1.0f);
    }

    glPointSize(4);
    glBegin(GL_POINTS);

    //clip legs
    float leg0, leg1, leg2, leg3, leg4, leg5;
    bool leftLegTooShort = ViewBikeMath::calcLegJoints(positions[Phase::leftFootTra], positions[Phase::leftFootRot], true, viewBike.robotDimensions, leg0, leg1, leg2, leg3, leg4, leg5);

    if(clipLegJointsWithLimits(leg1, leg2, leg3, JointData::LHipYawPitch) || leftLegTooShort)
    {
      positions[Phase::leftFootTra] = ViewBikeMath::calcFootPos(leg0, leg1, leg2, leg3, leg4, leg5, JointData::LHipYawPitch, viewBike.robotDimensions);
      glColor4f(1.0f, 0.5f, 0.f, 1.f);
    }
    else if(widget.selectedPoint.limb == Phase::leftFootTra)  glColor4f(1.0f, 1.0f, 0.0f, 1.0f);  //selected Limb yellow
    else if(widget.selectedPoint.phaseNumber == phaseNumber) glColor4f(0.0f, 1.0f, 0.0f, 1.0f);  //green selected Phase
    else glColor4f(0.0f, 0.0f, 1.0f, 1.0f);

    glVertex3f(positions[Phase::leftFootTra].x, positions[Phase::leftFootTra].y, positions[Phase::leftFootTra].z);

    bool rightLegTooShort = ViewBikeMath::calcLegJoints(positions[Phase::rightFootTra], positions[Phase::rightFootRot], false, viewBike.robotDimensions, leg0, leg1, leg2, leg3, leg4, leg5);

    if(clipLegJointsWithLimits(leg1, leg2, leg3, JointData::RHipYawPitch) || rightLegTooShort)
    {
      positions[Phase::rightFootTra] = ViewBikeMath::calcFootPos(leg0, leg1, leg2, leg3, leg4, leg5, JointData::RHipYawPitch, viewBike.robotDimensions);
      glColor4f(1.0f, 0.5f, 0.f, 1.f);
    }
    else if(widget.selectedPoint.limb == Phase::rightFootTra) glColor4f(1.0f, 1.0f, 0.0f, 1.0f);  //selected Limb yellow
    else if(widget.selectedPoint.phaseNumber == phaseNumber) glColor4f(0.0f, 1.0f, 0.0f, 1.0f);  //green selected Phase
    else glColor4f(0.0f, 0.0f, 1.0f, 1.0f);

    //draw footpos
    glVertex3f(positions[Phase::rightFootTra].x, positions[Phase::rightFootTra].y, positions[Phase::rightFootTra].z);

    if(widget.selectedPoint.phaseNumber == phaseNumber) glColor4f(0.7f, 1.0f, 0.75f, 1.0f);  //light green selected Phase
    else glColor4f(1.0f, 1.0f, 1.0f, 1.0f);

    // draw left footRotation
    Vector3<> lDrawPos(0., 150., 0.);
    lDrawPos = RotationMatrix::fromRotationZ(positions[Phase::leftFootRot].z) *
               RotationMatrix::fromRotationX(positions[Phase::leftFootRot].x) * lDrawPos;
    glVertex3f(cubePoint[NUM_OF_POINTS - 1][Phase::leftFootTra].x + lDrawPos.x,
               cubePoint[NUM_OF_POINTS - 1][Phase::leftFootTra].y + lDrawPos.y,
               cubePoint[NUM_OF_POINTS - 1][Phase::leftFootTra].z + lDrawPos.z);

    Vector3<> lDrawPos2(150., 0., 0.);
    lDrawPos2 = RotationMatrix::fromRotationZ(positions[Phase::leftFootRot].z) *
                RotationMatrix::fromRotationY(positions[Phase::leftFootRot].y) * lDrawPos2;
    glVertex3f(cubePoint[NUM_OF_POINTS - 1][Phase::leftFootTra].x + lDrawPos2.x,
               cubePoint[NUM_OF_POINTS - 1][Phase::leftFootTra].y + lDrawPos2.y,
               cubePoint[NUM_OF_POINTS - 1][Phase::leftFootTra].z + lDrawPos2.z);

    //draw right footRotation
    Vector3<> rDrawPos(0., -150., 0.);
    rDrawPos = RotationMatrix::fromRotationZ(positions[Phase::rightFootRot].z) *
               RotationMatrix::fromRotationX(positions[Phase::rightFootRot].x) * rDrawPos;
    glVertex3f(cubePoint[NUM_OF_POINTS - 1][Phase::rightFootTra].x + rDrawPos.x,
               cubePoint[NUM_OF_POINTS - 1][Phase::rightFootTra].y + rDrawPos.y,
               cubePoint[NUM_OF_POINTS - 1][Phase::rightFootTra].z + rDrawPos.z);

    Vector3<> rDrawPos2(150., 0., 0.);
    rDrawPos2 = RotationMatrix::fromRotationZ(positions[Phase::rightFootRot].z) *
                RotationMatrix::fromRotationY(positions[Phase::rightFootRot].y) * rDrawPos2;
    glVertex3f(cubePoint[NUM_OF_POINTS - 1][Phase::rightFootTra].x + rDrawPos2.x,
               cubePoint[NUM_OF_POINTS - 1][Phase::rightFootTra].y + rDrawPos2.y,
               cubePoint[NUM_OF_POINTS - 1][Phase::rightFootTra].z + rDrawPos2.z);

    glEnd();
    glPushMatrix();

    glBegin(GL_POINTS);

    if(widget.selectedPoint.phaseNumber == phaseNumber) glColor4f(0.7f, 1.0f, 0.75f, 1.0f);  //green selected Phase
    else glColor4f(1.0f, 1.0f, 1.0f, 1.0f);

    Vector3<> drawPos3(0.f, viewBike.robotDimensions.lowerArmLength, 0.f);
    drawPos3 = RotationMatrix::fromRotationX(positions[Phase::leftHandRot].x) * drawPos3;
    glVertex3f(cubePoint[NUM_OF_POINTS - 1][Phase::leftArmTra].x + drawPos3.x,
               cubePoint[NUM_OF_POINTS - 1][Phase::leftArmTra].y + drawPos3.y,
               cubePoint[NUM_OF_POINTS - 1][Phase::leftArmTra].z + drawPos3.z);

    //draw left Handrotation
    Vector3<> lHDrawPos(-viewBike.robotDimensions.lowerArmLength, 0., 0.);
    lHDrawPos = RotationMatrix::fromRotationZ(positions[Phase::leftHandRot].z) *
                RotationMatrix::fromRotationY(positions[Phase::leftHandRot].y) *
                RotationMatrix::fromRotationX(positions[Phase::leftHandRot].x) * lHDrawPos;
    glVertex3f(cubePoint[NUM_OF_POINTS - 1][Phase::leftArmTra].x + lHDrawPos.x,
               cubePoint[NUM_OF_POINTS - 1][Phase::leftArmTra].y + lHDrawPos.y,
               cubePoint[NUM_OF_POINTS - 1][Phase::leftArmTra].z + lHDrawPos.z);


    Vector3<> drawPos2(0.f, -viewBike.robotDimensions.lowerArmLength, 0.f);
    drawPos2 = RotationMatrix::fromRotationX(positions[Phase::rightHandRot].x) * drawPos2;
    glVertex3f(cubePoint[NUM_OF_POINTS - 1][Phase::rightArmTra].x + drawPos2.x,
               cubePoint[NUM_OF_POINTS - 1][Phase::rightArmTra].y + drawPos2.y,
               cubePoint[NUM_OF_POINTS - 1][Phase::rightArmTra].z + drawPos2.z);

    //draw right Handrotation
    Vector3<> rHDrawPos(-viewBike.robotDimensions.lowerArmLength, 0., 0.);
    rHDrawPos = RotationMatrix::fromRotationZ(positions[Phase::rightHandRot].z) *
                RotationMatrix::fromRotationY(positions[Phase::rightHandRot].y) *
                RotationMatrix::fromRotationX(positions[Phase::rightHandRot].x) * rHDrawPos;
    glVertex3f(cubePoint[NUM_OF_POINTS - 1][Phase::rightArmTra].x + rHDrawPos.x,
               cubePoint[NUM_OF_POINTS - 1][Phase::rightArmTra].y + rHDrawPos.y,
               cubePoint[NUM_OF_POINTS - 1][Phase::rightArmTra].z + rHDrawPos.z);

    if(widget.selectedPoint.limb == Phase::rightArmTra) glColor4f(1.0f, 1.0f, 0.0f, 1.0f);  //selected Limb yellow
    else if(widget.selectedPoint.phaseNumber == phaseNumber) glColor4f(0.0f, 1.0f, 0.0f, 1.0f);  //green selected Phase
    else glColor4f(0.0f, 0.0f, 1.0f, 1.0f);

    glVertex3f(positions[Phase::rightArmTra].x, positions[Phase::rightArmTra].y, positions[Phase::rightArmTra].z);

    if(widget.selectedPoint.limb == Phase::leftArmTra)  glColor4f(1.0f, 1.0f, 0.0f, 1.0f);  //selected Limb yellow
    else if(widget.selectedPoint.phaseNumber == phaseNumber) glColor4f(0.0f, 1.0f, 0.0f, 1.0f);  //green selected Phase
    else glColor4f(0.0f, 0.0f, 1.0f, 1.0f);

    glVertex3f(positions[Phase::leftArmTra].x, positions[Phase::leftArmTra].y, positions[Phase::leftArmTra].z);
    glEnd();
    glPointSize(ps[0]);

    glPopMatrix();
  }
}

void ViewBikeGLWidget::calculateControlPointRot(const Vector3 <>& point0, const Vector3<>& point1, const Vector3<> point2, Vector3<>& rotation)
{
  Vector3<> vec1(point2.x - point1.x,
                 point2.y - point1.y,
                 point2.z - point1.z);

  if(vec1.abs() > 1.f)
  {
    rotation.x = toDegrees(atan2(vec1.y, vec1.z));
    rotation.y = toDegrees(vec1.z <= 0 ?  atan2(vec1.x, vec1.z) : pi - atan2(vec1.x, vec1.z));
    rotation.z = toDegrees(atan2(vec1.x, vec1.y));
  }
  else
  {
    Vector3<> vec(point2.x - point0.x,
                  point2.y - point0.y,
                  point2.z - point0.z);

    rotation.x = toDegrees(atan2(vec.y, vec.z));
    rotation.y = toDegrees(vec.z <= 0 ?  atan2(vec.x, vec.z) : pi - atan2(vec.x, vec.z));
    rotation.z = toDegrees(atan2(vec.x, vec.y));
  }
}

void ViewBikeGLWidget::drawArrow(const Vector3 <>& point, const Vector3<>& rotation)
{
  glPushMatrix();
  setMatrix(&point.x, originRot);
  glRotatef(rotation.x, 1.f, 0.f, 0.f);
  glRotatef(rotation.y, 0.f, 1.f, 0.f);
  glRotatef(rotation.z, 0.f, 0.f, 1.f);
  glTranslatef(0.f, 0.f, -7.f);
  GLUquadric* quadric = gluNewQuadric();
  gluQuadricDrawStyle(quadric, GLU_FILL);
  gluCylinder(quadric, 0., 10., 15., 15., 15.);
  glTranslated(0., 0., 15.);
  gluDisk(quadric, 0., 10., 15., 15.);
  gluDeleteQuadric(quadric);
  glPopMatrix();
}

void ViewBikeGLWidget::drawControlPoint(const Vector3 <>& point, const float& cubeFaktor)
{
  glBegin(GL_QUADS);
  glVertex3f(point.x + cubeFaktor, point.y + cubeFaktor, point.z - cubeFaktor);
  glVertex3f(point.x - cubeFaktor, point.y + cubeFaktor, point.z - cubeFaktor);
  glVertex3f(point.x - cubeFaktor, point.y + cubeFaktor, point.z + cubeFaktor);
  glVertex3f(point.x + cubeFaktor, point.y + cubeFaktor, point.z + cubeFaktor);

  glVertex3f(point.x + cubeFaktor, point.y - cubeFaktor, point.z + cubeFaktor);
  glVertex3f(point.x - cubeFaktor, point.y - cubeFaktor, point.z + cubeFaktor);
  glVertex3f(point.x - cubeFaktor, point.y - cubeFaktor, point.z - cubeFaktor);
  glVertex3f(point.x + cubeFaktor, point.y - cubeFaktor, point.z - cubeFaktor);

  glVertex3f(point.x + cubeFaktor, point.y + cubeFaktor, point.z + cubeFaktor);
  glVertex3f(point.x - cubeFaktor, point.y + cubeFaktor, point.z + cubeFaktor);
  glVertex3f(point.x - cubeFaktor, point.y - cubeFaktor, point.z + cubeFaktor);
  glVertex3f(point.x + cubeFaktor, point.y - cubeFaktor, point.z + cubeFaktor);

  glVertex3f(point.x + cubeFaktor, point.y - cubeFaktor, point.z - cubeFaktor);
  glVertex3f(point.x - cubeFaktor, point.y - cubeFaktor, point.z - cubeFaktor);
  glVertex3f(point.x - cubeFaktor, point.y + cubeFaktor, point.z - cubeFaktor);
  glVertex3f(point.x + cubeFaktor, point.y + cubeFaktor, point.z - cubeFaktor);

  glVertex3f(point.x - cubeFaktor, point.y + cubeFaktor, point.z + cubeFaktor);
  glVertex3f(point.x - cubeFaktor, point.y + cubeFaktor, point.z - cubeFaktor);
  glVertex3f(point.x - cubeFaktor, point.y - cubeFaktor, point.z - cubeFaktor);
  glVertex3f(point.x - cubeFaktor, point.y - cubeFaktor, point.z + cubeFaktor);

  glVertex3f(point.x + cubeFaktor, point.y + cubeFaktor, point.z - cubeFaktor);
  glVertex3f(point.x + cubeFaktor, point.y + cubeFaktor, point.z + cubeFaktor);
  glVertex3f(point.x + cubeFaktor, point.y - cubeFaktor, point.z + cubeFaktor);
  glVertex3f(point.x + cubeFaktor, point.y - cubeFaktor, point.z - cubeFaktor);
  glEnd();
}

void ViewBikeGLWidget::paintGL()
{
  float position[3];
  float r[3][3];
  viewBike.robot->getPose(position, r);
  originRot = RotationMatrix(Matrix3x3<>(r[0][0], r[0][1], r[0][2], r[1][0], r[1][1], r[1][2], r[2][0], r[2][1], r[2][2]));
  originRot = RotationMatrix::fromRotationZ(originRot.getZAngle());

  if(widget.getString > 0)
  {
    widget.commands.push_back("_get2 representation:MotionRequest");
    widget.getString--;
  }

  if(!widget.commands.empty())
  {
    if(widget.commands[0] == " ")
    {
      widget.commands.erase(widget.commands.begin());
    }
    else
    {
      viewBike.console.handleConsole(widget.commands[0]);
      widget.commands.erase(widget.commands.begin());
    }
  }

  if(viewBike.motionRequest.motion == MotionRequest::bike && widget.lastMotion != MotionRequest::bike)
  {
    for(int i = 0; i < Phase::numOfLimbs; i++)
    {
      reachedPositions[i].clear();
    }
  }
  widget.lastMotion = viewBike.motionRequest.motion;

  if(viewBike.motionRequest.motion == MotionRequest::bike && widget.reachedDraw)
  {
    Vector3<> positions[Phase::numOfLimbs];

    positions[Phase::leftFootTra] = ViewBikeMath::calculateFootPos(viewBike.sensorData, viewBike.jointData, JointData::LHipYawPitch, viewBike.robotDimensions).translation;
    positions[Phase::rightFootTra] = ViewBikeMath::calculateFootPos(viewBike.sensorData, viewBike.jointData, JointData::RHipYawPitch, viewBike.robotDimensions).translation;

    positions[Phase::leftArmTra] = ViewBikeMath::calculateHandPos(viewBike.jointData, JointData::LShoulderPitch, viewBike.robotDimensions).translation;
    positions[Phase::rightArmTra] = ViewBikeMath::calculateHandPos(viewBike.jointData, JointData::RShoulderPitch, viewBike.robotDimensions).translation;

    for(int i = 0; i < Phase::numOfLimbs; i++)
    {
      reachedPositions[i].push_back(positions[i]);
    }
  }

  //  Snap the Camera to Object
  Vector3<> cameraPos,
          targetPos;
  renderer.getCamera(&cameraPos.x, &targetPos.x);
  const float* p = viewBike.robot->getPosition();
  Vector3<> newTargetPos(p[0], p[1], p[2] - 0.06f);
  cameraPos -= targetPos;

  Vector3<> targetPosTempOffset(targetPosOffset.x, targetPosOffset.y, targetPosOffset.z),
          cameraPosTempOffset(cameraPosOffset.x, cameraPosOffset.y, cameraPosOffset.z);

  cameraPosTempOffset = RotationMatrix::fromRotationZ(-atan2(cameraPos.x, cameraPos.y)) * cameraPosTempOffset;
  targetPosTempOffset = RotationMatrix::fromRotationZ(-atan2(cameraPos.x, cameraPos.y)) * targetPosTempOffset;

  cameraPos += newTargetPos;
  targetPos = newTargetPos;

  Vector3<> cp = cameraPos + cameraPosTempOffset;
  Vector3<> tp = targetPos + targetPosTempOffset;
  renderer.setCamera(&cp.x, &tp.x);

  unsigned int width, height;
  renderer.getSize(width, height);

  if(widget.ghost)
  {
    glEnable(GL_POLYGON_STIPPLE);
    glPolygonStipple(stippleMask[16 - widget.ghost]);
    renderer.draw();
    glDisable(GL_POLYGON_STIPPLE);
  }
  else
    renderer.draw();

  glDisable(GL_BLEND);
  glEnable(GL_DEPTH_TEST);

  glPushMatrix();
  setMatrix(viewBike.robot->getPosition(), originRot);
  glTranslatef(0.f, 0.f, -0.085f);
  glScaled(0.001, 0.001, 0.001);

  if(widget.reachedDraw)
  {
    float ps[1];
    glGetFloatv(GL_POINT_SIZE, ps);
    glPointSize(4);
    glColor4f(1.f, 0.f, 1.f, 1.f);
    glBegin(GL_POINTS);
    for(int i = 0; i < Phase::numOfLimbs; i++)
    {
      if(!reachedPositions[i].empty())
      {
        for(unsigned int j = 0; j  < reachedPositions[i].size(); ++j)
        {
          glVertex3d(reachedPositions[i][j].x, reachedPositions[i][j].y, reachedPositions[i][j].z);

        }
      }
    }
    glEnd();
    glPointSize(ps[0]);
  }
  if(widget.phaseDrawings)
  {
    drawPhases();
  }
  glPopMatrix();
  glFlush();

  if(widget.tra2dWindows || widget.tra1dWindows || widget.velocityWindows || widget.accelWindows)
  {
    // draw background
    glDisable(GL_DEPTH_TEST);
    glColor4f(0.0f, 0.0f, 0.0f, 0.6f);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_BLEND);
    glMatrixMode(GL_PROJECTION);

    glPushMatrix();
    glLoadIdentity();
    glOrtho(0.0, width, 0.0, height, 0.0, 0.2);
    glMatrixMode(GL_MODELVIEW);

    glPushMatrix();

    glLoadIdentity();

    float offsetTop = 20.f;
    float offsetBetween = 35.f;

    float mini_window_width = width / 3.0f;
    float mini_window_height = (height - 150.0f) / 3.0f;

    glTranslatef(width - width / 3.0f - 55, height, 0.f);

    //draw the backgrounds
    glBegin(GL_QUADS);
    glVertex2f(0.0f,  -(mini_window_height * 3) - (offsetBetween * 3) - 10);
    glVertex2f(mini_window_width + 55, -(mini_window_height * 3) - (offsetBetween * 3) - 10);
    glVertex2f(mini_window_width + 55, 0.0f);
    glVertex2f(0.0f, 0.0f);
    glEnd();

    glBlendFunc(GL_DST_COLOR, GL_SRC_ALPHA);

    glTranslatef(35.f, -offsetTop, 0.f);

    glColor4f(1.0f, 0.0f, 0.0f, 1.f);

    if(widget.tra2dWindows)
    {
      renderText(0.5f, 4.0f, 0, "[C(phase) with P[x,y]", QFont("arial", 10));
      renderText(0.5f, -mini_window_height - offsetBetween + 4.0f, 0, "[C(phase) with P[x,z]]", QFont("arial", 10));
      renderText(0.5f, -mini_window_height * 2 - offsetBetween * 2 + 4.0f, 0, "[C(phase) with P[y,z]]", QFont("arial", 10));
    }
    else if(widget.tra1dWindows)
    {
      renderText(0.5f, 4.0f, 0, "[x:phase] with phase = [0..1]", QFont("arial", 10));
      renderText(0.5f, -mini_window_height - offsetBetween + 4.0f, 0, "[y:phase] with phase = [0..1]", QFont("arial", 10));
      renderText(0.5f, -mini_window_height * 2 - offsetBetween * 2 + 4.0f, 0, "[z:phase]  with phase = [0..1]", QFont("arial", 10));
    }
    else if(widget.velocityWindows)
    {
      renderText(0.5f, 4.0f, 0, "[x':phase] with phase = [0..1]", QFont("arial", 10));
      renderText(0.5f, -mini_window_height - offsetBetween + 4.0f, 0, "[y':phase] with phase = [0..1]", QFont("arial", 10));
      renderText(0.5f, -mini_window_height * 2 - offsetBetween * 2 + 4.0f, 0, "[z':phase]  with phase = [0..1]", QFont("arial", 10));
    }
    else if(widget.accelWindows)
    {
      renderText(0.5f, 4.0f, 0, "[x'':phase] with phase = [0..1]", QFont("arial", 10));
      renderText(0.5f, -mini_window_height - offsetBetween + 4.0f, 0, "[y'':phase] with phase = [0..1]", QFont("arial", 10));
      renderText(0.5f, -mini_window_height * 2 - offsetBetween * 2 + 4.0f, 0, "[z'':phase]  with phase = [0..1]", QFont("arial", 10));
    }

    glColor4f(1.0f, 0.0f, 0.0f, 0.6f);
    if(widget.selectedPoint.phaseNumber > -1 && widget.selectedPoint.limb > -1)
    {
      float maxXrange = 100, minXrange = -100, maxYrange = 150, minYrange = -150, maxZrange = 100, minZrange = -250;

      std::vector<Vector2<> > cpWindow1, cpWindow2, cpWindow3, pWindow1, pWindow2, pWindow3;

      for(unsigned int k = 0; k < NUM_OF_POINTS; k++)
      {
        Vector3<> controlPoint = parameters.phaseParameters[widget.selectedPoint.phaseNumber].controlPoints[widget.selectedPoint.limb][k];

        minXrange = std::min<float>(minXrange, controlPoint.x);
        maxXrange = std::max<float>(maxXrange, controlPoint.x);
        minYrange = std::min<float>(minYrange, controlPoint.y);
        maxYrange = std::max<float>(maxYrange, controlPoint.y);
        minZrange = std::min<float>(minZrange, controlPoint.z);
        maxZrange = std::max<float>(maxZrange, controlPoint.z);

        if(widget.tra2dWindows)
        {
          cpWindow1.push_back(Vector2<>(controlPoint.x, controlPoint.y));
          cpWindow2.push_back(Vector2<>(controlPoint.x, controlPoint.z));
          cpWindow3.push_back(Vector2<>(controlPoint.y, controlPoint.z));
        }
        else
        {
          cpWindow1.push_back(Vector2<>((1.0f / 3.0f) * (k + 1), controlPoint.x));
          cpWindow2.push_back(Vector2<>((1.0f / 3.0f) * (k + 1), controlPoint.y));
          cpWindow3.push_back(Vector2<>((1.0f / 3.0f) * (k + 1), controlPoint.z));
        }

      }

      if(widget.tra1dWindows || widget.tra2dWindows)
      {
        for(float phase = 0.f; phase <= 1.f;  phase += 0.01f)
        {
          Vector3<> point = parameters.getPositionBlame(phase, widget.selectedPoint.phaseNumber, widget.selectedPoint.limb);

          if(widget.tra2dWindows)
          {
            pWindow1.push_back(Vector2<>(point.x, point.y));
            pWindow2.push_back(Vector2<>(point.x, point.z));
            pWindow3.push_back(Vector2<>(point.y, point.z));
          }
          else
          {
            pWindow1.push_back(Vector2<>(phase, point.x));
            pWindow2.push_back(Vector2<>(phase, point.y));
            pWindow3.push_back(Vector2<>(phase, point.z));
          }
        }
      }
      else
      {
        Vector3<> p0 = parameters.getPositionBlame(0.f, widget.selectedPoint.phaseNumber, widget.selectedPoint.limb);

        for(float phase = 0.f; phase <= 1.f; phase += 0.01f)
        {
          Vector3<> point;
          if(widget.velocityWindows)
          {
            point.x = (-3.f + 6.f * phase - 3.f * phase * phase) * p0.x + 3.f * (2 * phase - 4.f * phase + 3.f * phase * phase) * cpWindow1[0].y + 3.f * (2.f * phase - 3.f * phase * phase) * cpWindow1[1].y + 3.f * phase * phase * cpWindow1[2].y;
            point.y = (-3.f + 6.f * phase - 3.f * phase * phase) * p0.y + 3.f * (2 * phase - 4.f * phase + 3.f * phase * phase) * cpWindow2[0].y + 3.f * (2.f * phase - 3.f * phase * phase) * cpWindow2[1].y + 3.f * phase * phase * cpWindow2[2].y;
            point.z = (-3.f + 6.f * phase - 3.f * phase * phase) * p0.z + 3.f * (2 * phase - 4.f * phase + 3.f * phase * phase) * cpWindow3[0].y + 3.f * (2.f * phase - 3.f * phase * phase) * cpWindow3[1].y + 3.f * phase * phase * cpWindow3[2].y;
          }
          else
          {
            point.x = (6.f - 6.f * phase) * p0.x + 3.f * (-2.f + 6.f * phase) * cpWindow1[0].y + 3.f * (2.f - 6.f * phase) * cpWindow1[1].y + 6.f * phase * cpWindow1[2].y;
            point.y = (6.f - 6.f * phase) * p0.y + 3.f * (-2.f + 6.f * phase) * cpWindow2[0].y + 3.f * (2.f - 6.f * phase) * cpWindow2[1].y + 6.f * phase * cpWindow2[2].y;
            point.z = (6.f - 6.f * phase) * p0.z + 3.f * (-2.f + 6.f * phase) * cpWindow3[0].y + 3.f * (2.f - 6.f * phase) * cpWindow3[1].y + 6.f * phase * cpWindow3[2].y;
          }
          //point+=(originOffset-originPos);

          minXrange = std::min<float>(minXrange, point.x);
          maxXrange = std::max<float>(maxXrange, point.x);
          minYrange = std::min<float>(minYrange, point.y);
          maxYrange = std::max<float>(maxYrange, point.y);
          minZrange = std::min<float>(minZrange, point.z);
          maxZrange = std::max<float>(maxZrange, point.z);

          pWindow1.push_back(Vector2<>(phase, point.x));
          pWindow2.push_back(Vector2<>(phase, point.y));
          pWindow3.push_back(Vector2<>(phase, point.z));
        }
      }

      //calculate zoomfactor
      float
      scaleFactorX = mini_window_width / std::max<>(maxXrange - minXrange, 1.0f),
      scaleFactorX1 = mini_window_height / std::max<>(maxXrange - minXrange, 1.0f),
      scaleFactorY = mini_window_height / std::max<>(maxYrange - minYrange, 1.0f),
      scaleFactorY1 = mini_window_width / std::max<>(maxYrange - minYrange, 1.0f),
      scaleFactorZ = mini_window_height / std::max<>(maxZrange - minZrange, 1.0f);

      float colorX[] = {1.0f, 0.0f, 0.0f, 1.0f}, colorY[] = {0.0f, 1.0f, 0.0f, 1.0f}, colorZ[] = {0.0f, 1.0f, 1.0f, 1.0f};
      if(widget.tra2dWindows)
      {
        draw2dCurves(minXrange, minYrange, maxXrange, maxYrange, scaleFactorX, scaleFactorY, -minXrange, -minYrange,
                     colorX, colorY, -mini_window_height, cpWindow1, pWindow1);
        draw2dCurves(minXrange, minZrange, maxXrange, maxZrange, scaleFactorX, scaleFactorZ, -minXrange, -minZrange,
                     colorX, colorZ, -mini_window_height * 2 - offsetBetween, cpWindow2, pWindow2);
        draw2dCurves(minYrange, minZrange, maxYrange, maxZrange, scaleFactorY1, scaleFactorZ, -minYrange, -minZrange,
                     colorY, colorZ, -mini_window_height * 3 - offsetBetween * 2, cpWindow3, pWindow3);
      }
      else
      {
        float colorPhase[] = {1.0f, 1.0f, 1.0f, 1.0f};
        draw2dCurves(0.0f, minXrange, 1.0f, maxXrange, mini_window_width, scaleFactorX1, 0.0f, -minXrange,
                     colorPhase, colorX, -mini_window_height, cpWindow1, pWindow1);
        draw2dCurves(0.0f, minYrange, 1.0f, maxYrange, mini_window_width, scaleFactorY, 0.0f, -minYrange,
                     colorPhase, colorY, -mini_window_height * 2 - offsetBetween, cpWindow2, pWindow2);
        draw2dCurves(0.0f, minZrange, 1.0f, maxZrange, mini_window_width, scaleFactorZ, 0.0f, -minZrange,
                     colorPhase, colorZ, -mini_window_height * 3 - offsetBetween * 2, cpWindow3, pWindow3);
      }
    }
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();

    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();

    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
  }

  if(moveDrag || moveViewOfViewDragXY || moveViewOfViewDragXZ || moveViewOfViewDragYZ || moveViewOfViewDragXP ||
     moveViewOfViewDragYP || moveViewOfViewDragZP)
  {
    showPlane();
    Vector3<> point = parameters.phaseParameters[widget.selectedPoint.phaseNumber].controlPoints[widget.selectedPoint.limb][widget.selectedPoint.pointNumber];
    glDisable(GL_DEPTH_TEST);
    glColor4f(1.0f, 1.0f, 0.0f, 1.0f);
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0.0, width, 0.0, height, 0.0, 0.2);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    glTranslatef((float)actualX - 30, (float)(height - actualY - 30), 0.0f);
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    renderText(0.0, 0.0, 0.0, "point " + QString::number(widget.selectedPoint.pointNumber), QFont("arial", 10));
    glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
    renderText(0.0, -15.0, 0.0, "x:" + QString::number(point.x), QFont("arial", 10));
    glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
    renderText(0.0, -30.0, 0.0, "y:" + QString::number(point.y), QFont("arial", 10));
    glColor4f(0.0f, 1.0f, 1.0f, 1.0f);
    renderText(0.0, -45.0, 0.0, "z:" + QString::number(point.z), QFont("arial", 10));

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();

    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
    glEnable(GL_DEPTH_TEST);

  }
  glFlush();
}

void ViewBikeGLWidget::draw2dCurves(const float& minA, const float& minB, const float& maxA, const float& maxB, const float& scaleFactorA, const float& scaleFactorB, const float& transA, const float& transB, float* colorA, float* colorB, const float& window, const std::vector<Vector2<> >& cp, const std::vector<Vector2<> >& point)
{
  glPushMatrix();
  glTranslatef(0.0f, window, 0.0f);
  float ps[1], lw[1];
  glGetFloatv(GL_POINT_SIZE, ps);
  glGetFloatv(GL_LINE_WIDTH, lw);

  glScalef(scaleFactorA, scaleFactorB, 1.0f);

  //draw grid and axes
  glColor4fv(colorA);

  //draw zeroline
  if(minA <= 0.0f && maxA >= 0.0f)
  {
    glLineWidth(4);
    glBegin(GL_LINES);
    glVertex2f(transA, minB + transB);
    glVertex2f(transA, maxB + transB);
    glEnd();
    renderText(double(transA), double(minB + transB - 15.0f / scaleFactorB), 0.0 , QString::number(0), QFont("arial", 8));
  }

  float tickA = floor(maxA - minA) / 10.0f;
  glLineWidth(1);
  for(float w = ceil(minA / tickA) * tickA; w <= maxA + 0.01; w += tickA)
  {
    if(abs(w) > 0.001)  //don't draw zeroline twice
    {
      glBegin(GL_LINES);
      glVertex2f(w + transA, minB + transB);
      glVertex2f(w + transA, maxB + transB);
      glEnd();
      renderText(double(w + transA), double(minB + transB - 15.0f / scaleFactorB), 0.0 , QString::number(w), QFont("arial", 8));
    }
  }

  glColor4fv(colorB);
  float tickB = floor(maxB - minB) / 10.0f;

  if(minB <= 0.0f && maxB >= 0.0f)
  {
    glLineWidth(4);
    glBegin(GL_LINES);
    glVertex2f(minA + transA, transB);
    glVertex2f(maxA + transA, transB);
    glEnd();
    renderText((double)(minA + transA - 30.0f / scaleFactorA) , (double)(transB), 0.0, QString::number(0),  QFont("arial", 8));
  }

  glLineWidth(1);
  for(float h = ceil(minB / tickB) * tickB; h <= maxB + 0.01; h += tickB)
  {
    if(abs(h) > 0.001)  //don't draw zeroline twice
    {
      glBegin(GL_LINES);
      glVertex2f(minA + transA, h + transB);
      glVertex2f(maxA + transA, h + transB);
      glEnd();
      renderText((double)(minA + transA - 30.0f / scaleFactorA) , (double)(h + transB), 0.0, QString::number(h),  QFont("arial", 8));
    }
  }

  //draw controlPoints
  glColor4f(1.0, 1.0, 1.0, 1.0);
  glPointSize(10);
  for(unsigned int k = 0; k < cp.size(); k++)
  {
    glBegin(GL_POINTS);
    glVertex2f(cp[k].x + transA, cp[k].y + transB);
    glEnd();
  }

  //draw curve
  glPointSize(4);
  for(unsigned int k = 0; k < point.size(); k++)
  {
    glBegin(GL_POINTS);
    glVertex2f(point[k].x + transA, point[k].y + transB);
    glEnd();
  }
  glPointSize(ps[0]);
  glLineWidth(lw[0]);
  glPopMatrix();
}

GLvoid ViewBikeGLWidget::setMatrix(const float* translation, const RotationMatrix& rotation)
{
  GLfloat modelmatrix[16];
  modelmatrix[0] = rotation.c0.x;
  modelmatrix[1] = rotation.c0.y;
  modelmatrix[2] = rotation.c0.z;
  modelmatrix[3] = modelmatrix[7] = modelmatrix[11] = 0.0f;
  modelmatrix[4] = rotation.c1.x;
  modelmatrix[5] = rotation.c1.y;
  modelmatrix[6] = rotation.c1.z;
  modelmatrix[8] = rotation.c2.x;
  modelmatrix[9] = rotation.c2.y;
  modelmatrix[10] = rotation.c2.z;
  modelmatrix[12] = translation[0];
  modelmatrix[13] = translation[1];
  modelmatrix[14] = translation[2];
  modelmatrix[15] = 1.0f;
  glMultMatrixf(modelmatrix);
}

void ViewBikeGLWidget::keyPressEvent(QKeyEvent* event)
{
  switch(event->key())
  {
  case Qt::Key_PageUp:
    event->accept();
    renderer.zoom(-100);
    updateGL();
    break;

  case Qt::Key_PageDown:
    event->accept();
    renderer.zoom(100);
    updateGL();
    break;

  case Qt::Key_Left:
    event->accept();
    cameraPosOffset.x -= 0.1f;
    targetPosOffset.x -= 0.1f;
    updateGL();
    break;

  case Qt::Key_Right:
    event->accept();
    cameraPosOffset.x += 0.1f;
    targetPosOffset.x += 0.1f;
    updateGL();
    break;
  case Qt::Key_X:
    widget.setDragPlane((int) SimRobotCore2::Renderer::yzPlane);
    break;
  case Qt::Key_Y:
    widget.setDragPlane((int) SimRobotCore2::Renderer::xzPlane);
    break;
  case Qt::Key_Z:
    widget.setDragPlane((int) SimRobotCore2::Renderer::xyPlane);
    break;

  default:
    QGLWidget::keyPressEvent(event);
    break;
  }
}

void ViewBikeGLWidget::wheelEvent(QWheelEvent* event)
{
  QWidget::wheelEvent(event);

  if(event->orientation() == Qt::Vertical)
  {
    renderer.zoom(event->delta());
    updateGL();
    event->accept();
  }
}

void ViewBikeGLWidget::mouseMoveEvent(QMouseEvent* event)
{
  Vector3<> newTarPos, camPos, camTarget, translationVec(0, 0, 0);
  renderer.getCamera(&camPos.x, &camTarget.x);
  const float* p = viewBike.robot->getPosition();
  newTarPos = Vector3<>(p[0], p[1], p[2]);
  camPos -= camTarget;
  camPos += newTarPos;
  renderer.setCamera(&camPos.x, &newTarPos.x);

  actualX = event->x();
  actualY = event->y();

  if((moveDrag || moveViewOfViewDragXY || moveViewOfViewDragXZ || moveViewOfViewDragYZ || moveViewOfViewDragXP ||
      moveViewOfViewDragYP || moveViewOfViewDragZP) && widget.selectedPoint.limb > -1)
  {
    Vector3<> vecFar, vecNear;
    gluUnProjectClick(event->x(), event->y(), vecFar, vecNear);
    Vector3<> planeIntersection;
    Vector3<> p;
    if(!moveViewOfViewDragXY && !moveViewOfViewDragXZ && !moveViewOfViewDragYZ && !moveViewOfViewDragXP && !moveViewOfViewDragYP && !moveViewOfViewDragZP)
    {
      p = parameters.phaseParameters[widget.selectedPoint.phaseNumber].controlPoints[widget.selectedPoint.limb][widget.selectedPoint.pointNumber];

      /* This is not supported at this time
      if (widget.selectedPoint.limb == Phase::leftFootRot)
      {
        if (widget.selectedPoint.xzRot)
        {
          Vector3<> drawPos(0.f,150.f, 0.f);
          drawPos.rotateX(p.x);
          drawPos.rotateZ(p.z);
          p = Vector3<>(parameters.footOrigin.x + drawPos.x,parameters.footOrigin.y + drawPos.y, parameters.footOrigin.z + drawPos.z);
        }
        else
        {
          Vector3<> drawPos2(150.f,0.f, 0.f);
          drawPos2.rotateY(p.y);
          drawPos2.rotateZ(p.z);
          p = Vector3<>(parameters.footOrigin.x + drawPos2.x, parameters.footOrigin.y + drawPos2.y, parameters.footOrigin.z + drawPos2.z);
        }
      }

      if (widget.selectedPoint.limb == Phase::rightFootRot)
      {
        if (widget.selectedPoint.xzRot)
        {
          Vector3<> drawPos(0.f,-150.f, 0.f);
          drawPos.rotateX(p.x);
          drawPos.rotateZ(p.z);
          p = Vector3<>(parameters.footOrigin.x + drawPos.x, -parameters.footOrigin.y + drawPos.y, parameters.footOrigin.z +drawPos.z);
        }
        else
        {
          Vector3<> drawPos2(150.f,0.f,0.f);
          drawPos2.rotateY(p.y);
          drawPos2.rotateZ(p.z);
          p = Vector3<>(parameters.footOrigin.x + drawPos2.x, -parameters.footOrigin.y + drawPos2.y, parameters.footOrigin.z + drawPos2.z);
        }
      }*/

      Vector3<> position((double)p.x, (double)p.y, (double)p.z);
      if(ViewBikeMath::intersectRayAndPlane(vecNear, vecFar, position,
                                            widget.dragPlane, planeIntersection))
      {
        translationVec = planeIntersection - position;
        //this is buggy
        /* if (widget.selectedPoint.limb == Phase::leftFootRot)
         {
           if (widget.selectedPoint.xzRot)
           {
             double temp = -sin((translationVec.x/150.));
             translationVec.x = sin((translationVec.z/150.));
             translationVec.y = 0.;
             translationVec.z = temp;
           }
           else
           {
             double temp = sin((translationVec.y/150.));
             translationVec.x = 0.;
             translationVec.y = -sin((translationVec.z/150.));
             translationVec.z = temp;
           }
         }
         if (widget.selectedPoint.limb == Phase::rightFootRot)
         {
           if (widget.selectedPoint.xzRot)
           {
             double temp = -sin((translationVec.x/-150.));
             translationVec.x = sin((translationVec.z/-150.));
             translationVec.y = 0.;
             translationVec.z = temp;
           }
           else
           {
             double temp = sin((translationVec.y/150.));
             translationVec.x = 0.;
             translationVec.y = -sin((translationVec.z/150.));
             translationVec.z = temp;
           }
         }*/
      }
    }
    else
    {
      unsigned int width, height;
      renderer.getSize(width, height);

      float mini_window_width = width / 3.0f;
      float mini_window_height = (height - 150.0f) / 3.0f;

      float maxXrange = 100, minXrange = -100, maxYrange = 150, minYrange = -150, maxZrange = 100, minZrange = -250;

      for(unsigned int k = 0; k < NUM_OF_POINTS; k++)
      {
        Vector3<> controlPoint = parameters.phaseParameters[widget.selectedPoint.phaseNumber].controlPoints[widget.selectedPoint.limb][k];
        minXrange = std::min<float>(minXrange, controlPoint.x);
        maxXrange = std::max<float>(maxXrange, controlPoint.x);
        minYrange = std::min<float>(minYrange, controlPoint.y);
        maxYrange = std::max<float>(maxYrange, controlPoint.y);
        minZrange = std::min<float>(minZrange, controlPoint.z);
        maxZrange = std::max<float>(maxZrange, controlPoint.z);
      }
      //calculate zoomfactor
      float
      scaleFactorX = mini_window_width / std::max<>(maxXrange - minXrange, 1.0f),
      scaleFactorX1 = mini_window_height / std::max<>(maxXrange - minXrange, 1.0f),
      scaleFactorY = mini_window_height / std::max<>(maxYrange - minYrange, 1.0f),
      scaleFactorY1 = mini_window_width / std::max<>(maxYrange - minYrange, 1.0f),
      scaleFactorZ = mini_window_height / std::max<>(maxZrange - minZrange, 1.0f);

      p = parameters.phaseParameters[widget.selectedPoint.phaseNumber].controlPoints[widget.selectedPoint.limb][widget.selectedPoint.pointNumber];

      Vector3<> windowVec;
      if(moveViewOfViewDragXY)
      {
        windowVec = Vector3<>(((event->x() - (width - 20.f - width / 3.0f)) / scaleFactorX) + minXrange,
                              ((event->y() - mini_window_height - 20.0) / -scaleFactorY) + minYrange, 0.0);
        p.z = 0.f;
      }
      if(moveViewOfViewDragXZ)
      {
        windowVec = Vector3<>(((event->x() - (width - 20.f - width / 3.0f)) / scaleFactorX) + minXrange,
                              0.0, ((event->y() - mini_window_height * 2 - 55.0) / -scaleFactorZ) + minZrange);
        p.y = 0.f;
      }
      if(moveViewOfViewDragYZ)
      {
        windowVec = Vector3<>(0.0, ((event->x() - (width - 20.f - width / 3.0f)) / scaleFactorY1) + minYrange,
                              ((event->y() - mini_window_height * 3 - 90.0) / -scaleFactorZ) + minZrange);
        p.x = 0.f;
      }
      if(moveViewOfViewDragXP)
      {
        windowVec = Vector3<>(((event->y() - mini_window_height - 20.0) / -scaleFactorX1) + minXrange, 0.0, 0.0);
        p.y = 0.f;
        p.z = 0.f;
      }
      if(moveViewOfViewDragYP)
      {
        windowVec = Vector3<>(0.0, ((event->y() - mini_window_height * 2 - 55.0) / -scaleFactorY) + minYrange, 0.0);
        p.x = 0.f;
        p.z = 0.f;
      }
      if(moveViewOfViewDragZP)
      {
        windowVec = Vector3<>(0.0, 0.0, ((event->y() - mini_window_height * 3 - 90.0) / -scaleFactorZ) + minZrange);
        p.y = 0.f;
        p.x = 0.f;
      }

      translationVec = windowVec - Vector3<>(p.x, p.y, p.z);
    }

	parameters.phaseParameters[widget.selectedPoint.phaseNumber].controlPoints[widget.selectedPoint.limb][widget.selectedPoint.pointNumber].x += (float)translationVec.x;
    parameters.phaseParameters[widget.selectedPoint.phaseNumber].controlPoints[widget.selectedPoint.limb][widget.selectedPoint.pointNumber].y += (float)translationVec.y;
    parameters.phaseParameters[widget.selectedPoint.phaseNumber].controlPoints[widget.selectedPoint.limb][widget.selectedPoint.pointNumber].z += (float)translationVec.z;

    if(widget.selectedPoint.limb == Phase::leftFootRot || widget.selectedPoint.limb == Phase::rightFootRot)
    {
      if(parameters.phaseParameters[widget.selectedPoint.phaseNumber].controlPoints[widget.selectedPoint.limb][widget.selectedPoint.pointNumber].x > pi_4)
        parameters.phaseParameters[widget.selectedPoint.phaseNumber].controlPoints[widget.selectedPoint.limb][widget.selectedPoint.pointNumber].x = pi_4;
      if(parameters.phaseParameters[widget.selectedPoint.phaseNumber].controlPoints[widget.selectedPoint.limb][widget.selectedPoint.pointNumber].y > pi_4)
        parameters.phaseParameters[widget.selectedPoint.phaseNumber].controlPoints[widget.selectedPoint.limb][widget.selectedPoint.pointNumber].y = pi_4;
      if(parameters.phaseParameters[widget.selectedPoint.phaseNumber].controlPoints[widget.selectedPoint.limb][widget.selectedPoint.pointNumber].z > pi_4)
        parameters.phaseParameters[widget.selectedPoint.phaseNumber].controlPoints[widget.selectedPoint.limb][widget.selectedPoint.pointNumber].z = pi_4;
      if(parameters.phaseParameters[widget.selectedPoint.phaseNumber].controlPoints[widget.selectedPoint.limb][widget.selectedPoint.pointNumber].x < -pi_4)
        parameters.phaseParameters[widget.selectedPoint.phaseNumber].controlPoints[widget.selectedPoint.limb][widget.selectedPoint.pointNumber].x = -pi_4;
      if(parameters.phaseParameters[widget.selectedPoint.phaseNumber].controlPoints[widget.selectedPoint.limb][widget.selectedPoint.pointNumber].y < -pi_4)
        parameters.phaseParameters[widget.selectedPoint.phaseNumber].controlPoints[widget.selectedPoint.limb][widget.selectedPoint.pointNumber].y = -pi_4;
      if(parameters.phaseParameters[widget.selectedPoint.phaseNumber].controlPoints[widget.selectedPoint.limb][widget.selectedPoint.pointNumber].z < -pi_4)
        parameters.phaseParameters[widget.selectedPoint.phaseNumber].controlPoints[widget.selectedPoint.limb][widget.selectedPoint.pointNumber].z = -pi_4;
    }

    //clipping
    clipCurve(translationVec);

    // äquidistant und kolinear p3-p2 = q1-q0
    //p3 = q0 => p3 - p2 = q1 - p3 => 2*p3 - p2 = q1;
    switch(widget.selectedPoint.pointNumber)
    {
    case 0:
      //p2
      if(widget.selectedPoint.phaseNumber > 0)
      {

        float factor = (float)parameters.phaseParameters[widget.selectedPoint.phaseNumber - 1].duration /
                       (float)parameters.phaseParameters[widget.selectedPoint.phaseNumber].duration;

        parameters.phaseParameters[widget.selectedPoint.phaseNumber - 1].controlPoints[widget.selectedPoint.limb][1] =
          parameters.phaseParameters[widget.selectedPoint.phaseNumber - 1].controlPoints[widget.selectedPoint.limb][2] -
          parameters.phaseParameters[widget.selectedPoint.phaseNumber].controlPoints[widget.selectedPoint.limb][0];

        parameters.phaseParameters[widget.selectedPoint.phaseNumber - 1].controlPoints[widget.selectedPoint.limb][1] *= factor;

        parameters.phaseParameters[widget.selectedPoint.phaseNumber - 1].controlPoints[widget.selectedPoint.limb][1] +=
          parameters.phaseParameters[widget.selectedPoint.phaseNumber - 1].controlPoints[widget.selectedPoint.limb][2];
      }
      break;
    case 1:
      //q1
      if(widget.selectedPoint.phaseNumber <  parameters.numberOfPhases - 1)
      {

        parameters.phaseParameters[widget.selectedPoint.phaseNumber + 1].controlPoints[widget.selectedPoint.limb][0] =
          parameters.phaseParameters[widget.selectedPoint.phaseNumber].controlPoints[widget.selectedPoint.limb][2] -
          parameters.phaseParameters[widget.selectedPoint.phaseNumber].controlPoints[widget.selectedPoint.limb][1];

        float factor = (float)parameters.phaseParameters[widget.selectedPoint.phaseNumber + 1].duration /
                       (float)parameters.phaseParameters[widget.selectedPoint.phaseNumber].duration;
        parameters.phaseParameters[widget.selectedPoint.phaseNumber + 1].controlPoints[widget.selectedPoint.limb][0] *= factor;

        parameters.phaseParameters[widget.selectedPoint.phaseNumber + 1].controlPoints[widget.selectedPoint.limb][0] +=
          parameters.phaseParameters[widget.selectedPoint.phaseNumber].controlPoints[widget.selectedPoint.limb][2];
      }
      break;
    case 2:
      //p2
      parameters.phaseParameters[widget.selectedPoint.phaseNumber].controlPoints[widget.selectedPoint.limb][1].x += (float)translationVec.x;
      parameters.phaseParameters[widget.selectedPoint.phaseNumber].controlPoints[widget.selectedPoint.limb][1].y += (float)translationVec.y;
      parameters.phaseParameters[widget.selectedPoint.phaseNumber].controlPoints[widget.selectedPoint.limb][1].z += (float)translationVec.z;
      //q1
      if(widget.selectedPoint.phaseNumber <  parameters.numberOfPhases - 1)
      {
        parameters.phaseParameters[widget.selectedPoint.phaseNumber + 1].controlPoints[widget.selectedPoint.limb][0].x += (float)translationVec.x;
        parameters.phaseParameters[widget.selectedPoint.phaseNumber + 1].controlPoints[widget.selectedPoint.limb][0].y += (float)translationVec.y;
        parameters.phaseParameters[widget.selectedPoint.phaseNumber + 1].controlPoints[widget.selectedPoint.limb][0].z += (float)translationVec.z;
      }
      break;
    default:
      break;
    }
    parameters.initFirstPhase();
    if(widget.selectedPoint.phaseNumber > -1)
      widget.fillModelWithPhaseData(widget.selectedPoint.phaseNumber);
    event->accept();
    if(widget.followMode)
    {
      widget.playMotion(widget.tabber->currentIndex());
    }
    updateGL();
  }
  else
  {
    if(renderer.moveDrag(event->x(), event->y()))
    {
      event->accept();
      if(widget.selectedPoint.phaseNumber > -1)
        widget.fillModelWithPhaseData(widget.selectedPoint.phaseNumber);
      updateGL();
    }
  }
}

void ViewBikeGLWidget::showPlane()
{
  Vector3<> p1, p2, p3, p4;
  Vector3<> zero1, zero2, zero3, zero4;

  Vector3<> point = parameters.phaseParameters[widget.selectedPoint.phaseNumber].controlPoints[widget.selectedPoint.limb][widget.selectedPoint.pointNumber];

  /* if (widget.selectedPoint.limb == Phase::leftFootRot)
   {
     if (widget.selectedPoint.xzRot)
     {
       Vector3<> drawPos(0.f,150.f, 0.f);
       drawPos.rotateX(point.x);
       drawPos.rotateZ(point.z);
       point = Vector3<>(parameters.footOrigin.x + drawPos.x,parameters.footOrigin.y + drawPos.y, parameters.footOrigin.z + drawPos.z);
     }
     else
     {
       Vector3<> drawPos2(150.f,0.f, 0.f);
       drawPos2.rotateY(point.y);
       drawPos2.rotateZ(point.z);
       point = Vector3<>(parameters.footOrigin.x + drawPos2.x, parameters.footOrigin.y + drawPos2.y, parameters.footOrigin.z + drawPos2.z);
     }
   }

   if (widget.selectedPoint.limb == Phase::rightFootRot)
   {
     if (widget.selectedPoint.xzRot)
     {
       Vector3<> drawPos(0.f,-150.f, 0.f);
       drawPos.rotateX(point.x);
       drawPos.rotateZ(point.z);
       point = Vector3<>(parameters.footOrigin.x + drawPos.x, -parameters.footOrigin.y + drawPos.y, parameters.footOrigin.z +drawPos.z);
     }
     else
     {
       Vector3<> drawPos2(150.f,0.f,0.f);
       drawPos2.rotateY(point.y);
       drawPos2.rotateZ(point.z);
       point = Vector3<>(parameters.footOrigin.x + drawPos2.x, -parameters.footOrigin.y + drawPos2.y, parameters.footOrigin.z + drawPos2.z);
     }
   }*/

  if(widget.dragPlane.x == 1)  //YZ
  {
    p1 = Vector3<>(point.x, -300.f, -300.f);
    p2 = Vector3<>(point.x, 300.f, -300.f);
    p3 = Vector3<>(point.x, 300.f, 300.f);
    p4 = Vector3<>(point.x, -300.f, 300.f);
    zero1 = Vector3<>(point.x, 0.f, -300.f);
    zero2 = Vector3<>(point.x, 0.f, 300.f);
    zero3 = Vector3<>(point.x, -300.f, 0.f);
    zero4 = Vector3<>(point.x, 300.f, 0.f);

  }
  else if(widget.dragPlane.y == 1)  //XZ
  {
    p1 = Vector3<>(-300.f, point.y, -300.f);
    p2 = Vector3<>(300.f, point.y, -300.f);
    p3 = Vector3<>(300.f, point.y, 300.f);
    p4 = Vector3<>(-300.f, point.y, 300.f);
    zero1 = Vector3<>(0.f, point.y, -300.f);
    zero2 = Vector3<>(0.f, point.y, 300.f);
    zero3 = Vector3<>(-300.f, point.y,  0.f);
    zero4 = Vector3<>(300.f, point.y, 0.f);
  }
  else if(widget.dragPlane.z == 1)  //XY
  {
    p1 = Vector3<>(-300.f, -300.f, point.z);
    p2 = Vector3<>(300.f, -300.f, point.z);
    p3 = Vector3<>(300.f, 300.f, point.z);
    p4 = Vector3<>(-300.f, 300.f, point.z);
    zero1 = Vector3<>(0.f, -300.f, point.z);
    zero2 = Vector3<>(0.f, 300.f, point.z);
    zero3 = Vector3<>(-300.f, 0.f, point.z);
    zero4 = Vector3<>(300.f, 0.f, point.z);
  }

  glPushMatrix();
  setMatrix(viewBike.robot->getPosition(), originRot);
  glScaled(0.001, 0.001, 0.001);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_BLEND);
  glColor4f(0.f, 0.5f, 0.12f, 0.6f);
  glBegin(GL_QUADS);
  glVertex3f(p1.x, p1.y, p1.z);
  glVertex3f(p2.x, p2.y, p2.z);
  glVertex3f(p3.x, p3.y, p3.z);
  glVertex3f(p4.x, p4.y, p4.z);
  glEnd();
  glColor4f(1.f, 1.f, 1.f, 1.f);

  float lw[1];
  glGetFloatv(GL_LINE_WIDTH, lw);
  glLineWidth(4);
  glBegin(GL_LINES);
  glVertex3f(zero1.x, zero1.y, zero1.z);
  glVertex3f(zero2.x, zero2.y, zero2.z);
  glEnd();
  glBegin(GL_LINES);
  glVertex3f(zero3.x, zero3.y, zero3.z);
  glVertex3f(zero4.x, zero4.y, zero4.z);
  glEnd();
  glLineWidth(lw[0]);
  glDisable(GL_BLEND);
  glPopMatrix();
  glFlush();
}

void ViewBikeGLWidget::setSteadyDiff()
{

  for(int phaseNumber = 0; phaseNumber < parameters.numberOfPhases - 1; phaseNumber++)
  {
    float factor = (float)parameters.phaseParameters[phaseNumber + 1].duration / (float)parameters.phaseParameters[phaseNumber].duration;

    for(int j = 0; j < Phase::numOfLimbs; j++)
    {

      Vector3<> checkConstant =  parameters.phaseParameters[phaseNumber + 1].controlPoints[j][0];

      parameters.phaseParameters[phaseNumber + 1].controlPoints[j][0] =
        parameters.phaseParameters[phaseNumber].controlPoints[j][2] -
        parameters.phaseParameters[phaseNumber].controlPoints[j][1];
      //add the time
      // dt2/dt1 * (P3-P2)+Q0; mit Q0 = P3
      parameters.phaseParameters[phaseNumber + 1].controlPoints[j][0] *= factor;

      parameters.phaseParameters[phaseNumber + 1].controlPoints[j][0] += parameters.phaseParameters[phaseNumber].controlPoints[j][2];

      if(checkConstant.x == parameters.phaseParameters[phaseNumber + 1].controlPoints[j][2].x &&
         checkConstant.y == parameters.phaseParameters[phaseNumber + 1].controlPoints[j][2].y &&
         checkConstant.z == parameters.phaseParameters[phaseNumber + 1].controlPoints[j][2].z)
      {
        parameters.phaseParameters[phaseNumber + 1].controlPoints[j][1] =  parameters.phaseParameters[phaseNumber + 1].controlPoints[j][0];
        parameters.phaseParameters[phaseNumber + 1].controlPoints[j][2] =  parameters.phaseParameters[phaseNumber + 1].controlPoints[j][0];
      }
    }
  }
}

void ViewBikeGLWidget::clipCurve(Vector3<>& translationVec)
{
  Vector3<> positions[2];
  parameters.initFirstPhase();
  for(float t = 0.f; t < 1.01f; t += 0.01f)
  {
    float leg0, leg1, leg2, leg3, leg4, leg5;

    if(widget.selectedPoint.limb == Phase::leftFootTra)
    {

      positions[0] = parameters.getPositionBlame(t, widget.selectedPoint.phaseNumber, Phase::leftFootTra);
      positions[1] = parameters.getPositionBlame(t, widget.selectedPoint.phaseNumber, Phase::leftFootRot);

      bool leftLegTooShort = ViewBikeMath::calcLegJoints(positions[0], positions[1], true, viewBike.robotDimensions, leg0, leg1, leg2, leg3, leg4, leg5);

      if(clipLegJointsWithLimits(leg1, leg2, leg3, JointData::LHipYawPitch) || leftLegTooShort)
      {
        parameters.phaseParameters[widget.selectedPoint.phaseNumber].controlPoints[widget.selectedPoint.limb][widget.selectedPoint.pointNumber].x -= (float)translationVec.x;
        parameters.phaseParameters[widget.selectedPoint.phaseNumber].controlPoints[widget.selectedPoint.limb][widget.selectedPoint.pointNumber].y -= (float)translationVec.y;
        parameters.phaseParameters[widget.selectedPoint.phaseNumber].controlPoints[widget.selectedPoint.limb][widget.selectedPoint.pointNumber].z -= (float)translationVec.z;
        translationVec = Vector3<>(0., 0., 0.);
      }
    }
    if(widget.selectedPoint.limb == Phase::rightFootTra)
    {

      positions[0] = parameters.getPositionBlame(t, widget.selectedPoint.phaseNumber, Phase::rightFootTra);
      positions[1] = parameters.getPositionBlame(t, widget.selectedPoint.phaseNumber, Phase::rightFootRot);

      bool rightLegTooShort = ViewBikeMath::calcLegJoints(positions[0], positions[1], false, viewBike.robotDimensions, leg0, leg1, leg2, leg3, leg4, leg5);
      if(clipLegJointsWithLimits(leg1, leg2, leg3, JointData::RHipYawPitch) || rightLegTooShort)
      {
        parameters.phaseParameters[widget.selectedPoint.phaseNumber].controlPoints[widget.selectedPoint.limb][widget.selectedPoint.pointNumber].x -= (float)translationVec.x;
        parameters.phaseParameters[widget.selectedPoint.phaseNumber].controlPoints[widget.selectedPoint.limb][widget.selectedPoint.pointNumber].y -= (float)translationVec.y;
        parameters.phaseParameters[widget.selectedPoint.phaseNumber].controlPoints[widget.selectedPoint.limb][widget.selectedPoint.pointNumber].z -= (float)translationVec.z;
        translationVec = Vector3<>(0., 0., 0.);
      }
    }
  }
}

bool ViewBikeGLWidget::clipLegJointsWithLimits(float& leg1, float& leg2, float& leg3, const JointData::Joint& joint)
{
  bool clipped = false;

  //clip Leg1
  if(viewBike.jointCalibration.joints[joint + 1].maxAngle < leg1)
  {
    leg1 = viewBike.jointCalibration.joints[joint + 1].maxAngle;
    clipped = true;
  }
  if(viewBike.jointCalibration.joints[joint + 1].minAngle > leg1)
  {
    leg1 = viewBike.jointCalibration.joints[joint + 1].minAngle;
    clipped = true;
  }

  //clip Leg2
  if(viewBike.jointCalibration.joints[joint + 2].maxAngle < leg2)
  {
    leg2 = viewBike.jointCalibration.joints[joint + 2].maxAngle;
    clipped = true;
  }
  if(viewBike.jointCalibration.joints[joint + 2].minAngle > leg2)
  {
    leg2 = viewBike.jointCalibration.joints[joint + 2].minAngle;
    clipped = true;
  }

  //clip Leg3
  if(viewBike.jointCalibration.joints[joint + 3].maxAngle < leg3)
  {
    leg3 = viewBike.jointCalibration.joints[joint + 3].maxAngle;
    clipped = true;
  }
  if(viewBike.jointCalibration.joints[joint + 3].minAngle > leg3)
  {
    leg3 = viewBike.jointCalibration.joints[joint + 3].minAngle;
    clipped = true;
  }
  return clipped;
}


void ViewBikeGLWidget::mousePressEvent(QMouseEvent* event)
{
  QWidget::mousePressEvent(event);

  if(event->button() == Qt::LeftButton)
  {
    if(clickControlPoint(event->x(), event->y()))
    {
      widget.parent->addStateToUndoList();
      actualX = event->x();
      actualY = event->y();
      event->accept();
      updateGL();
    }
    if(renderer.startDrag(event->x(), event->y(), QApplication::keyboardModifiers() & Qt::ShiftModifier ? SimRobotCore2::Renderer::dragRotate : SimRobotCore2::Renderer::dragNormal))
    {
      event->accept();
      updateGL();
    }
  }

  if(event->button() == Qt::RightButton)
  {
    actualX = event->x();
    actualY = event->y();
    event->accept();
  }
}

void ViewBikeGLWidget::mouseReleaseEvent(QMouseEvent* event)
{
  QWidget::mouseReleaseEvent(event);

  if(moveDrag || moveViewOfViewDragXY || moveViewOfViewDragXZ || moveViewOfViewDragYZ || moveViewOfViewDragXP || moveViewOfViewDragYP || moveViewOfViewDragZP)
  {
    moveDrag = false;
    moveViewOfViewDragXY = false;
    moveViewOfViewDragXZ = false;
    moveViewOfViewDragYZ = false;
    moveViewOfViewDragXP = false;
    moveViewOfViewDragYP = false;
    moveViewOfViewDragZP = false;
    event->accept();

    updateGL();
  }
  else
  {
    if(renderer.releaseDrag(event->x(), event->y()))
    {
      event->accept();
      updateGL();
    }
  }
}

void ViewBikeGLWidget::contextMenuEvent(QContextMenuEvent* event)
{
  event->accept();
  QMenu menu;
  menu.addMenu(widget.bikeMenuBar->dragPlaneMenu);
  menu.addSeparator();
  actualX = event->x();
  actualY = event->y();
  menu.exec(mapToGlobal(QPoint(event->x(), event->y())));
}
