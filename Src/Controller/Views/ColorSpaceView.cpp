/**
* @file Controller/Views/ColorSpaceView.cpp
*
* Implementation of class ColorSpaceView
*
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#include "Platform/Thread.h"
#include "ColorSpaceView.h"
#include "Controller/RobotConsole.h"
#include "Controller/Visualization/OpenGLMethods.h"
#ifdef MACOSX
#include <gl.h>
#else
#include <GL/gl.h>
#endif

ColorSpaceView::ColorSpaceView(const QString& fullName, RobotConsole& c, const std::string& n, ColorModel cm, int ch, const Vector3<>& b)
  : View3D(fullName, b),
    console(c),
    name(n),
    colorModel(cm),
    channel(ch),
    lastTimeStamp(0)
{
}

void ColorSpaceView::updateDisplayLists()
{
  SYNC_WITH(console);
  Image* image = 0,
       * raw = 0;
  RobotConsole::Images::const_iterator i = console.images.find(name);
  if(i != console.images.end())
    image = i->second.image;
  i = console.images.find("raw image");
  if(i != console.images.end())
    raw = i->second.image;
  if(image && (channel < 3 || raw))
  {
    if(!channel)
      OpenGLMethods::paintCubeToOpenGLList(256, 256, 256,
                                           cubeId, true,
                                           127, //scale
                                           -127, -127, -127, // offsets
                                           int(background.x * 255) ^ 0xc0,
                                           int(background.y * 255) ^ 0xc0,
                                           int(background.z * 255) ^ 0xc0);
    else
      OpenGLMethods::paintCubeToOpenGLList(
        image->resolutionWidth, image->resolutionHeight, 128,
        cubeId, true,
        127, //scale
        -image->resolutionWidth / 2, -image->resolutionHeight / 2, -65, // offsets
        int(background.x * 255) ^ 0xc0,
        int(background.y * 255) ^ 0xc0,
        int(background.z * 255) ^ 0xc0);

    OpenGLMethods::paintImagePixelsToOpenGLList(*image, colorModel, channel - 1, false, colorsId);
    lastTimeStamp = image->timeStamp;
  }
  else
  {
    glNewList(cubeId, GL_COMPILE_AND_EXECUTE);
    glEndList();
    glNewList(colorsId, GL_COMPILE_AND_EXECUTE);
    glEndList();
    lastTimeStamp = 0;
  }
}

bool ColorSpaceView::needsUpdate() const
{
  SYNC_WITH(console);
  Image* image = 0;
  RobotConsole::Images::const_iterator i = console.images.find(name);
  if(i != console.images.end())
    image = i->second.image;
  return ((image && image->timeStamp != lastTimeStamp) ||
          (!image && lastTimeStamp));
}

