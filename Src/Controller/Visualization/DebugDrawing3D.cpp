/**
* @file Controller/Visualization/DebugDrawing3D.cpp
* Implementation of class DebugDrawing3D.
*
* @author Philippe Schober
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
*/

#include "Controller/RobotConsole.h"
#ifdef MACOSX
#include <gl.h>
#include <glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif
#include "DebugDrawing3D.h"
#include "Platform/BHAssert.h"
#include "Platform/SystemCall.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/RobotDimensions.h"


DebugDrawing3D::DebugDrawing3D() : flip(false), robotConsole(0)
{
  timeStamp = SystemCall::getCurrentSystemTime();
  drawn = false;
  scaleX  = scaleY  = scaleZ  = 1.0f;
  rotateX = rotateY = rotateZ = 0.0f;
  transX  = transY  = transZ  = 0.0f;
}

const DebugDrawing3D& DebugDrawing3D::operator=(const DebugDrawing3D& other)
{
  reset();
  timeStamp = other.timeStamp;
  drawn = other.drawn;
  flip = other.flip;
  scaleX = other.scaleX;
  scaleY = other.scaleY;
  scaleZ = other.scaleZ;
  rotateX = other.rotateX;
  rotateY = other.rotateY;
  rotateZ = other.rotateZ;
  transX = other.transX;
  transY = other.transY;
  transZ = other.transZ;
  processIdentifier = other.processIdentifier;
  robotConsole  = other.robotConsole;
  return *this += other;
}

const DebugDrawing3D& DebugDrawing3D::operator+=(const DebugDrawing3D& other)
{
  lines.insert(lines.end(), other.lines.begin(), other.lines.end());
  dots.insert(dots.end(), other.dots.begin(), other.dots.end());
  polygons.insert(polygons.end(), other.polygons.begin(), other.polygons.end());
  quads.insert(quads.end(), other.quads.begin(), other.quads.end());
  spheres.insert(spheres.end(), other.spheres.begin(), other.spheres.end());
  cylinders.insert(cylinders.end(), other.cylinders.begin(), other.cylinders.end());
  images.insert(images.end(), other.images.begin(), other.images.end());
  return *this;
}

DebugDrawing3D::DebugDrawing3D(const DebugDrawing3D& other)
{
  *this = other;
}

DebugDrawing3D::DebugDrawing3D(const DebugDrawing3D* pDebugDrawing3D)
{
  *this = *pDebugDrawing3D;
}

DebugDrawing3D::~DebugDrawing3D()
{
}

void DebugDrawing3D::draw()
{
  if(robotConsole)
  {
    SYNC_WITH(*robotConsole);
    draw2();
  }
  else
    draw2();
}

void DebugDrawing3D::draw2()
{
  glPushAttrib(GL_ENABLE_BIT | GL_LIGHTING_BIT);
  glPushMatrix();

  if(flip)
    glRotated(180.f, 0, 0, 1);

  // Convert mm to m.
  glScaled(0.001, 0.001, 0.001);

  // Custom scaling.
  glScaled(scaleX, scaleY, scaleZ);

  // Custom rotation.
  if(rotateX != 0) glRotated(toDegrees(rotateX), 1, 0, 0);
  if(rotateY != 0) glRotated(toDegrees(rotateY), 0, 1, 0);
  if(rotateZ != 0) glRotated(toDegrees(rotateZ), 0, 0, 1);

  // Custom translation.
  glTranslated(transX, transY, transZ);

  //
  //glDisable(GL_LIGHTING);
  glEnable(GL_NORMALIZE);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  // Draw all polygons/triangles.
  std::vector<Polygon>::iterator p;
  for(p = polygons.begin(); p != polygons.end(); ++p)
  {
    glColor4ub(p->color.r, p->color.g, p->color.b, p->color.a);
    glBegin(GL_TRIANGLES);
    glVertex3d(p->points[0].x, p->points[0].y, p->points[0].z);
    glVertex3d(p->points[1].x, p->points[1].y, p->points[1].z);
    glVertex3d(p->points[2].x, p->points[2].y, p->points[2].z);
    glEnd();
  }

  // Draw all lines.
  if(!lines.empty())
  {
    glPushAttrib(GL_LINE_BIT);
    std::vector<Line>::iterator l;
    for(l = lines.begin(); l != lines.end(); ++l)
    {
      glLineWidth(l->width);
      glColor4ub(l->color.r, l->color.g, l->color.b, l->color.a);
      glBegin(GL_LINES);
      glVertex3d(l->points[0].x, l->points[0].y, l->points[0].z);
      glVertex3d(l->points[1].x, l->points[1].y, l->points[1].z);
      glEnd();
    }
    glPopAttrib();
  }

  // Draw all points.
  if(!dots.empty())
  {
    glPushAttrib(GL_POINT_BIT);
    std::vector<Dot>::iterator d;
    for(d = dots.begin(); d != dots.end(); ++d)
    {
      // Since each point may have a different size we can't handle all
      // points in a single glBegin(GL_POINTS).
      // ( glPointSize is not allowed in a glBegin(...). )
      glPointSize(d->width);
      glColor4ub(d->color.r, d->color.g, d->color.b, d->color.a);
      glBegin(GL_POINTS);
      glVertex3d(d->point.x, d->point.y, d->point.z);
      glEnd();
    }
    glPopAttrib();
  }

  // draw spheres
  std::vector<Sphere>::iterator s;
  for(s = spheres.begin(); s != spheres.end(); ++s)
  {
    glColor4ub(s->color.r, s->color.g, s->color.b, s->color.a);
    glPushMatrix();
    glTranslated(s->point.x, s->point.y, s->point.z);
    GLUquadric* q = gluNewQuadric();
    gluSphere(q, s->radius, 16, 16);
    gluDeleteQuadric(q);
    glPopMatrix();
  }

  // draw cylinders
  std::vector<Cylinder>::iterator c;
  for(c = cylinders.begin(); c != cylinders.end(); ++c)
  {
    glColor4ub(c->color.r, c->color.g, c->color.b, c->color.a);

    glPushMatrix();
    glTranslated(c->point.x, c->point.y, c->point.z);
    if(c->rotation.x != 0)
      glRotated(toDegrees(c->rotation.x), 1, 0, 0);
    if(c->rotation.y != 0)
      glRotated(toDegrees(c->rotation.y), 0, 1, 0);
    if(c->rotation.z != 0)
      glRotated(toDegrees(c->rotation.z), 0, 0, 1);
    glTranslated(0, 0, -c->height / 2);
    GLUquadric* q = gluNewQuadric();
    gluCylinder(q, c->baseRadius, c->topRadius, c->height, 16, 1);
    glRotated(180, 0, 1, 0);
    if(c->baseRadius > 0.f)
      gluDisk(q, 0, c->baseRadius, 16, 1);
    glRotated(180, 0, 1, 0);
    glTranslated(0, 0, c->height);
    if(c->topRadius > 0.f)
      gluDisk(q, 0, c->topRadius, 16, 1);
    gluDeleteQuadric(q);
    glPopMatrix();
  }

  // Draw all quads.
  std::vector<Quad>::iterator q;
  for(q = quads.begin(); q != quads.end(); ++q)
  {
    glColor4ub(q->color.r, q->color.g, q->color.b, q->color.a);
    glBegin(GL_QUADS);

    const Vector3<>& p1 = q->points[0];
    const Vector3<>& p2 = q->points[1];
    const Vector3<>& p3 = q->points[2];
    const Vector3<>& p4 = q->points[3];
    Vector3<> u(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
    Vector3<> v(p3.x - p1.x, p3.y - p1.y, p3.z - p1.z);
    Vector3<> n(u.y * v.z - u.z * v.y, u.z * v.x - u.x * v.z, u.x * v.y - u.y * v.x);
    n.normalize();

    glNormal3fv(&n.x);
    glVertex3fv(&p1.x);
    glVertex3fv(&p2.x);
    glVertex3fv(&p3.x);
    glVertex3fv(&p4.x);
    glEnd();
  }

  // draw 3d images
  if(!images.empty())
  {
    glPushAttrib(GL_TEXTURE_BIT);
    glDisable(GL_CULL_FACE);
    glEnable(GL_TEXTURE_2D);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    std::vector<Image3D>::iterator i;
    for(i = images.begin(); i != images.end(); ++i)
    {
      GLuint t;
      glGenTextures(1, &t);
      glBindTexture(GL_TEXTURE_2D, t);

      int width, height;
      char* imageData = copyImage(*i->image, width, height);
      glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
                   width, height,
                   0, GL_RGB, GL_UNSIGNED_BYTE, imageData);

      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
      delete [] imageData;

      glPushMatrix();
      glTranslated(i->point.x, i->point.y, i->point.z);
      if(i->rotation.x != 0)
        glRotated(toDegrees(i->rotation.x), 1, 0, 0);
      if(i->rotation.y != 0)
        glRotated(toDegrees(i->rotation.y), 0, 1, 0);
      if(i->rotation.z != 0)
        glRotated(toDegrees(i->rotation.z), 0, 0, 1);
      glBegin(GL_QUADS);
      float right = (float) i->image->resolutionWidth / width;
      float top = (float) i->image->resolutionHeight / height;
      glTexCoord2d(right, top);
      glVertex3d(0, -i->width / 2, i->height / 2);
      glTexCoord2d(0, top);
      glVertex3d(0, i->width / 2, i->height / 2);
      glTexCoord2d(0, 0);
      glVertex3d(0, i->width / 2, -i->height / 2);
      glTexCoord2d(right, 0);
      glVertex3d(0, -i->width / 2, -i->height / 2);
      glEnd();
      glPopMatrix();
      glDeleteTextures(1, &t);
    }
    glPopAttrib();
  }

  //
  glPopAttrib();
  glPopMatrix();
}


void DebugDrawing3D::reset()
{
  timeStamp = SystemCall::getCurrentSystemTime();
  lines.clear();
  dots.clear();
  polygons.clear();
  quads.clear();
  spheres.clear();
  cylinders.clear();
  images.clear();
}

void DebugDrawing3D::quad(const Vector3<>* points, float width, ColorRGBA color)
{
  Quad element;
  element.points[0] = points[0];
  element.points[1] = points[1];
  element.points[2] = points[2];
  element.points[3] = points[3];
  element.color = color;
  element.width = width;
  quads.push_back(element);
}


void DebugDrawing3D::line
(
  float xStart, float yStart, float zStart,
  float xEnd,   float yEnd,   float zEnd,
  float width,   ColorRGBA color
)
{
  Line element;
  element.points[0] = Vector3<float>(xStart, yStart, zStart);
  element.points[1] = Vector3<float>(xEnd,   yEnd,   zEnd);
  element.color = color;
  element.width = width;
  lines.push_back(element);
}

void DebugDrawing3D::line(Vector3<> *points, float width, ColorRGBA color)
{
  Line element;
  element.points[0] = points[0];
  element.points[1] = points[1];
  element.color = color;
  element.width = width;
  lines.push_back(element);
}

void DebugDrawing3D::line
(
  float xStart,
  float yStart,
  float zStart,
  float xEnd,
  float yEnd,
  float zEnd
)
{
  line(xStart, yStart, zStart, xEnd, yEnd, zEnd, 1, ColorRGBA(0, 0, 0));
}

void DebugDrawing3D::polygon
(
  const Vector3<>* points,
  float width,
  ColorRGBA color
)
{
  Polygon element;
  for(int i = 0; i < 3; i++)
    element.points[i] = points[i];
  element.width = width;
  element.color = color;
  polygons.push_back(element);
}

void DebugDrawing3D::dot
(
  Vector3<> v,
  float  w,
  ColorRGBA color
)
{
  Dot element;
  element.point = v;
  element.color = color;
  element.width = w;
  dots.push_back(element);
}

void DebugDrawing3D::sphere
(
  Vector3<> v,
  float r,
  ColorRGBA color
)
{
  Sphere element;
  element.point = v;
  element.color = color;
  element.radius = r;
  spheres.push_back(element);
}

void DebugDrawing3D::cylinder
(
  Vector3<> v,
  Vector3<> rot,
  float baseRadius,
  float topRadius,
  float h,
  ColorRGBA color
)
{
  Cylinder element;
  element.point = v;
  element.rotation = rot;
  element.color = color;
  element.baseRadius = baseRadius;
  element.topRadius = topRadius;
  element.height = h;
  cylinders.push_back(element);
}

void DebugDrawing3D::image
(
  Vector3<> v,
  Vector3<> rot,
  float w,
  float h,
  Image* i
)
{
  Image3D element;
  element.point = v;
  element.rotation = rot;
  element.image = i;
  element.width = w;
  element.height = h;
  images.push_back(element);
}

bool DebugDrawing3D::addShapeFromQueue
(
  InMessage& message,
  Drawings3D::ShapeType shapeType,
  char identifier
)
{
  processIdentifier = identifier;

  switch((Drawings3D::ShapeType)shapeType)
  {
  case Drawings3D::translate:
  {
    message.bin >> transX;
    message.bin >> transY;
    message.bin >> transZ;
  }
  break;
  case Drawings3D::scale:
  {
    message.bin >> scaleX;
    message.bin >> scaleY;
    message.bin >> scaleZ;
  }
  break;
  case Drawings3D::rotate:
  {
    message.bin >> rotateX;
    message.bin >> rotateY;
    message.bin >> rotateZ;
  }
  break;
  case Drawings3D::coordinates:
  {
    float width;
    float length;
    message.bin >> length;
    message.bin >> width;
    this->line(0, 0, 0, length, 0, 0, width, ColorRGBA(255, 0, 0));
    this->line(0, 0, 0, 0, length, 0, width, ColorRGBA(0, 255, 0));
    this->line(0, 0, 0, 0, 0, length, width, ColorRGBA(0, 0, 255));
  }
  break;
  case Drawings3D::quad:
  {
    Vector3<> points[4];
    ColorRGBA c;
    message.bin >> points[0];
    message.bin >> points[1];
    message.bin >> points[2];
    message.bin >> points[3];
    message.bin >> c;
    this->quad(points, 1.0f, c);
  }
  break;
  case Drawings3D::polygon:
  {
    Vector3<> points[3];
    ColorRGBA c;
    message.bin >> points[0];
    message.bin >> points[1];
    message.bin >> points[2];
    message.bin >> c;
    this->polygon(points, 1.0f, c);
  }
  break;
  case Drawings3D::line:
  {
    Vector3<> points[2];
    float width;
    ColorRGBA c;
    message.bin >> points[0];
    message.bin >> points[1];
    message.bin >> width;
    message.bin >> c;
    this->line(points, width, c);
  }
  break;
  case Drawings3D::cube:
  {
    Vector3<> points[8];
    float width;
    ColorRGBA c;
    for(int i = 0; i < 8; i++)
    {
      message.bin >> points[i];
    }
    message.bin >> width;
    message.bin >> c;
    this->line(points[0].x, points[0].y, points[0].z, points[1].x, points[1].y, points[1].z, width, c); //AB
    this->line(points[0].x, points[0].y, points[0].z, points[2].x, points[2].y, points[2].z, width, c); //AC
    this->line(points[0].x, points[0].y, points[0].z, points[4].x, points[4].y, points[4].z, width, c); //AE
    this->line(points[1].x, points[1].y, points[1].z, points[3].x, points[3].y, points[3].z, width, c); //BD
    this->line(points[1].x, points[1].y, points[1].z, points[5].x, points[5].y, points[5].z, width, c); //BF
    this->line(points[2].x, points[2].y, points[2].z, points[3].x, points[3].y, points[3].z, width, c); //CD
    this->line(points[2].x, points[2].y, points[2].z, points[6].x, points[6].y, points[6].z, width, c); //CG
    this->line(points[3].x, points[3].y, points[3].z, points[7].x, points[7].y, points[7].z, width, c); //DH
    this->line(points[4].x, points[4].y, points[4].z, points[6].x, points[6].y, points[6].z, width, c); //EG
    this->line(points[4].x, points[4].y, points[4].z, points[5].x, points[5].y, points[5].z, width, c); //EF
    this->line(points[5].x, points[5].y, points[5].z, points[7].x, points[7].y, points[7].z, width, c); //FH
    this->line(points[6].x, points[6].y, points[6].z, points[7].x, points[7].y, points[7].z, width, c); //GH
  }
  break;
  case Drawings3D::dot:
  {
    Vector3<> v;
    float w;
    ColorRGBA c;
    bool withLines;
    message.bin >> v;
    message.bin >> w;
    message.bin >> c;
    message.bin >> withLines;
    this->dot(v, w, c);

    if(withLines)
    {
      this->line(0,  0,  0,   v.x,  0,  0, 1.0f, c);
      this->line(0,  0,  0,     0, v.y,  0, 1.0f, c);
      this->line(0,  0,  0,     0,  0, v.z, 1.0f, c);
      this->line(v.x,  0,  0,   v.x, v.y,  0, 1.0f, c);
      this->line(v.x,  0,  0,   v.x,  0, v.z, 1.0f, c);
      this->line(0, v.y,  0,     0, v.y, v.z, 1.0f, c);
      this->line(v.x, v.y,  0,     0, v.y,  0, 1.0f, c);
      this->line(v.x, v.y,  0,   v.x, v.y, v.z, 1.0f, c);
      this->line(v.x,  0, v.z,   v.x, v.y, v.z, 1.0f, c);
      this->line(v.x,  0, v.z,     0,  0, v.z, 1.0f, c);
      this->line(0, v.y, v.z,     0,  0, v.z, 1.0f, c);
      this->line(0, v.y, v.z,   v.x, v.y, v.z, 1.0f, c);
    }

  }
  break;
  case Drawings3D::sphere:
  {
    Vector3<> v;
    float r;
    ColorRGBA c;
    message.bin >> v;
    message.bin >> r;
    message.bin >> c;
    this->sphere(v, r, c);
  }
  break;
  case Drawings3D::cylinder:
  {
    Vector3<> v,
            rot;
    float r, r2, h;
    ColorRGBA c;
    message.bin >> v;
    message.bin >> rot;
    message.bin >> r;
    message.bin >> r2;
    message.bin >> h;
    message.bin >> c;
    this->cylinder(v, rot, r, r2, h, c);
  }
  break;
  case Drawings3D::image:
  {
    Vector3<> v,
            rot;
    float w,
          h;
    Image* i = new Image;
    message.bin >> v;
    message.bin >> rot;
    message.bin >> w;
    message.bin >> h;
    message.bin >> *i;
    this->image(v, rot, w, h, i);
  }
  break;
  }
  return true;
}

char* DebugDrawing3D::copyImage(const Image& srcImage, int& width, int& height) const
{
  width = 1;
  while(width < srcImage.resolutionWidth)
    width <<= 1;
  height = 1;
  while(height < srcImage.resolutionHeight)
    height <<= 1;

  static const int factor1 = 29016;
  static const int factor2 = 5662;
  static const int factor3 = 22972;
  static const int factor4 = 11706;
  char* imageData = new char[width * height * 3];
  int r, g, b;
  int yImage, uImage, vImage;
  for(int y = srcImage.resolutionHeight - 1; y >= 0; y--)
  {
    char* p = imageData + width * 3 * (srcImage.resolutionHeight - 1 - y);
    const Image::Pixel* cur = &srcImage.image[y][0];
    const Image::Pixel* end = cur + srcImage.resolutionWidth;
    for(; cur < end; cur++)
    {
      yImage = int(cur->y) << 14;
      uImage = int(cur->cr) - 128;
      vImage = int(cur->cb) - 128;

      r = (yImage + factor3 * uImage) >> 14;
      g = (yImage - factor2 * vImage - factor4 * uImage) >> 14;
      b = (yImage + factor1 * vImage) >> 14;

      *p++ = r < 0 ? 0 : (r > 255 ? 255 : r);
      *p++ = g < 0 ? 0 : (g > 255 ? 255 : g);
      *p++ = b < 0 ? 0 : (b > 255 ? 255 : b);
    }
  }
  return imageData;
}

DebugDrawing3D::Image3D::~Image3D() {if(image) delete image;}

const DebugDrawing3D::Image3D& DebugDrawing3D::Image3D::operator=(const DebugDrawing3D::Image3D& other)
{
  if(image) delete image;
  (Element&) *this = other;
  point = other.point;
  rotation = other.rotation;
  width = other.width;
  height = other.height;
  image = other.image;
  // dirty hack: works only for the current use of this class
  const_cast<Image3D&>(other).image = 0;
  return *this;
}
