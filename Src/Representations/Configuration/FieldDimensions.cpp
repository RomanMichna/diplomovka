/**
* @file FieldDimensions.cpp
*
* Some useful functions regarding field dimensions.
*
* @author Max Risler
*/

#include "FieldDimensions.h"
#include "Tools/Debugging/Modify.h"
#include "Tools/Settings.h"
#include "Tools/Streams/InStreams.h"
#include "Platform/BHAssert.h"
#include "Tools/Math/Geometry.h"
#include "Representations/Infrastructure/RoboCupGameControlData.h"
#include "Tools/Debugging/DebugDrawings.h"
#include <limits>

// Streaming operators for special objects
const ConfigValue& operator>>(const ConfigMap& cm, Vector2<std::string> &value)
{
  cm["x"] >> value.x;
  cm["y"] >> value.y;
  return cm;
}
CONFIGMAP_STREAM_OUT(ConfigMap, Vector2<std::string>);
const ConfigValue& operator>>(const ConfigMap& cm, FieldDimensions::Dimension& value)
{
  cm["id"] >> value.id;
  cm["value"] >> value.value;
  return cm;
}
CONFIGMAP_STREAM_OUT(ConfigMap, FieldDimensions::Dimension);

FieldDimensions::FieldDimensions()
  : Boundary<>(0, 0)
{
}

void FieldDimensions::load()
{
  int* const valuesInt[] =
  {
    &xPosOpponentFieldBorder, &xPosOpponentGoal, &xPosOpponentGoalpost,
    &xPosOpponentGroundline, &xPosOpponentSideCorner, &xPosOpponentPenaltyArea,
    &xPosOpponentPenaltyMark, &xPosHalfWayLine,
    &xPosOwnPenaltyArea, &xPosOwnPenaltyMark, &xPosOwnSideCorner, &xPosOwnGroundline, &xPosOwnGoalpost,
    &xPosOwnGoal, &xPosOwnFieldBorder,
    &yPosLeftFieldBorder, &yPosLeftSideline, &yPosLeftGroundline,
    &yPosLeftPenaltyArea, &yPosLeftGoal, &yPosCenterGoal, &yPosRightGoal,
    &yPosRightPenaltyArea, &yPosRightGroundline, &yPosRightSideline, &yPosRightFieldBorder,
    &centerCircleRadius, &goalHeight, &fieldLinesWidth, &goalPostRadius,
    &xPosThrowInPointOpponentHalf, &xPosThrowInPointCenter,
    &xPosThrowInPointOwnHalf, &ballRadius, &ballFriction,
    &yPosDropInLineLeft, &yPosDropInLineRight, &xPosDropInLineOpponentHalf, &xPosDropInLineOwnHalf
  };
  const char* const namesInt[] =
  {
    "xPosOpponentFieldBorder", "xPosOpponentGoal", "xPosOpponentGoalpost",
    "xPosOpponentGroundline", "xPosOpponentSideCorner", "xPosOpponentPenaltyArea",
    "xPosOpponentPenaltyMark", "xPosHalfWayLine",
    "xPosOwnPenaltyArea", "xPosOwnPenaltyMark", "xPosOwnSideCorner", "xPosOwnGroundline", "xPosOwnGoalpost",
    "xPosOwnGoal", "xPosOwnFieldBorder",
    "yPosLeftFieldBorder", "yPosLeftSideline", "yPosLeftGroundline",
    "yPosLeftPenaltyArea", "yPosLeftGoal", "yPosCenterGoal", "yPosRightGoal",
    "yPosRightPenaltyArea", "yPosRightGroundline", "yPosRightSideline", "yPosRightFieldBorder",
    "centerCircleRadius",
    "goalHeight", "fieldLinesWidth", "goalPostRadius",
    "xPosThrowInPointOpponentHalf", "xPosThrowInPointCenter",
    "xPosThrowInPointOwnHalf", "ballRadius", "ballFriction",
    "yPosDropInLineLeft", "yPosDropInLineRight", "xPosDropInLineOpponentHalf", "xPosDropInLineOwnHalf"
  };

  const int numOfValuesInt = sizeof(valuesInt) / sizeof(int*);
  ASSERT(sizeof(namesInt) / sizeof(char*) == numOfValuesInt);

  bool initializedInt[numOfValuesInt];

  for(int i = 0; i < numOfValuesInt; initializedInt[i++] = false);

  ConfigMap cm;
  if(cm.read(Global::getSettings().expandLocationFilename("fieldDimensions.cfg")) >= 0)
  {
    readDimensions(valuesInt, namesInt, initializedInt, numOfValuesInt, cm);
    readLines(valuesInt, namesInt, initializedInt, numOfValuesInt, cm);
    readCorners(valuesInt, namesInt, initializedInt, numOfValuesInt, cm);

    add(Vector2<>((float) xPosOpponentFieldBorder, (float) yPosLeftFieldBorder));
    add(Vector2<>((float) xPosOwnFieldBorder, (float) yPosRightFieldBorder));
  }
  else
    ASSERT(false);
}

void FieldDimensions::readDimensions(int* const* valuesInt, const char* const* namesInt, bool* initializedInt, int numOfValuesInt, const ConfigMap& cm)
{
  std::vector<Dimension> dimensions;
  cm["dimensions"] >> dimensions;

  for(std::vector<Dimension>::iterator dim = dimensions.begin(); dim != dimensions.end(); ++dim)
  {
    int i;
    for(i = 0; i < numOfValuesInt; i++)
      if(dim->id == namesInt[i])
      {
        if(initializedInt[i])
        {
          ASSERT(false);
        }
        *valuesInt[i] = readValue(dim->value, valuesInt, namesInt, initializedInt, numOfValuesInt);
        initializedInt[i] = true;
        break;
      }
    ASSERT(i < numOfValuesInt);
  }
  int i;
  for(i = 0; i < numOfValuesInt; i++)
    if(!initializedInt[i])
    {
      ASSERT(false);
    }
}

void FieldDimensions::readLines(const int* const* valuesInt, const char* const* namesInt, const bool* initializedInt, int numOfValuesInt, const ConfigMap& cm)
{
  LinesTable* linesTables[] = {&carpetBorder, &fieldBorder, &fieldLines};
  const char* linesTableNames[] = {"carpetBorder", "fieldBorder", "fieldLines"};
  const int numOfLinesTables = sizeof(linesTables) / sizeof(LinesTable*);
  ASSERT(sizeof(linesTableNames) / sizeof(char*) == numOfLinesTables);

  int i;
  for(i = 0; i < numOfLinesTables; i++)
  {
    ListConfigValue lines = cm[linesTableNames[i]];

    for(size_t line = 0; line < lines.length(); ++line)
    {
      std::string type;
      lines[line]["type"] >> type;
      if(type == "line")
      {
        Vector2<std::string> from, to;
        Vector2<> p1, p2;

        lines[line]["from"] >> from;
        lines[line]["to"] >> to;

        p1.x = (float) readValue(from.x, valuesInt, namesInt, initializedInt, numOfValuesInt);
        p1.y = (float) readValue(from.y, valuesInt, namesInt, initializedInt, numOfValuesInt);
        p2.x = (float) readValue(to.x, valuesInt, namesInt, initializedInt, numOfValuesInt);
        p2.y = (float) readValue(to.y, valuesInt, namesInt, initializedInt, numOfValuesInt);

        linesTables[i]->push(p1, p2);
      }
      else if(type == "circle")
      {
        Vector2<std::string> centerString;
        std::string radiusString;
        std::string numOfSegmentsString;
        Vector2<> center;
        float radius;
        int numOfSegments;

        lines[line]["center"] >> centerString;
        lines[line]["radius"] >> radiusString;
        lines[line]["numOfSegments"] >> numOfSegmentsString;

        center.x = (float) readValue(centerString.x, valuesInt, namesInt, initializedInt, numOfValuesInt);
        center.y = (float) readValue(centerString.y, valuesInt, namesInt, initializedInt, numOfValuesInt);
        radius = (float) readValue(radiusString, valuesInt, namesInt, initializedInt, numOfValuesInt);
        numOfSegments = readValue(numOfSegmentsString, valuesInt, namesInt, initializedInt, numOfValuesInt);

        linesTables[i]->pushCircle(center, radius, numOfSegments);
      }
      else if(type == "quartercircle")
      {
        Vector2<std::string> centerString;
        std::string radiusString;
        std::string numOfSegmentsString;
        std::string angleString;
        Vector2<> center;
        float radius;
        int numOfSegments;
        int angle;

        lines[line]["center"] >> centerString;
        lines[line]["radius"] >> radiusString;
        lines[line]["numOfSegments"] >> numOfSegmentsString;
        lines[line]["angle"] >> angleString;

        center.x = (float) readValue(centerString.x, valuesInt, namesInt, initializedInt, numOfValuesInt);
        center.y = (float) readValue(centerString.y, valuesInt, namesInt, initializedInt, numOfValuesInt);
        radius = (float) readValue(radiusString, valuesInt, namesInt, initializedInt, numOfValuesInt);
        numOfSegments = readValue(numOfSegmentsString, valuesInt, namesInt, initializedInt, numOfValuesInt);
        angle = readValue(angleString, valuesInt, namesInt, initializedInt, numOfValuesInt);

        linesTables[i]->pushQuarterCircle(center, radius, numOfSegments, angle);
      }
      else
      {
        ASSERT(false);
      }
    }
  }
}

void FieldDimensions::readCorners(const int* const* valuesInt, const char* const* namesInt, const bool* initializedInt, int numOfValuesInt, const ConfigMap& cm)
{
  for(int i = 0; i < numOfCornerClasses; ++i)
  {
    std::vector<Vector2<std::string> > cornersString;
    cm[getName((CornerClass) i)] >> cornersString;

    for(std::vector<Vector2<std::string> >::iterator p1 = cornersString.begin(); p1 != cornersString.end(); ++p1)
    {
      Vector2<int> corner;
      corner.x = readValue(p1->x, valuesInt, namesInt, initializedInt, numOfValuesInt);
      corner.y = readValue(p1->y, valuesInt, namesInt, initializedInt, numOfValuesInt);
      corners[i].push_back(corner);
    }
  }
}

int FieldDimensions::readValue(const string& buf, const int* const* values, const char* const* names, const bool* initialized, int numOfValues) const
{
  string n = buf;
  bool neg = false;
  if(buf[0] == '-')
  {
    neg = true;
    n = n.substr(1);
  }
  for(int j = 0; j < numOfValues; j++)
    if(n == names[j])
    {
      if(!initialized[j])
      {
        ASSERT(false);
      }
      return (neg) ? (-*values[j]) : (*values[j]);
    }
  return (int)strtol(buf.c_str(), (char**)NULL, 0);
}

Pose2D FieldDimensions::randomPoseOnField() const
{
  Pose2D pose;
  do
    pose = Pose2D::random(x, y, Range<>(-pi, pi));
  while(!isInsideField(pose.translation));
  return pose;
}

Pose2D FieldDimensions::randomPoseOnCarpet() const
{
  Pose2D pose;
  do
    pose = Pose2D::random(x, y, Range<>(-pi, pi));
  while(!isInsideCarpet(pose.translation));
  return pose;
}

void FieldDimensions::draw() const
{
  drawLines();
  drawCorners();
}

void FieldDimensions::drawLines() const
{
  DECLARE_DEBUG_DRAWING("field lines", "drawingOnField");
  COMPLEX_DRAWING("field lines",
  {
    ASSERT(carpetBorder.lines.size() <= 4);
    Vector2<> points[4];
    for(unsigned i = 0; i < carpetBorder.lines.size(); ++i)
      points[i] = carpetBorder.lines[i].corner.translation;
    POLYGON("field lines", (int) carpetBorder.lines.size(), points, 0, Drawings::ps_solid, ColorRGBA(0, 180, 0), Drawings::bs_solid, ColorRGBA(0, 140, 0));

    ColorRGBA lineColor(192, 192, 192);
    for(vector<LinesTable::Line>::const_iterator i = fieldLines.lines.begin(); i != fieldLines.lines.end(); ++i)
    {
      Vector2<> source = i->corner.translation;
      Pose2D target(i->corner);
      target.translate(i->length, 0);
      LINE("field lines", source.x, source.y, target.translation.x, target.translation.y, fieldLinesWidth, Drawings::ps_solid, lineColor);
    }
  });
}

void FieldDimensions::drawPolygons(RoboCup::uint32 ownColor) const
{
  DECLARE_DEBUG_DRAWING("field polygons", "drawingOnField");
  COMPLEX_DRAWING("field polygons",
  {
    ColorRGBA own = ownColor == TEAM_BLUE ? ColorRGBA(50, 120, 127) : ColorRGBA(127, 120, 50);
    ColorRGBA opp = ownColor != TEAM_BLUE ? ColorRGBA(50, 120, 127) : ColorRGBA(127, 120, 50);

    Vector2<> goal[4];
    goal[0] = Vector2<>((float) xPosOwnGoalpost - fieldLinesWidth * 0.5f, (float) yPosLeftGoal);
    goal[1] = Vector2<>((float) xPosOwnGoalpost - fieldLinesWidth * 0.5f, (float) yPosRightGoal);
    goal[2] = Vector2<>((float) xPosOwnGoal, (float) yPosRightGoal);
    goal[3] = Vector2<>((float) xPosOwnGoal, (float) yPosLeftGoal);
    POLYGON("field polygons", 4, goal, 0, Drawings::ps_solid, own, Drawings::bs_solid, own);

    goal[0] = Vector2<>((float) xPosOpponentGoalpost + fieldLinesWidth * 0.5f, (float) yPosLeftGoal);
    goal[1] = Vector2<>((float) xPosOpponentGoalpost + fieldLinesWidth * 0.5f, (float) yPosRightGoal);
    goal[2] = Vector2<>((float) xPosOpponentGoal, (float) yPosRightGoal);
    goal[3] = Vector2<>((float) xPosOpponentGoal, (float) yPosLeftGoal);
    POLYGON("field polygons", 4, goal, 0, Drawings::ps_solid, opp, Drawings::bs_solid, opp);

    CIRCLE("field polygons", xPosOpponentGoalpost, yPosLeftGoal, 50, 0, Drawings::ps_solid,
    ColorRGBA(ColorClasses::yellow), Drawings::bs_solid, ColorRGBA(ColorClasses::yellow));
    CIRCLE("field polygons", xPosOpponentGoalpost, yPosRightGoal, 50, 0, Drawings::ps_solid,
    ColorRGBA(ColorClasses::yellow), Drawings::bs_solid, ColorRGBA(ColorClasses::yellow));

    CIRCLE("field polygons", xPosOwnGoalpost, yPosLeftGoal, 50, 0, Drawings::ps_solid,
    ColorRGBA(ColorClasses::yellow), Drawings::bs_solid, ColorRGBA(ColorClasses::yellow));
    CIRCLE("field polygons", xPosOwnGoalpost, yPosRightGoal, 50, 0, Drawings::ps_solid,
    ColorRGBA(ColorClasses::yellow), Drawings::bs_solid, ColorRGBA(ColorClasses::yellow));
  });
}

void FieldDimensions::drawCorners() const
{
  DECLARE_DEBUG_DRAWING("field corners", "drawingOnField");
#ifndef RELEASE
  CornerClass c = xCorner;
#endif
  MODIFY_ENUM("fieldDimensions:cornerClass", c);
  COMPLEX_DRAWING("field corners",
  {
    for(CornersTable::const_iterator i = corners[c].begin(); i != corners[c].end(); ++i)
      LARGE_DOT("field corners", i->x, i->y, ColorRGBA(255, 255, 255), ColorRGBA(255, 255, 255));
  });
}

void FieldDimensions::LinesTable::push(const Pose2D& p, float l, bool isPartOfCircle)
{
  LinesTable::Line line;
  line.corner = p;
  line.length = l;
  line.isPartOfCircle = isPartOfCircle;
  lines.push_back(line);
}

void FieldDimensions::LinesTable::push(const Vector2<>& s, const Vector2<>& e, bool isPartOfCircle)
{
  Vector2<> d = e - s;
  push(Pose2D(d.angle(), s), d.abs(), isPartOfCircle);
}

void FieldDimensions::LinesTable::pushCircle(const Vector2<>& center, float radius, int numOfSegments)
{
  Vector2<> p1, p2;
  for(float a = 0; a <= pi_4; a += pi2 / numOfSegments)
  {
    p1 = Vector2<>(sin(a), cos(a)) * radius;
    if(a > 0)
    {
      push(center + p1, center + p2, true);
      push(center + Vector2<>(p1.x, -p1.y), center + Vector2<>(p2.x, -p2.y), true);
      push(center + Vector2<>(-p1.x, p1.y), center + Vector2<>(-p2.x, p2.y), true);
      push(center - p1, center - p2, true);
      push(center + Vector2<>(p1.y, p1.x), center + Vector2<>(p2.y, p2.x), true);
      push(center + Vector2<>(p1.y, -p1.x), center + Vector2<>(p2.y, -p2.x), true);
      push(center + Vector2<>(-p1.y, p1.x), center + Vector2<>(-p2.y, p2.x), true);
      push(center + Vector2<>(-p1.y, -p1.x), center + Vector2<>(-p2.y, -p2.x), true);
    }
    p2 = p1;
  }
}

void FieldDimensions::LinesTable::pushQuarterCircle(const Vector2<>& center, float radius, int numOfSegments, int angle)
{
  Vector2<> p1, p2;
  for(float a = 0; a <= pi_4; a += pi2 / numOfSegments)
  {
    p1 = Vector2<>(sin(a), cos(a)) * radius;
    if(a > 0)
    {
      switch(angle)
      {
      case 0:
        push(center + p1, center + p2, true);
        push(center + Vector2<>(p1.y, p1.x), center + Vector2<>(p2.y, p2.x), true);
        break;
      case 1:
        push(center + Vector2<>(-p1.x, p1.y), center + Vector2<>(-p2.x, p2.y), true);
        push(center + Vector2<>(-p1.y, p1.x), center + Vector2<>(-p2.y, p2.x), true);
        break;
      case 2:
        push(center - p1, center - p2, true);
        push(center + Vector2<>(-p1.y, -p1.x), center + Vector2<>(-p2.y, -p2.x), true);
        break;
      case 3:
        push(center + Vector2<>(p1.x, -p1.y), center + Vector2<>(p2.x, -p2.y), true);
        push(center + Vector2<>(p1.y, -p1.x), center + Vector2<>(p2.y, -p2.x), true);
        break;
      }
    }
    p2 = p1;
  }
}

void FieldDimensions::LinesTable::doubleSided(float width, const FieldDimensions::LinesTable& single)
{
  for(vector<Line>::const_iterator i = single.lines.begin(); i != single.lines.end(); ++i)
  {
    push(i->corner +
         Pose2D(Vector2<>(-width / 2, width / 2)),
         i->length + width);
    push(i->corner +
         Pose2D(pi, Vector2<>(i->length, 0)) +
         Pose2D(Vector2<>(-width / 2, width / 2)),
         i->length + width);
  }
}

bool FieldDimensions::LinesTable::isInside(const Vector2<>& v) const
{
  //note:
  //This function assumes that the point (0,0) is inside and
  //that for any point inside the area the line to (0,0) belongs to the area too.

  Geometry::Line testLine(v, -v);
  for(vector<Line>::const_iterator i = lines.begin(); i != lines.end(); ++i)
  {
    float factor;
    Geometry::Line border(i->corner, i->length);
    if(Geometry::getIntersectionOfRaysFactor(border, testLine, factor))
      return false;
  }
  return true;
}

float FieldDimensions::LinesTable::clip(Vector2<>& v) const
{
  if(isInside(v))
    return 0;
  else
  {
    Vector2<> old = v,
              v2;
    float minDist = 100000;
    for(vector<Line>::const_iterator i = lines.begin(); i != lines.end(); ++i)
    {
      Vector2<> diff = (Pose2D(old) - i->corner).translation;
      if(diff.x < 0)
        v2 = i->corner.translation;

      else if(diff.x > i->length)
        v2 = (i->corner + Pose2D(Vector2<>(i->length, 0))).translation;
      else
        v2 = (i->corner + Pose2D(Vector2<>(diff.x, 0))).translation;
      float dist = (old - v2).abs();
      if(minDist > dist)
      {
        minDist = dist;
        v = v2;
      }
    }
    return (v - old).abs();
  }
}

bool FieldDimensions::LinesTable::getClosestPoint(Vector2<>& vMin, const Pose2D& p, int numberOfRotations, float minLength) const
{
  int trueNumberOfRotations = numberOfRotations;
  if(numberOfRotations == 2)
    numberOfRotations = 4;

  // target angle -> target index
  float r = p.rotation / pi2 * numberOfRotations + 0.5f;
  if(r < 0)
    r += numberOfRotations;
  int targetRot = int(r);
  ASSERT(targetRot >= 0 && targetRot < numberOfRotations);
  targetRot %= trueNumberOfRotations;
  Vector2<> v2;
  float minDist = 100000;
  for(vector<Line>::const_iterator i = lines.begin(); i != lines.end(); ++i)
    if(i->length >= minLength)
    {
      // angle -> index
      float r = (i->corner.rotation + pi_2) / pi2 * numberOfRotations + 0.5f;
      if(r < 0)
        r += numberOfRotations;
      else if(r >= numberOfRotations)
        r -= numberOfRotations;
      int rot = int(r);
      ASSERT(rot >= 0 && rot < numberOfRotations);
      rot %= trueNumberOfRotations;

      // index must be target index
      if(rot == targetRot)
      {
        Vector2<> diff = (p - i->corner).translation;
        if(diff.x < 0)
          v2 = i->corner.translation;
        else if(diff.x > i->length)
          v2 = (i->corner + Pose2D(Vector2<>(i->length, 0))).translation;
        else
          v2 = (i->corner + Pose2D(Vector2<>(diff.x, 0))).translation;
        Vector2<> vDiff = v2 - p.translation;
        float dist = vDiff.abs();
        if(minDist > dist)
        {
          minDist = dist;
          vMin = v2;
        }
      }
    }
  return (minDist < 100000);
}

float FieldDimensions::LinesTable::getDistance(const Pose2D& pose) const
{
  float minDist = 100000;
  for(vector<Line>::const_iterator i = lines.begin(); i != lines.end(); ++i)
  {
    Vector2<> v1 = (i->corner - pose).translation,
              v2 = (i->corner + Pose2D(Vector2<>(i->length, 0))
                    - pose).translation;
    if(v1.y < 0 && v2.y > 0)
    {
      float dist = v1.x + (v2.x - v1.x) * -v1.y / (v2.y - v1.y);
      if(dist >= 0 && dist < minDist)
        minDist = dist;
    }
  }
  return minDist == 100000 ? -1 : minDist;
}
/*
void FieldDimensions::LinesTable::draw(const ColorRGBA& color, bool drawNormals) const
{
  for(vector<Line>::const_iterator i = lines.begin(); i != lines.end(); ++i)
  {
    Vector2<> s = i->corner.translation;
    Vector2<> p = (i->corner + Pose2D(Vector2<>(i->length, 0))).translation;
    LINE("field lines",
         (int) s.x,
         (int) s.y,
         (int) p.x,
         (int) p.y,
         0, Drawings::ps_solid, color);
    if(drawNormals)
    {
      p = (i->corner + Pose2D(Vector2<>(i->length / 2, 0))).translation;
      Vector2<> p2 = (i->corner + Pose2D(Vector2<>(i->length / 2, 100))).translation;
      LINE("field lines",
           (int) p2.x,
           (int) p2.y,
           (int) p.x,
           (int) p.y,
           0, Drawings::ps_solid, color);
    }
  }
}
*/
const Vector2<int>& FieldDimensions::CornersTable::getClosest(const Vector2<int>& p) const
{
  ASSERT(!empty());
  int maxDistance2 = numeric_limits<int>().max();
  const Vector2<int>* closest = 0;
  for(const_iterator i = begin(); i != end(); ++i)
  {
    Vector2<int> diff = p - *i;
    int distance2 = diff * diff;
    if(maxDistance2 > distance2)
    {
      maxDistance2 = distance2;
      closest = &*i;
    }
  }
  return *closest;
}

const Vector2<> FieldDimensions::CornersTable::getClosest(const Vector2<>& p) const
{
  Vector2<int> closest = getClosest(Vector2<int>((int) p.x, (int) p.y));
  return Vector2<>((float) closest.x, (float) closest.y);
}

void FieldDimensions::serialize(In* in, Out* out)
{
  STREAM_REGISTER_BEGIN;
  STREAM(xPosOpponentFieldBorder);
  STREAM(xPosOpponentGoal);
  STREAM(xPosOpponentGoalpost);
  STREAM(xPosOpponentGroundline);
  STREAM(xPosOpponentSideCorner);
  STREAM(xPosOpponentPenaltyArea);
  STREAM(xPosOpponentPenaltyMark);
  STREAM(xPosHalfWayLine);
  STREAM(xPosOwnPenaltyArea);
  STREAM(xPosOwnPenaltyMark);
  STREAM(xPosOwnSideCorner);
  STREAM(xPosOwnGroundline);
  STREAM(xPosOwnGoalpost);
  STREAM(xPosOwnGoal);
  STREAM(xPosOwnFieldBorder);
  STREAM(yPosLeftFieldBorder);
  STREAM(yPosLeftSideline);
  STREAM(yPosLeftGroundline);
  STREAM(yPosLeftPenaltyArea);
  STREAM(yPosLeftGoal);
  STREAM(yPosCenterGoal);
  STREAM(yPosRightGoal);
  STREAM(yPosRightPenaltyArea);
  STREAM(yPosRightGroundline);
  STREAM(yPosRightSideline);
  STREAM(yPosRightFieldBorder);
  STREAM(centerCircleRadius);
  STREAM(goalHeight);
  STREAM(ballRadius);
  STREAM(ballFriction);
  STREAM(fieldLinesWidth);
  STREAM(xPosThrowInPointOpponentHalf);
  STREAM(xPosThrowInPointCenter);
  STREAM(xPosThrowInPointOwnHalf);
  STREAM(fieldLines);
  STREAM(carpetBorder);
  STREAM(fieldBorder);
  STREAM(corners);
  STREAM_REGISTER_FINISH;
}
