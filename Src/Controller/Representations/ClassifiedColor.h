/*
 * ClassifiedColor.h
 * Declaration of a class representing a classified YUV color.
 */

#pragma once

#include "Tools/ColorClasses.h"
#include "Tools/Math/Vector3.h"
#include "Tools/Math/Common.h"
#include "Tools/Configuration/ConfigMap.h"

class ClassifiedColor
{
private:
  ColorClasses::Color colorClass;
  Vector3<int> color;
public:
  ClassifiedColor() { ClassifiedColor(ColorClasses::none, 0, 0, 0); }
  ClassifiedColor(ColorClasses::Color colorClass, int y, int u, int v)
    : colorClass(colorClass)
  {
    color[0] = y;
    color[1] = u;
    color[2] = v;
  }
  ClassifiedColor(const ClassifiedColor& other) : colorClass(other.colorClass), color(other.color) {}
  ColorClasses::Color getColorClass() const { return colorClass; }
  int getU() const { return color[1]; }
  int getV() const { return color[2]; }
  int getY() const { return color[0]; }
  int get(int i) const { return color[i]; }
  int distance(const Vector3<int>& otherColor) const
  {
    return (color - otherColor).abs();
  }
  ClassifiedColor& operator=(const ClassifiedColor& other) { colorClass = other.colorClass; color = other.color; return *this; }
};

class ClassifiedColorComparator
{
private:
  unsigned int depth;
public:
  ClassifiedColorComparator(unsigned int depth) : depth(depth) {}
  bool operator()(const ClassifiedColor& color1, const ClassifiedColor& color2)
  { unsigned int component = depth % 3; return color1.get(component) < color2.get(component); }
  ClassifiedColorComparator& operator=(const ClassifiedColorComparator& other) { depth = other.depth; return *this; }
};

CONFIGMAP_STREAM_IN_DELCARE(ClassifiedColor);
CONFIGMAP_STREAM_OUT_DELCARE(ClassifiedColor);

ConfigMap& operator << (ConfigMap& cv, const ClassifiedColor& classifiedColor);
const ConfigMap& operator >> (const ConfigMap& cv, ClassifiedColor& classifiedColor);
