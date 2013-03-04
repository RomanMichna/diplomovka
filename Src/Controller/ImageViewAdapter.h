/**
 * @file Controller/ImageViewAdapter.h
 *
 * ...
 *
 * @author <a href="mailto:ojlr@informatik.uni-bremen.de">Ole Jan Lars Riemann</a>
 */

#pragma once

#include "Tools/Math/Vector2.h"
#include <string>
#include <map>
#include <memory>

using namespace std;

class PointListener
{
public:
  virtual void deliverPoint(const Vector2<int>& point) = 0;
  virtual ~PointListener();
};

class ImageViewAdapter
{
private:
  static multimap<const string, PointListener*> listeners;
public:
  static void fireClick(const string view, const Vector2<int>& point);
  static bool addListener(PointListener* listener, const string view);
  static void removeListener(PointListener* listener, const string view);
  static void removeListener(PointListener* listener);
};
