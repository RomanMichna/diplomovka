/**
* @file Controller/Views/ColorTableView.h
*
* Declaration of class ColorTableView
*
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#pragma once

#include "View3D.h"

class RobotConsole;
class ColorTableHandler;

/**
* @class ColorTableView
*
* A class to represent a view with information about the timing of modules.
*
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/
class ColorTableView : public View3D
{
public:
  /**
  * Constructor.
  * @param fullName The full name of the view.
  * @param c The console object.
  * @param cth The color table handler containing the color table to be drawn.
  * @param b The background color.
  */
  ColorTableView(const QString& fullName, RobotConsole& c, const ColorTableHandler& cth, const Vector3<>& b);

protected:
  /**
  * Update the display lists if required.
  */
  virtual void updateDisplayLists();

  /**
  * Need the display lists to be updated?
  * @return Yes or no?
  */
  virtual bool needsUpdate() const;

private:
  RobotConsole& console; /**< A reference to the console object. */
  const ColorTableHandler& colorTableHandler; /**< The color table handler containing the color table to be drawn. */
  unsigned lastTimeStamp; /**< The time stamp of last color table that was drawn. */
};
