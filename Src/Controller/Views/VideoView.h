/**
* @file Controller/Views/VideoView.h
*
* Declaration of class VideoView
*
* @author <a href="mailto:matroc@tzi.de">Max Trocha</a>
*/

#pragma once

#include <QString>
#include <QIcon>

#include "SimRobot.h"

class VideoCtrl;

/**
* @class VideoView
*
* A class to represent a view displaying video recording of a game.
*
* @author <a href="mailto:matroc@tzi.de">Max Trocha</a>
*/
class VideoView : public SimRobot::Object
{
public:
  /**
  * Constructor.
  * @param fullName The path to this view in the scene graph
  * @param console The console object.
  */
  VideoView(const QString& fullName, VideoCtrl& console);

private:
  const QString fullName; /**< The path to this view in the scene graph */
  const QIcon icon; /**< The icon used for listing this view in the scene graph */
  VideoCtrl& console; /**< A reference to the console object. */

  /**
  * The method returns a new instance of a widget for this direct view.
  * The caller has to delete this instance. (Qt handles this)
  * @return The widget.
  */
  virtual SimRobot::Widget* createWidget();

  virtual const QString& getFullName() const {return fullName;}
  virtual const QIcon* getIcon() const {return &icon;}

  friend class VideoWidget;
};
