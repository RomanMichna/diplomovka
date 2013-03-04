/**
* @file Controller/Views/TimeCtrlView.h
*
* Declaration of class TimeCtrlView
*
* @author <a href="mailto:matroc@tzi.de">Max Trocha</a>
*/

#pragma once

#include <QString>
#include <QIcon>

#include "SimRobot.h"

class TimeCtrl;

/**
* @class TimeCtrlView
*
* A class to controll playback time of simulator logs.
*
* @author <a href="mailto:matroc@tzi.de">Max Trocha</a>
*/
class TimeCtrlView : public SimRobot::Object
{
public:
  /**
  * Constructor.
  * @param fullName The path to this view in the scene graph
  * @param timeCtrl A reference to the TimeCtrl object.
  */
  TimeCtrlView(const QString& fullName, TimeCtrl& timeCtrl);

private:
  const QString fullName; /**< The path to this view in the scene graph */
  const QIcon icon; /**< The icon used for listing this view in the scene graph */
  TimeCtrl& timeCtrl; /**< A reference to the TimeCtrl object. */

  /**
  * The method returns a new instance of a widget for this direct view.
  * The caller has to delete this instance. (Qt handles this)
  * @return The widget.
  */
  virtual SimRobot::Widget* createWidget();

  virtual const QString& getFullName() const {return fullName;}
  virtual const QIcon* getIcon() const {return &icon;}

  friend class TimeCtrlWidget;
};
