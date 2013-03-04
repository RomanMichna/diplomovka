/**
* @file Controller/Views/ViewBike/BikeMenuBar.h
*
* Declaration of class BikeMenuBar
*
* @author <a href="mailto:judy@tzi.de">Judith Müller</a>
*/

#pragma once

#include <QMenuBar>
#include <QWidget>

class BikeMenuBar : public QMenuBar
{
public:
  BikeMenuBar(QWidget* parent);

  QAction* xy_plane;
  QAction* xz_plane;
  QAction* yz_plane;

  QMenu* phaseMenu;
  QMenu* dragPlaneMenu;

  QActionGroup* dragPlaneActionGroup;
private:

  void createActions();
  void createMenus();
};
