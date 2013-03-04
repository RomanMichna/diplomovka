/*
 * RepresentationWidget.h
 *
 *  Created on: Apr 17, 2012
 *      Author: arne
 */

#pragma once

#include <QWidget>
#include <QPainter>
#include <QApplication>
#include <QAction>
#include <QMouseEvent>
#include <QResizeEvent>
#include <QSettings>
#include <QMenu>
#include <QGridLayout>
#include <QPushButton>
#include <QCheckBox>
#include <QToolBar>
#include <QToolButton>
#include <QObject>
#include "Controller/RobotConsole.h"
#include <QtTreePropertyBrowser>
#include "PropertyEditorFactory.h"




class RepresentationWidget : public QWidget, public SimRobot::Widget
{
  Q_OBJECT

  public:

  /**
   * @param manager reference to the property manager which has been used to create the properties.
   */
  RepresentationWidget(RepresentationView& view, QtVariantPropertyManager& manager) : theView(view),
    pTheBrowser(new QtTreePropertyBrowser(this)), pTheCurrentProperty(NULL), theEditorFactory(&view), theManager(manager), theRootPropertyHasChanged(false)
  {
    setFocusPolicy(Qt::StrongFocus);

    pSetAction = new QAction(QIcon(":/Icons/upload.png"), tr("&Set"), this);
    pSetAction->setText("Set");
    pSetAction->setToolTip("Set values");
    pSetAction->setEnabled(false);
    pUnchangedAction = new QAction(QIcon(":/Icons/arrow_undo.png"), tr("&Unchanged"), this);
    pUnchangedAction->setToolTip("Set representation to unchanged");
    pUnchangedAction->setEnabled(false);
    pUnchangedAction->setText("Unchanged");
//    pAutoRefreshAction = new QAction(QIcon(":/Icons/arrow_refresh.png"), tr("&Toggle auto-update"), this);
//    pAutoRefreshAction->setCheckable(true);
//    pAutoRefreshAction->setChecked(true);
//    pAutoRefreshAction->setText("Auto-refresh");
//    pAutoRefreshAction->setToolTip("Toggle auto-refresh");
    pAutoSetAction = new QAction(QIcon(":/Icons/arrow_refresh.png"), tr("&Auto-set"), this);
    pAutoSetAction->setCheckable(true);
    pAutoSetAction->setChecked(true);
    pAutoSetAction->setText("Auto-set");
    pAutoSetAction->setToolTip("If enabled the data is send to the robot automatically.");

    connect(pSetAction , SIGNAL(triggered()), this, SLOT(setPressed()));
    connect(pUnchangedAction , SIGNAL(triggered()), this, SLOT(unchangedPressed()));
    //connect(pAutoRefreshAction , SIGNAL(triggered()), this, SLOT(autoRefreshPressed()));
  //  connect(pAutoRefreshAction, SIGNAL(toggled(bool)), this, SLOT(autoRefreshToggled(bool)));
    connect(pAutoSetAction, SIGNAL(toggled(bool)), this, SLOT(autoSetToggled(bool)));
    //Grid
    QGridLayout *pGrid = new QGridLayout(this);
    pGrid->setSpacing(2);
    pGrid->addWidget(pTheBrowser,0,0);

    setLayout(pGrid);

    pTheBrowser->setFactoryForManager(&theManager,&theEditorFactory);
    pTheBrowser->setResizeMode(QtTreePropertyBrowser::ResizeToContents);
    //connects
 //   connect(pSetButton, SIGNAL(pressed()), this, SLOT(setPressed()));
 //   connect(pAutoRefreshButton, SIGNAL(toggled(bool)), this, SLOT(refreshToggled(bool)));
 //   connect(pAutoRefreshButton, SIGNAL(pressed()), this, SLOT(autoRefreshPressed()));
 //   connect(pUnchangedButton, SIGNAL(pressed()), this, SLOT(unchangedPressed()));
  }

  ~RepresentationWidget()
  {
    pTheBrowser->clear();
    theView.removeWidget(); //Remove the Widget from the view.
    //Without a view the widget will stop updating the properties.
  }

  virtual QWidget* getWidget()
  {
    return this;
  }
  virtual void update() {

    //Adding the property to the browser draws the property.
    //Therefore this is done in the gui thread.
    SYNC_WITH(theView); //without this lock pTheCurrentProperty might be changed while adding it.

    if(theRootPropertyHasChanged && NULL != pTheCurrentProperty)
    {
      //If the property has children it is only a container.
      //Discard the container and add the children directly for a more compact view.
      QList<QtProperty *> subProps = pTheCurrentProperty->subProperties();
      if(subProps.size() > 0)
      {
        for(int i = 0; i < subProps.size(); i++)
        {
          pTheBrowser->addProperty(subProps[i]);
        }
      }
      else
      {
        pTheBrowser->addProperty(pTheCurrentProperty);
      }
      theRootPropertyHasChanged = false;
    }
  };
  virtual bool canClose() {return true;}
  virtual QMenu* createFileMenu() {return 0;}
  virtual QMenu* createUserMenu()
  {
    QMenu* pMenu = new QMenu(theView.getFullName());
    pMenu->addAction(pSetAction);
    pMenu->addAction(pUnchangedAction);
    pMenu->addAction(pAutoSetAction);
    return pMenu;
  }
  virtual QMenu* createEditMenu() {return 0;}
  virtual void paint(QPainter& painter) {}


  /**
  * Sets the root property which is displayed by this widget.
  * @note The property must have been created using the manager specified in the constructor.
  */
  void setRootProperty(QtProperty* pRootProperty)
  {
    if (pTheCurrentProperty != pRootProperty)
    {
      theRootPropertyHasChanged = true;
      pTheCurrentProperty = pRootProperty;
    }
  }

  void setUnchangedButtonEnabled(bool value)
  {
    pUnchangedAction->setEnabled(value);
  }

  void setSetButtonEnabled(bool value)
  {
    pSetAction->setEnabled(value);
  }

  private slots:
  /**
   * Is called when the set button has been pressed
   */
  void setPressed()
  {
    theView.set();
  }

  void autoSetToggled(bool checked)
  {
    theView.setAutoSet(checked);
  }

  void unchangedPressed()
  {
    theView.setUnchanged();
    setUnchangedButtonEnabled(false);
  }


  private:

  RepresentationView& theView;
  /**
   * Pointer to the property browser which is used to display the properties.
   * This is a pointer because it is registered as a child for some other widget.
   * If that widget is closed it deletes its children. However if the child is not allocated on the heap
   * the program will crash...
   */
  QtTreePropertyBrowser* pTheBrowser;
  QtProperty* pTheCurrentProperty; /**< Pointer to the property that should be displayed */
  PropertyEditorFactory theEditorFactory; /** < provides qt widgets for properties */
  QtVariantPropertyManager& theManager; /** < reference to the manager that has been used to create the properties */
  bool theRootPropertyHasChanged; /** < True when the root property has changed*/
  QAction* pSetAction;
  QAction* pUnchangedAction;
  QAction* pAutoSetAction;


};
