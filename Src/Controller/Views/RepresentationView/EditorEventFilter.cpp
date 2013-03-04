/*
 * EditorEventFilter.cpp
 *
 *  Created on: May 3, 2012
 *      Author: arne
 */

#include "EditorEventFilter.h"
#include <QEvent>
#include "RepresentationView.h"

EditorEventFilter::EditorEventFilter(QObject* pParent, RepresentationView* pView, QWidget* pSource, QtProperty* pProperty) :
  QObject(pParent), pTheView(pView), pTheSource(pSource), pTheProperty(pProperty)
{}

bool EditorEventFilter::eventFilter(QObject *obj, QEvent *event)
{
  if (NULL != pTheView)
  {
    return pTheView->handlePropertyEditorEvent(pTheSource, pTheProperty, event);
  }
  else
  {
    return false;
  }
}



