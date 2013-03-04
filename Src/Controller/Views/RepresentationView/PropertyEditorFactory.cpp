/*
 * PropertyEditorFactory.cpp
 *
 *  Created on: Apr 27, 2012
 *      Author: arne
 */

#include "PropertyEditorFactory.h"
#include <qtpropertybrowser.h>
#include <limits>
#include "EditorEventFilter.h"
#include "Platform/BHAssert.h"
#include <QLabel>

QWidget* PropertyEditorFactory::createEditor(QtVariantPropertyManager *pManager, QtProperty *pProperty, QWidget *pParent)
{
  /**
   * Whenever the user wants to edit a property a new editor is created.
   * The editor is destroyed as soon as the user finished editing the property.
   */
  QWidget* pReturnWidget = NULL;
  //FIXME copy&paste code
  if(TypeDescriptor::getTypeId<unsigned int>() == pManager->propertyType(pProperty))
  {
    QSpinBox* pBox = new QSpinBox(pParent);
    pBox->setKeyboardTracking(false); //This is done to avoid changed updates on every keypress
    propertyToSpinBox[pProperty] = pBox;
    spinBoxToProperty[pBox] = pProperty;
    pBox->setMinimum(0);
    //Maximum can only be int max because the spinbox uses int internally
    pBox->setMaximum(std::numeric_limits<int>::max());
    pBox->setValue((int)pManager->value(pProperty).value<unsigned int>()); //the cast to int is not very nice :(
    pReturnWidget = pBox;
    connect(pBox, SIGNAL(valueChanged(int)), this, SLOT(slotSpinBoxValueChanged(int)));
    connect(pBox, SIGNAL(destroyed(QObject*)), this, SLOT(slotEditorDestroyed(QObject*)));
  }
  else if(TypeDescriptor::getTypeId<float>() == pManager->propertyType(pProperty))
  {
    QDoubleSpinBox* pBox = new QDoubleSpinBox(pParent);
    pBox->setKeyboardTracking(false);
    pBox->setDecimals(10);
    propertyToDoubleSpinBox[pProperty] = pBox;
    doubleSpinBoxToProperty[pBox] = pProperty;
    pBox->setMinimum(-std::numeric_limits<float>::max());
    pBox->setMaximum(std::numeric_limits<float>::max());
    pBox->setValue(pManager->value(pProperty).value<float>());
    pReturnWidget = pBox;
    connect(pBox, SIGNAL(valueChanged(double)), this, SLOT(slotFloatSpinBoxValueChanged(double)));
    connect(pBox, SIGNAL(destroyed(QObject*)), this, SLOT(slotEditorDestroyed(QObject*)));
  }
  else if(TypeDescriptor::getTypeId<short>() == pManager->propertyType(pProperty))
  {
    QSpinBox* pBox = new QSpinBox(pParent);
    pBox->setKeyboardTracking(false);
    propertyToSpinBox[pProperty] = pBox;
    spinBoxToProperty[pBox] = pProperty;
    pBox->setMinimum(std::numeric_limits<short>::min());
    pBox->setMaximum(std::numeric_limits<short>::max());
    pBox->setValue((int)pManager->value(pProperty).value<short>());
    pReturnWidget = pBox;
    connect(pBox, SIGNAL(valueChanged(int)), this, SLOT(slotSpinBoxValueChanged(int)));
    connect(pBox, SIGNAL(destroyed(QObject*)), this, SLOT(slotEditorDestroyed(QObject*)));
  }
  else if(TypeDescriptor::getTypeId<unsigned short>() == pManager->propertyType(pProperty))
  {
    QSpinBox* pBox = new QSpinBox(pParent);
    pBox->setKeyboardTracking(false);
    propertyToSpinBox[pProperty] = pBox;
    spinBoxToProperty[pBox] = pProperty;
    pBox->setMinimum(std::numeric_limits<unsigned short>::min());
    pBox->setMaximum(std::numeric_limits<unsigned short>::max());
    pBox->setValue((int)pManager->value(pProperty).value<unsigned short>());
    pReturnWidget = pBox;
    connect(pBox, SIGNAL(valueChanged(int)), this, SLOT(slotSpinBoxValueChanged(int)));
    connect(pBox, SIGNAL(destroyed(QObject*)), this, SLOT(slotEditorDestroyed(QObject*)));
  }
  else if(TypeDescriptor::getTypeId<unsigned char>() == pManager->propertyType(pProperty))
  {
    QSpinBox* pBox = new QSpinBox(pParent);
    pBox->setKeyboardTracking(false);
    propertyToSpinBox[pProperty] = pBox;
    spinBoxToProperty[pBox] = pProperty;
    pBox->setMinimum(std::numeric_limits<unsigned char>::min());
    pBox->setMaximum(std::numeric_limits<unsigned char>::max());
    pBox->setValue((int)pManager->value(pProperty).value<unsigned char>());
    pReturnWidget = pBox;
    connect(pBox, SIGNAL(valueChanged(int)), this, SLOT(slotSpinBoxValueChanged(int)));
    connect(pBox, SIGNAL(destroyed(QObject*)), this, SLOT(slotEditorDestroyed(QObject*)));
  }
  else
  {
    pReturnWidget = QtVariantEditorFactory::createEditor(pManager,pProperty,pParent);
  }
  
  if(NULL != pReturnWidget)
  {
    //The event filter is used to forward all events to the RepresentationView
    pReturnWidget->installEventFilter(new EditorEventFilter(this, pTheView, pReturnWidget, pProperty));
  }
  else
  {
    //In case of error return a label containing an error message
    QLabel* l = new QLabel(pParent);
    l->setText("Failed to create an editor for this type");
    pReturnWidget = l;
  }
  
  return pReturnWidget;
}

void PropertyEditorFactory::connectPropertyManager(QtVariantPropertyManager *manager)
{
  connect(manager, SIGNAL(valueChanged(QtProperty*, const QVariant&)), this, SLOT(slotManagerValueChanged(QtProperty*, const QVariant&)));
  pTheManager = manager;
  QtVariantEditorFactory::connectPropertyManager(manager);
}

void PropertyEditorFactory::disconnectPropertyManager(QtVariantPropertyManager *manager)
{
  //FIXME disconnect signals that have been connected in connectPropertyManager
}

/**
 * This slot is invoked whenever a properties value changes.
 * It updates the editors value.
 */
void PropertyEditorFactory::slotManagerValueChanged(QtProperty *property, const QVariant & val)
{
  //This method does not get called usually because the auto updating is disabled while editing.
  if(propertyToSpinBox.contains(property))
  {
    int value = val.value<int>();
    QSpinBox* pBox = propertyToSpinBox[property];
    pBox->setValue(value);
  }
  else if(propertyToDoubleSpinBox.contains(property))
  {
    //FIXME float -> double
    propertyToDoubleSpinBox[property]->setValue(val.value<float>()); //the variant contains a float not a double
  }
}


/**
 * This slot is invoked whenever one of the managed spinboxes changes its value.
 * It updates the value in the property belonging to the box.
 */
void PropertyEditorFactory::slotSpinBoxValueChanged(int newValue)
{
  QSpinBox* pCaller = qobject_cast<QSpinBox*>(QObject::sender());
  
  ASSERT(pCaller != NULL); //qobject_cast returns NULL in case of error
  ASSERT(spinBoxToProperty.contains(pCaller));
  ASSERT(NULL != pTheManager);
  
  pTheManager->setValue(spinBoxToProperty[pCaller],QVariant::fromValue(newValue));
}

/**
 * This slot is invoked whenever one of the managed DoubleSpinboxes (float really) changes its value.
 * It updates the value of the property belonging to the box.
 */
void PropertyEditorFactory::slotFloatSpinBoxValueChanged(double newValue)
{
  QDoubleSpinBox* pCaller = qobject_cast<QDoubleSpinBox*>(QObject::sender());
  
  ASSERT(NULL != pCaller); //qobject_cast returns NULL in case of error
  ASSERT(doubleSpinBoxToProperty.contains(pCaller));
  ASSERT(NULL != pTheManager);

  float val = (float)newValue;//this is ok because the spinBox was limited to float ranges
  pTheManager->setValue(doubleSpinBoxToProperty[pCaller],QVariant::fromValue(val));

}

void PropertyEditorFactory::slotEditorDestroyed(QObject* pObject)
{
  //since we only get a pointer to QObject and not to QSpinBox or QDoubleSpinBox
  //we have to search all maps to find the right one :(
  //note: qobject_cast does not work. Casting pObject to QSpinBox always returns NULL, even if it is a QSpinBox ...
  
  QMap<QSpinBox *, QtProperty *>::ConstIterator itEditor =
  spinBoxToProperty.constBegin();
  while (itEditor != spinBoxToProperty.constEnd())
  {
    if (itEditor.key() == pObject)
    {
      QSpinBox* editor = itEditor.key();
      QtProperty* property = itEditor.value();
      propertyToSpinBox.remove(property);
      spinBoxToProperty.remove(editor);
      return; //there can only be one editor in any of the lists.
    }
    itEditor++;
  }
  
  QMap<QDoubleSpinBox *, QtProperty *>::ConstIterator itEditorDouble =
  doubleSpinBoxToProperty.constBegin();
  while (itEditorDouble != doubleSpinBoxToProperty.constEnd())
  {
    if (itEditorDouble.key() == pObject)
    {
      QDoubleSpinBox* editor = itEditorDouble.key();
      QtProperty* property = itEditorDouble.value();
      propertyToDoubleSpinBox.remove(property);
      doubleSpinBoxToProperty.remove(editor);
      return; //there can only be one editor in any of the lists.
    }
    itEditorDouble++;
  }
}

