/**
* @file ActuatorsWidget.cpp
* Implementation of class ActuatorsWidget and ActuatorWidget
*/ 

#include <QLabel>
#include <QVBoxLayout>
#include <QSlider>
#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QCheckBox>
#include <QSettings>

#include "Simulation/Simulation.h"
#include "ActuatorsWidget.h"
#include "CoreModule.h"

static inline double toDeg(double angleInRad)
{ return (angleInRad * 180.0 / M_PI);}
static inline double toRad(double angleInDegree)
{ return (angleInDegree * M_PI / 180.0);}


SimRobot::Widget* ActuatorsObject::createWidget()
{
  return new ActuatorsWidget();
}

ActuatorWidget::ActuatorWidget(SimRobotCore2::ActuatorPort* actuator, QWidget* parent) : QWidget(parent),
  actuatorName(actuator->getFullName()), actuator(actuator), value(0)
{
  setMaximumWidth(80);

  isAngle = actuator->getUnit() == "°";

  slider = new QSlider(Qt::Vertical, this);
  slider->setMinimumHeight(40);
  btnExit = new QPushButton(tr("Close"), this);
  txbValue = new QSpinBox(this);
  QStringList nameList = actuatorName.split(".");
  QLabel* label = new QLabel(nameList[nameList.size() - 2], this);

  cbxSet = new QCheckBox(tr("set"), this);

  QVBoxLayout *layout = new QVBoxLayout(this);

  layout->addWidget(label);
  layout->addWidget(slider, 0, Qt::AlignHCenter);
  layout->addWidget(txbValue);
  layout->addWidget(cbxSet);
  layout->addWidget(btnExit);

  setLayout(layout);

  connect(slider, SIGNAL(valueChanged(int)), this, SLOT(valueChanged(int)));
  connect(txbValue, SIGNAL(valueChanged(int)), this, SLOT(valueChanged(int)));
  connect(btnExit, SIGNAL(released()), this, SIGNAL(releasedClose()));

  float minVal, maxVal;
  actuator->getMinAndMax(minVal, maxVal);

  if(isAngle)
  {
    minVal = toDeg(minVal);
    maxVal = toDeg(maxVal);
  }

  slider->setRange((int)minVal, (int)maxVal);
  slider->setValue(value);

  txbValue->setRange((int)minVal, (int)maxVal);
  txbValue->setValue(value);

  // restore layout
  QSettings* settings = &CoreModule::application->getLayoutSettings();
  settings->beginGroup(actuatorName);
  valueChanged(settings->value("Value", int(0)).toInt());
  cbxSet->setChecked(settings->value("Set", false).toBool());
  settings->endGroup();
}

ActuatorWidget::~ActuatorWidget()
{
  QSettings* settings = &CoreModule::application->getLayoutSettings();
  settings->beginGroup(actuatorName);
  settings->setValue("Set", cbxSet->checkState() == Qt::Checked);
  settings->setValue("Value", value);
  settings->endGroup();
}

//QSize ActuatorWidget::sizeHint () const { return QSize(80, 300); }

void ActuatorWidget::valueChanged(int value)
{
  txbValue->setValue(value);
  slider->setValue(value);
  this->value = value;
}

void ActuatorWidget::adoptActuator()
{
  if(cbxSet->checkState() == Qt::Checked)
  {    
    double value = (double)this->value;
    if(isAngle)
      value = toRad(value);
    actuator->setValue(value);
  }
}

ActuatorsWidget* ActuatorsWidget::actuatorsWidget = 0;

ActuatorsWidget::ActuatorsWidget()
{
  Q_ASSERT(!actuatorsWidget);
  actuatorsWidget = this;

  //setFocusPolicy(Qt::StrongFocus);
  QHBoxLayout* outerlayout = new QHBoxLayout(this);
  layout = new QHBoxLayout();
  outerlayout->setContentsMargins(0,0,0,0);
  outerlayout->addLayout(layout);
  setLayout(outerlayout);

  // load layout
  QSettings& settings = CoreModule::application->getLayoutSettings();
  settings.beginGroup("Actuators");
  QStringList openedActuators = settings.value("OpenedActuators").toStringList();
  settings.endGroup();
  foreach(QString actuator, openedActuators)
    openActuator(actuator);
}

ActuatorsWidget::~ActuatorsWidget()
{
  actuatorsWidget = 0;

  // save layout
  QSettings& settings = CoreModule::application->getLayoutSettings();
  settings.beginGroup("Actuators");
  settings.setValue("OpenedActuators", actuatorNames);
  settings.endGroup();
}

void ActuatorsWidget::openActuator(const QString& actuatorName)
{
  if(actuators.contains(actuatorName))
  {
    ActuatorWidget *widget = actuators.value(actuatorName);
    widget->setFocus();
    return;
  }

  SimRobotCore2::ActuatorPort* actuator = (SimRobotCore2::ActuatorPort*)CoreModule::application->resolveObject(actuatorName, SimRobotCore2::actuatorPort);
  if(!actuator)
    return;
  ActuatorWidget* widget = new ActuatorWidget(actuator, this);
  connect(widget, SIGNAL(releasedClose()), this, SLOT(closeActuator()));
  layout->addWidget(widget);
  actuators.insert(actuatorName, widget);
  actuatorNames.append(actuatorName);
}

void ActuatorsWidget::adoptActuators()
{
  foreach(ActuatorWidget* widget, actuators)
    widget->adoptActuator();
}

void ActuatorsWidget::closeActuator()
{
  ActuatorWidget* actuator = qobject_cast<ActuatorWidget*>(sender());
  if(!actuator)
    return;
 
  layout->removeWidget(actuator);
  actuators.remove(actuator->actuatorName);
  actuatorNames.removeOne(actuator->actuatorName);
  delete actuator;

  if(actuators.count() == 0)
    CoreModule::application->closeObject(CoreModule::module->actuatorsObject);
}
