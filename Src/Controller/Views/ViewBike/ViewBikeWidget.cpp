/**
* @file Controller/Views/ViewBike/ViewBikeWidget.cpp
*
* Implementation of class ViewBikeWidget.
*
* @author <a href="mailto:judy@tzi.de">Judith M�ller</a>
*/

#include "ViewBikeWidget.h"
#include "Tools/Streams/InStreams.h"
#include "Platform/BHAssert.h"
#include "Tools/Math/Vector2.h"

#include "Controller/RobotConsole.h"
#include "ViewBikeMath.h"

#include <vector>
#include <sstream>

#include <QHeaderView>
#include <QFileDialog>
#include <QGridLayout>
#include <QPalette>
#include <QColor>
#include <QLineEdit>
#include <QLabel>
#include <QToolButton>

ViewBikeWidget::ViewBikeWidget(ViewBike& viewBike, BIKEParameters& parameters, ViewBikeHeaderedWidget* parent) :
  QWidget(parent),
  parent(parent),
  glWidget(new ViewBikeGLWidget(viewBike, parameters, this)),
  viewBike(viewBike),
  parameters(parameters),
  phaseDrawings(true),
  singleDraw(false),
  reachedDraw(false),
  tra2dWindows(false),
  tra1dWindows(false),
  velocityWindows(false),
  accelWindows(false),
  followMode(false),
  dragPlane(0, 1, 0),
  getString(4),
  ghost(0),
  mirror(false)
{
  //Vertical Layouts
  QVBoxLayout* vbox = new QVBoxLayout(this);
  QVBoxLayout* vbox2 = new QVBoxLayout();
  QVBoxLayout* vbox3 = new QVBoxLayout();

  //horizontal Layouts
  QHBoxLayout* hbox = new QHBoxLayout();
  QHBoxLayout* hbox2 = new QHBoxLayout();

  //make the Button
  QPushButton* play = new QPushButton("Play Motion", this);
  QPushButton* play2 = new QPushButton("Play Motion until Active", this);
  QPushButton* reset = new QPushButton("Reset Robot", this);
  QPushButton* stop = new QPushButton("Stand", this);
  QPushButton* clay = new QPushButton("Record Pose", this);

  addPhase = new QPushButton("Add Phase", this);
  deletePhase = new QPushButton("Delete Phase", this);

  addPhase->setShortcut(tr("Shift+A"));
  deletePhase->setShortcut(tr("Shift+D"));
  play->setShortcut(tr("Shift+P"));
  play2->setShortcut(tr("Shift+Alt+P"));
  reset->setShortcut(tr("Shift+R"));
  stop->setShortcut(tr("Shift+T"));
  clay->setShortcut(tr("Shift+C"));

  addPhase->setToolTip("Shift+A");
  deletePhase->setToolTip("Shift+D");
  play->setToolTip("Shift+P");
  play2->setToolTip("Shift+Alt+P");
  reset->setToolTip("Shift+R");
  stop->setToolTip("Shift+T");
  clay->setToolTip("Shift+C");

  QPalette pal = play->palette();
  pal.setColor(QPalette::Background, Qt::darkGray);
  play->setPalette(pal);
  play->setBackgroundRole(QPalette::NoRole);
  play->setAutoFillBackground(true);
  play2->setPalette(pal);
  play2->setBackgroundRole(QPalette::NoRole);
  play2->setAutoFillBackground(true);
  reset->setPalette(pal);
  reset->setBackgroundRole(QPalette::NoRole);
  reset->setAutoFillBackground(true);
  stop->setPalette(pal);
  stop->setBackgroundRole(QPalette::NoRole);
  stop->setAutoFillBackground(true);
  clay->setPalette(pal);
  clay->setBackgroundRole(QPalette::NoRole);
  clay->setAutoFillBackground(true);
  addPhase->setPalette(pal);
  addPhase->setBackgroundRole(QPalette::NoRole);
  addPhase->setAutoFillBackground(true);
  deletePhase->setPalette(pal);
  deletePhase->setBackgroundRole(QPalette::NoRole);
  deletePhase->setAutoFillBackground(true);

  QSlider* slider1 = new QSlider(Qt::Horizontal, this);
  QLabel* label1 = new QLabel(tr("Robot Opacity"), this);

  //Set Sliderrange
  pal = slider1->palette();
  pal.setColor(QPalette::Background, Qt::darkGray);
  slider1->setPalette(pal);
  slider1->setAutoFillBackground(true);
  slider1->setRange(0, 16);

  pal = label1->palette();
  pal.setColor(QPalette::Background, Qt::darkGray);
  pal.setColor(QPalette::Foreground, Qt::white);
  label1->setPalette(pal);

  pal = palette();
  pal.setColor(QPalette::Background, Qt::darkGray);
  setPalette(pal);

  QCheckBox* mirrorCB = new QCheckBox(tr("Mirror"), this);
  mirrorCB->setCheckState(Qt::Unchecked);
  mirrorCB->setBackgroundRole(QPalette::NoRole);
  mirrorCB->setAutoFillBackground(true);
  pal = mirrorCB->palette();
  pal.setColor(QPalette::Background, Qt::darkGray);
  pal.setColor(QPalette::Foreground, Qt::white);
  mirrorCB->setPalette(pal);

  QToolButton* softLimb = new QToolButton(this);
  softLimb->setText("Soften Limb...");

  QMenu* limbmenu = new QMenu(this);

  lftra = new QAction(tr("Left Foot Translation off"), this);
  rftra = new QAction(tr("Right Foot Translation off"), this);
  lfrot = new QAction(tr("Left Foot Rotation off"), this);
  rfrot = new QAction(tr("Right Foot Rotation off"), this);
  lhtra = new QAction(tr("Left Hand Translation off"), this);
  rhtra = new QAction(tr("Right Hand Translation off"), this);
  lhrot = new QAction(tr("Left Hand Rotation off"), this);
  rhrot = new QAction(tr("Right Hand Rotation off"), this);

  lftra->setCheckable(true);
  rftra->setCheckable(true);
  lfrot->setCheckable(true);
  rfrot->setCheckable(true);
  lhtra->setCheckable(true);
  rhtra->setCheckable(true);
  lhrot->setCheckable(true);
  rhrot->setCheckable(true);

  lftra->setShortcut(tr("Shift+1"));
  rftra->setShortcut(tr("Shift+2"));
  lfrot->setShortcut(tr("Shift+3"));
  rfrot->setShortcut(tr("Shift+4"));
  lhtra->setShortcut(tr("Shift+5"));
  rhtra->setShortcut(tr("Shift+6"));
  lhrot->setShortcut(tr("Shift+7"));
  rhrot->setShortcut(tr("Shift+8"));

  limbmenu->addAction(lftra);
  limbmenu->addAction(rftra);
  limbmenu->addAction(lfrot);
  limbmenu->addAction(rfrot);
  limbmenu->addAction(lhtra);
  limbmenu->addAction(rhtra);
  limbmenu->addAction(lhrot);
  limbmenu->addAction(rhrot);

  softLimb->setMenu(limbmenu);
  softLimb->setPopupMode(QToolButton::InstantPopup);

  label1->setAutoFillBackground(true);
  mirrorCB->setAutoFillBackground(true);
  clay->setAutoFillBackground(true);
  play->setAutoFillBackground(true);
  play2->setAutoFillBackground(true);
  reset->setAutoFillBackground(true);
  stop->setAutoFillBackground(true);

  addPhase->setAutoFillBackground(true);
  deletePhase->setAutoFillBackground(true);

  vbox3->addWidget(mirrorCB, 0, Qt::AlignTop);
  vbox3->addWidget(play, 0, Qt::AlignTop);
  vbox3->addWidget(play2, 0, Qt::AlignTop);
  vbox3->addWidget(reset, 0, Qt::AlignTop);
  vbox3->addWidget(stop, 0, Qt::AlignTop);

  vbox3->addWidget(label1, 1, Qt::AlignBottom);
  vbox3->addWidget(slider1, 0, Qt::AlignBottom);
  vbox3->addWidget(addPhase, 0, Qt::AlignBottom);
  vbox3->addWidget(deletePhase, 0, Qt::AlignBottom);
  vbox3->addWidget(clay, 0, Qt::AlignBottom);
  vbox3->addWidget(softLimb, 1, Qt::AlignBottom);

  //Make the tabs
  tabber = new TabWidget(this);
  tabber->setAutoFillBackground(true);

  QPalette palette = tabber->palette();
  palette.setColor(QPalette::Background, Qt::darkGray);
  tabber->setPalette(palette);

  treeViewCommon = new QTreeView(this);
  treeViewCommon->setDragDropMode(QTreeView::DragOnly);
  treeViewCommon->setSelectionMode(QTreeView::ExtendedSelection);
  treeViewCommon->header()->setVisible(true);
  treeViewCommon->setRootIsDecorated(true);
  treeViewCommon->setDragEnabled(false);
  treeViewCommon->setAcceptDrops(false);

  //initialize Tabs with some working stuff
  modelCommon = new QStandardItemModel();

  QStandardItem* parentItem = modelCommon->invisibleRootItem();

  QList <QStandardItem*> nameLabels;
  nameLabels.append(makeLabel(QString("Name")));
  nameLabels.append(makeLabel(QString("newKick")));
  parentItem->appendRow(nameLabels);

  QList <QStandardItem*> phaseNumber;
  phaseNumber.append(makeLabel(QString("Number of Phases")));
  phaseNumber.append(makeLabel(QString::number(0)));
  parentItem->appendRow(phaseNumber);

  QStandardItem* footOrigin = makeLabel(QString("Foot Origin"));
  parentItem->appendRow(addXYZLabel(footOrigin));

  QList <QStandardItem*> list2;
  list2.append(makeLabel(QString("")));
  list2.append(makeLabel(QString("")));
  list2.append(makeValue((float) 0.f));
  list2.append(makeValue((float) 50.f));
  list2.append(makeValue((float) - 210.f));
  footOrigin->appendRow(list2);

  QStandardItem* footRotOrigin = makeLabel(QString("Foot Rot Origin"));
  parentItem->appendRow(addXYZLabel(footRotOrigin));

  QList <QStandardItem*> list21;
  list21.append(makeLabel(QString("")));
  list21.append(makeLabel(QString("")));
  list21.append(makeValue((float) 0.f));
  list21.append(makeValue((float) 0.f));
  list21.append(makeValue((float) 0.f));
  footRotOrigin->appendRow(list21);

  QStandardItem* armOrigin = makeLabel(QString("Hand Origin"));
  parentItem->appendRow(addXYZLabel(armOrigin));

  QList <QStandardItem*> list3;
  list3.append(makeLabel(QString("")));
  list3.append(makeLabel(QString("")));
  list3.append(makeValue((float) 0.f));
  list3.append(makeValue((float) 100.f));
  list3.append(makeValue((float) 30.f));
  armOrigin->appendRow(list3);

  QStandardItem* armRotOrigin = makeLabel(QString("Hand Rot Origin"));
  parentItem->appendRow(addXYZLabel(armRotOrigin));

  QList <QStandardItem*> list31;
  list31.append(makeLabel(QString("")));
  list31.append(makeLabel(QString("")));
  list31.append(makeValue((float) 0.f));
  list31.append(makeValue((float) 0.f));
  list31.append(makeValue((float) 0.f));
  armRotOrigin->appendRow(list31);

  QStandardItem* comOrigin = makeLabel(QString("COM Origin (only for no auto COM)"));
  QList <QStandardItem*> comOlist;
  comOlist.append(comOrigin);
  comOlist.append(makeLabel(QString("")));
  comOlist.append(makeLabel(QString("x")));
  comOlist.append(makeLabel(QString("y")));
  parentItem->appendRow(comOlist);
  QList <QStandardItem*> list33;
  list33.append(makeLabel(QString("")));
  list33.append(makeLabel(QString("")));
  list33.append(makeValue((float) 10.f));
  list33.append(makeValue((float) 0.f));
  comOrigin->appendRow(list33);

  QStandardItem* pidx = makeLabel(QString("COM Balance X"));
  QList <QStandardItem*> pidxlist;
  pidxlist.append(pidx);
  pidxlist.append(makeLabel(QString("")));
  pidxlist.append(makeLabel(QString("p")));
  pidxlist.append(makeLabel(QString("i")));
  pidxlist.append(makeLabel(QString("d")));
  parentItem->appendRow(pidxlist);
  QList <QStandardItem*> list4;
  list4.append(makeLabel(QString("")));
  list4.append(makeLabel(QString("")));
  list4.append(makeValue((float) 0.f));
  list4.append(makeValue((float) 0.f));
  list4.append(makeValue((float) 0.f));
  pidx->appendRow(list4);

  QStandardItem* pidy = makeLabel(QString("COM Balance Y"));
  QList <QStandardItem*> pidylist;
  pidylist.append(pidy);
  pidylist.append(makeLabel(QString("")));
  pidylist.append(makeLabel(QString("p")));
  pidylist.append(makeLabel(QString("i")));
  pidylist.append(makeLabel(QString("d")));
  parentItem->appendRow(pidylist);
  QList <QStandardItem*> list5;
  list5.append(makeLabel(QString("")));
  list5.append(makeLabel(QString("")));
  list5.append(makeValue((float) 0.f));
  list5.append(makeValue((float) 0.f));
  list5.append(makeValue((float) 0.f));
  pidy->appendRow(list5);

  parentItem->appendRow(makeStringAndValueRow(QString("Preview (in Frames)"), (int) 0));

  parentItem->appendRow(makeStringAndValueRow(QString("Loop"), (bool) 0));

  parentItem->appendRow(makeStringAndValueRow(QString("Auto COM"), (bool) 0));

  treeViewCommon->setModel(modelCommon);
  modelCommon->setColumnCount(5);
  modelCommon->setHeaderData(0, Qt::Horizontal,  QVariant(""));
  modelCommon->setHeaderData(1, Qt::Horizontal,  QVariant(""));
  modelCommon->setHeaderData(2, Qt::Horizontal,  QVariant(""));
  modelCommon->setHeaderData(3, Qt::Horizontal,  QVariant(""));
  modelCommon->setHeaderData(4, Qt::Horizontal,  QVariant(""));

  treeViewCommon->header()->setResizeMode(0, QHeaderView::ResizeToContents);
  treeViewCommon->header()->setResizeMode(1, QHeaderView::Stretch);
  treeViewCommon->header()->setResizeMode(2, QHeaderView::Stretch);
  treeViewCommon->header()->setResizeMode(3, QHeaderView::Stretch);
  treeViewCommon->header()->setResizeMode(4, QHeaderView::Stretch);

  tabber->addTab(treeViewCommon, "Common");

  hbox2->addWidget(tabber, 1);
  hbox2->addLayout(vbox3);
  hbox2->addWidget(glWidget, 3);

  hbox2->addLayout(vbox2);
  vbox->addLayout(hbox2);
  vbox->addLayout(hbox);

  treeViewCommon->expandAll();
  treeViewCommon->header()->setResizeMode(0, QHeaderView::ResizeToContents);

  bikeMenuBar = new BikeMenuBar(this);
  bikeMenuBar->hide();

  QMenu shortcutMenu;
  shortcutMenu.addMenu(bikeMenuBar->dragPlaneMenu);
  shortcutMenu.hide();

  //Connect Buttons and Stuff with
  connect(slider1, SIGNAL(valueChanged(int)), this, SLOT(transparencyChanged(int)));
  connect(deletePhase, SIGNAL(clicked()), this, SLOT(removePhase()));
  connect(addPhase, SIGNAL(clicked()), this, SLOT(addPhaseAfterActual()));
  connect(mirrorCB, SIGNAL(stateChanged(int)), this, SLOT(setMirrored(int)));
  connect(play, SIGNAL(clicked()), this, SLOT(playWholeMotion()));
  connect(reset, SIGNAL(clicked()), this, SLOT(resetRobot()));
  connect(stop, SIGNAL(clicked()), this, SLOT(standRobot()));
  connect(play2, SIGNAL(clicked()), this, SLOT(playMotionTilActive()));
  connect(clay, SIGNAL(clicked()), this, SLOT(recordPose()));

  connect(modelCommon, SIGNAL(itemChanged(QStandardItem*)), this, SLOT(updateCommonParameters(QStandardItem*)));
  connect(tabber->getTabBar(), SIGNAL(tabMoveRequested(int, int)), this, SLOT(movePhase(int, int)));
  connect(tabber, SIGNAL(currentChanged(int)), this, SLOT(setSelectedFromEditor(int)));

  dragPlaneMapper.setMapping(bikeMenuBar->xy_plane, SimRobotCore2::Renderer::xyPlane);
  dragPlaneMapper.setMapping(bikeMenuBar->xz_plane, SimRobotCore2::Renderer::xzPlane);
  dragPlaneMapper.setMapping(bikeMenuBar->yz_plane, SimRobotCore2::Renderer::yzPlane);
  connect(&dragPlaneMapper, SIGNAL(mapped(int)), SLOT(setDragPlane(int)));

  connect(bikeMenuBar->xy_plane, SIGNAL(triggered()), &dragPlaneMapper, SLOT(map()));
  connect(bikeMenuBar->xz_plane, SIGNAL(triggered()), &dragPlaneMapper, SLOT(map()));
  connect(bikeMenuBar->yz_plane, SIGNAL(triggered()), &dragPlaneMapper, SLOT(map()));

  qobject_cast<QAction*>(dragPlaneMapper.mapping(SimRobotCore2::Renderer::xzPlane))->setChecked(true);

  softenMapper.setMapping(lftra, 0);
  softenMapper.setMapping(rftra, 1);
  softenMapper.setMapping(lfrot, 2);
  softenMapper.setMapping(rfrot, 3);
  softenMapper.setMapping(lhtra, 4);
  softenMapper.setMapping(rhtra, 5);
  softenMapper.setMapping(lhrot, 6);
  softenMapper.setMapping(rhrot, 7);
  connect(&softenMapper, SIGNAL(mapped(int)), SLOT(setHardness(int)));
  connect(lftra, SIGNAL(toggled(bool)), &softenMapper, SLOT(map()));
  connect(rftra, SIGNAL(toggled(bool)), &softenMapper, SLOT(map()));
  connect(lfrot, SIGNAL(toggled(bool)), &softenMapper, SLOT(map()));
  connect(rfrot, SIGNAL(toggled(bool)), &softenMapper, SLOT(map()));
  connect(lhtra, SIGNAL(toggled(bool)), &softenMapper, SLOT(map()));
  connect(rhtra, SIGNAL(toggled(bool)), &softenMapper, SLOT(map()));
  connect(lhrot, SIGNAL(toggled(bool)), &softenMapper, SLOT(map()));
  connect(rhrot, SIGNAL(toggled(bool)), &softenMapper, SLOT(map()));

  //initialize actual Parameters
  parameters.footOrigin = Vector3 <>(0.f, 60.f, -210.f);
  parameters.armOrigin = Vector3 <>(0.f, 100.f, 30.f);
  parameters.numberOfPhases = 0;
  parameters.loop = false;
  parameters.kpx = 0.f;
  parameters.kdx = 0.f;
  parameters.kix = 0.f;
  parameters.kpy = 0.f;
  parameters.kdy = 0.f;
  parameters.kiy = 0.f;
  parameters.preview = 150;
  parameters.comOrigin = Vector2<>(10.f, 0.f);
  parameters.autoComTra = false;
  strcpy(parameters.name, "newKick");

  selectedPoint.limb = -1;
  selectedPoint.phaseNumber = -1;
  selectedPoint.xzRot = false;

  if(!commands.empty()) commands.clear();
}

ViewBikeWidget::~ViewBikeWidget()
{
  delete glWidget;
}

void ViewBikeWidget::setDragPlane(const int& plane)
{
  switch(plane)
  {
  case SimRobotCore2::Renderer::xyPlane:
    dragPlane = Vector3<>(0, 0, 1);
    break;
  case SimRobotCore2::Renderer::xzPlane:
    dragPlane = Vector3<>(0, 1, 0);
    break;
  case SimRobotCore2::Renderer::yzPlane:
    dragPlane = Vector3<>(1, 0, 0);
    break;
  default:
    break;
  }
  qobject_cast<QAction*>(dragPlaneMapper.mapping(plane))->setChecked(true);
}

void ViewBikeWidget::setHardness(int limb)
{
  switch(limb)
  {
  case 0: //lftra
    if(lftra->isChecked())
    {
      commands.push_back(std::string("set module:BIKE:lFootTraOff true"));
    }
    else
    {
      commands.push_back(std::string("set module:BIKE:lFootTraOff false"));
    }
    break;
  case 1: //rftra
    if(rftra->isChecked())
    {
      commands.push_back(std::string("set module:BIKE:rFootTraOff true"));
    }
    else
    {
      commands.push_back(std::string("set module:BIKE:rFootTraOff false"));
    }
    break;
  case 2: //lfrot
    if(lfrot->isChecked())
    {
      commands.push_back(std::string("set module:BIKE:lFootRotOff true"));
    }
    else
    {
      commands.push_back(std::string("set module:BIKE:lFootRotOff false"));
    }
    break;
  case 3: //rfrot
    if(rfrot->isChecked())
    {
      commands.push_back(std::string("set module:BIKE:rFootRotOff true"));
    }
    else
    {
      commands.push_back(std::string("set module:BIKE:rFootRotOff false"));
    }
    break;
  case 4: //lhtra
    if(lhtra->isChecked())
    {
      commands.push_back(std::string("set module:BIKE:lHandTraOff true"));
    }
    else
    {
      commands.push_back(std::string("set module:BIKE:lHandTraOff false"));
    }
    break;
  case 5: //rhtra
    if(rhtra->isChecked())
    {
      commands.push_back(std::string("set module:BIKE:rHandTraOff true"));
    }
    else
    {
      commands.push_back(std::string("set module:BIKE:rHandTraOff false"));
    }
    break;
  case 6: //lhrot
    if(lhrot->isChecked())
    {
      commands.push_back(std::string("set module:BIKE:lHandRotOff true"));
    }
    else
    {
      commands.push_back(std::string("set module:BIKE:lHandRotOff false"));
    }
    break;
  case 7: //rhrot
    if(rhrot->isChecked())
    {
      commands.push_back(std::string("set module:BIKE:rHandRotOff true"));
    }
    else
    {
      commands.push_back(std::string("set module:BIKE:rHandRotOff false"));
    }
    break;
  default:
    break;
  }
}

void ViewBikeWidget::removePhase()
{
  int phaseNumber = tabber->currentIndex() - 1;

  if(phaseNumber > -1)
  {
    parent->addStateToUndoList();
    std::vector<Phase>::iterator it;
    it = parameters.phaseParameters.begin() + phaseNumber;
    if(it != parameters.phaseParameters.end())
    {
      disconnect(*(phaseTab.begin() + phaseNumber), SIGNAL(itemChanged(QStandardItem*)), this, SLOT(updatePhaseParameters(QStandardItem*)));
      tabber->setCurrentIndex(phaseNumber);
      tabber->removeTab(phaseNumber + 1);
      for(int i = phaseNumber + 1; i < tabber->count(); i++)
      {
        tabber->setTabText(i, QString("Phase %1").arg(i - 1));
      }
      deleteKids(phaseTab[phaseNumber]->invisibleRootItem());
      delete *(treeView.begin() + phaseNumber);
      delete *(phaseTab.begin() + phaseNumber);
      treeView.erase(treeView.begin() + phaseNumber);
      phaseTab.erase(phaseTab.begin() + phaseNumber);
      parameters.phaseParameters.erase(it);
      parameters.numberOfPhases -= 1;

      for(int limb = 0; limb < Phase::numOfLimbs; limb++)
      {
        if(phaseNumber <  parameters.numberOfPhases - 1)
        {
          float factor = (float)parameters.phaseParameters[phaseNumber].duration /
                         (float)parameters.phaseParameters[phaseNumber + 1].duration;

          parameters.phaseParameters[phaseNumber + 1].controlPoints[limb][0] =
            parameters.phaseParameters[phaseNumber].controlPoints[limb][2] -
            parameters.phaseParameters[phaseNumber].controlPoints[limb][1];

          parameters.phaseParameters[phaseNumber + 1].controlPoints[limb][0] *= factor;

          parameters.phaseParameters[phaseNumber + 1].controlPoints[limb][0] +=
            parameters.phaseParameters[phaseNumber].controlPoints[limb][2];
        }
      }
      parameters.initFirstPhase();
      disconnect(modelCommon, SIGNAL(itemChanged(QStandardItem*)), this, SLOT(updateCommonParameters(QStandardItem*)));
      modelCommon->invisibleRootItem()->child(1, 1)->setData(QString::number(parameters.numberOfPhases), Qt::DisplayRole);
      updateCommon();
    }
  }
}

void ViewBikeWidget::movePhase(const int fromIndex, int toIndex)
{
  if(parameters.phaseParameters.begin() + fromIndex - 1 != parameters.phaseParameters.end())
  {
    toIndex = (fromIndex < toIndex) ? toIndex - 1 : toIndex;

    QStandardItemModel* temp = phaseTab[fromIndex - 1];
    phaseTab.erase(phaseTab.begin() + fromIndex - 1);
    phaseTab.insert(phaseTab.begin() + toIndex - 1, temp);

    Phase object = parameters.phaseParameters[fromIndex - 1];
    parameters.phaseParameters.erase(parameters.phaseParameters.begin() + fromIndex - 1);
    parameters.phaseParameters.insert(parameters.phaseParameters.begin() + toIndex - 1, object);
    parameters.initFirstPhase();
    updateEditorView();
    tabber->setCurrentIndex(toIndex);
  }
}

void ViewBikeWidget::makeNewPhaseWithModelAndTree(const int& phaseNumber)
{
  Phase newPhase;

  parameters.numberOfPhases++;
  disconnect(modelCommon, SIGNAL(itemChanged(QStandardItem*)), this, SLOT(updateCommonParameters(QStandardItem*)));
  modelCommon->invisibleRootItem()->child(1, 1)->setData(QString::number(parameters.numberOfPhases), Qt::DisplayRole);
  updateCommon();

  newPhase.duration = 1000;
  if(phaseNumber > 0)
  {
    for(int limb = 0; limb < Phase::numOfLimbs; ++limb)
      for(int point = 0; point < NUM_OF_POINTS; ++point)
        newPhase.controlPoints[limb][point] = Vector3 <> (0.f, 0.f, 0.f);
  }
  else
  {
    for(int point = 0; point < NUM_OF_POINTS; ++point)
    {
      newPhase.controlPoints[Phase::rightFootTra][point] = Vector3<>(parameters.footOrigin.x, -parameters.footOrigin.y, parameters.footOrigin.z);
      newPhase.controlPoints[Phase::leftFootTra][point] = parameters.footOrigin;
      newPhase.controlPoints[Phase::rightFootRot][point] = Vector3<>(-parameters.footRotOrigin.x, parameters.footRotOrigin.y, -parameters.footRotOrigin.z);
      newPhase.controlPoints[Phase::leftFootRot][point] = parameters.footRotOrigin;
      newPhase.controlPoints[Phase::rightArmTra][point] = Vector3<>(parameters.armOrigin.x, -parameters.armOrigin.y, parameters.armOrigin.z);
      newPhase.controlPoints[Phase::leftArmTra][point] = parameters.armOrigin;
      newPhase.controlPoints[Phase::rightHandRot][point] = Vector3<>(-parameters.handRotOrigin.x, parameters.handRotOrigin.y, -parameters.handRotOrigin.z);
      newPhase.controlPoints[Phase::leftHandRot][point] = parameters.handRotOrigin;
      newPhase.comTra[point] = parameters.comOrigin;
    }
  }

  parameters.phaseParameters.insert(parameters.phaseParameters.begin() + phaseNumber, newPhase);

  QStandardItemModel* model = makeNewModelWithLabels();
  phaseTab.insert(phaseTab.begin() + phaseNumber, model);
  fillModelWithPhaseData(phaseNumber);

  QTreeView* newTree = new QTreeView();
  newTree->setModel(phaseTab[phaseNumber]);
  newTree->setDragDropMode(QTreeView::DragOnly);
  newTree->setSelectionMode(QTreeView::ExtendedSelection);
  newTree->header()->setVisible(true);
  newTree->setRootIsDecorated(true);
  newTree->setDragEnabled(false);
  newTree->setAcceptDrops(false);
  newTree->expandToDepth(3);
  newTree->header()->setResizeMode(0, QHeaderView::ResizeToContents);
  newTree->header()->setResizeMode(1, QHeaderView::Stretch);
  newTree->header()->setResizeMode(2, QHeaderView::Stretch);
  newTree->header()->setResizeMode(3, QHeaderView::Stretch);
  newTree->header()->setResizeMode(4, QHeaderView::Stretch);
  newTree->header()->setResizeMode(5, QHeaderView::Stretch);
  treeView.insert(treeView.begin() + phaseNumber, newTree);
}

void ViewBikeWidget::addPhaseAfterActual()
{
  int phaseNumber = tabber->currentIndex(); // we want to insert after actual Phase

  parent->addStateToUndoList();

  makeNewPhaseWithModelAndTree(phaseNumber);

  tabber->insertTab(phaseNumber + 1, treeView[phaseNumber], QString("new"));

  if(phaseNumber > 0)  //if there is a phase before copy last parameters
  {
    for(int limb = 0; limb < Phase::numOfLimbs; ++limb)
    {
      //Set all Points from new Phase to the End of last Phase
      for(int point = 0; point < (int)NUM_OF_POINTS; ++point)
      {
        parameters.phaseParameters[phaseNumber].controlPoints[limb][point] = parameters.phaseParameters[phaseNumber - 1].controlPoints[limb][NUM_OF_POINTS - 1];
        parameters.phaseParameters[phaseNumber].comTra[point] = parameters.phaseParameters[phaseNumber - 1].comTra[NUM_OF_POINTS - 1];
      }

      //correct controlPoints
      // �quidistant und kolinear p3-p2 = q1-q0
      //p3 = q0 => p3 - p2 = q1 - p3 => 2*p3 - p2 = q1;

      //q1
      if(phaseNumber <  parameters.numberOfPhases - 1)
      {
        float factor = (float)parameters.phaseParameters[phaseNumber].duration /
                       (float)parameters.phaseParameters[phaseNumber + 1].duration;

        parameters.phaseParameters[phaseNumber + 1].controlPoints[limb][0] =
          parameters.phaseParameters[phaseNumber].controlPoints[limb][2] -
          parameters.phaseParameters[phaseNumber].controlPoints[limb][1];

        parameters.phaseParameters[phaseNumber + 1].controlPoints[limb][0] *= factor;

        parameters.phaseParameters[phaseNumber + 1].controlPoints[limb][0] +=
          parameters.phaseParameters[phaseNumber].controlPoints[limb][2];


        parameters.phaseParameters[phaseNumber + 1].comTra[0] =
          parameters.phaseParameters[phaseNumber].comTra[2] -
          parameters.phaseParameters[phaseNumber].comTra[1];

        parameters.phaseParameters[phaseNumber + 1].comTra[0] *= factor;

        parameters.phaseParameters[phaseNumber + 1].comTra[0] +=
          parameters.phaseParameters[phaseNumber].comTra[2];
      }
      //p1
      parameters.phaseParameters[phaseNumber].controlPoints[limb][0] =
        parameters.phaseParameters[phaseNumber - 1].controlPoints[limb][2] -
        parameters.phaseParameters[phaseNumber - 1].controlPoints[limb][1];

      float factor = (float)parameters.phaseParameters[phaseNumber].duration /
                     (float)parameters.phaseParameters[phaseNumber - 1].duration;

      parameters.phaseParameters[phaseNumber].controlPoints[limb][0] *= factor;

      parameters.phaseParameters[phaseNumber].controlPoints[limb][0] +=
        parameters.phaseParameters[phaseNumber - 1].controlPoints[limb][2];


      parameters.phaseParameters[phaseNumber].comTra[0] =
        parameters.phaseParameters[phaseNumber - 1].comTra[2] -
        parameters.phaseParameters[phaseNumber - 1].comTra[1];

      parameters.phaseParameters[phaseNumber].comTra[0] *= factor;

      parameters.phaseParameters[phaseNumber].comTra[0] +=
        parameters.phaseParameters[phaseNumber - 1].comTra[2];
    }
  }
  parameters.initFirstPhase();
  updateEditorView();
  tabber->setCurrentIndex(phaseNumber + 1);
}

std::string ViewBikeWidget::floatToStr(const float& f)
{
  char temp[100];
  sprintf(temp, "%.3f", f);
  return temp;
}

std::string ViewBikeWidget::intToStr(const int& i)
{
  char temp[100];
  sprintf(temp, "%i", i);
  return temp;
}

std::string ViewBikeWidget::boolToStr(const bool& b)
{
  if(b)
    return "true";
  else
    return "false";
}

void ViewBikeWidget::setMirrored(int state)
{
  mirror = (state == Qt::Checked);
}

void ViewBikeWidget::playWholeMotion()
{
  if(parameters.numberOfPhases > 0)
  {
    playMotion(parameters.numberOfPhases);
    std::stringstream todo(std::stringstream::in | std::stringstream::out);
    int idxMotion = viewBike.motionRequestCommand.find("motion");
    int idxSpecialAction = viewBike.motionRequestCommand.find("specialActionRequest", idxMotion);
    int idxBMotion = viewBike.motionRequestCommand.find("bMotionType", idxSpecialAction);
    int idxDynamical = viewBike.motionRequestCommand.find("dynamical", idxBMotion);
    todo << viewBike.motionRequestCommand.substr(0, idxMotion + 9) << "bike; ";
    todo << viewBike.motionRequestCommand.substr(idxSpecialAction, idxBMotion + 14 - idxSpecialAction) << "newKick; mirror = ";
    todo << (mirror ? "true" : "false") << "; " << viewBike.motionRequestCommand.substr(idxDynamical);
    commands.push_back(todo.str());
  }
}

void ViewBikeWidget::playMotionTilActive()
{
  int numberOfPhases = tabber->currentIndex();
  if(numberOfPhases > 0)
  {
    playMotion(numberOfPhases);

    std::stringstream todo(std::stringstream::in | std::stringstream::out);
    int idxMotion = viewBike.motionRequestCommand.find("motion");
    int idxSpecialAction = viewBike.motionRequestCommand.find("specialActionRequest", idxMotion);
    int idxBMotion = viewBike.motionRequestCommand.find("bMotionType", idxSpecialAction);
    int idxDynamical = viewBike.motionRequestCommand.find("dynamical", idxBMotion);
    todo << viewBike.motionRequestCommand.substr(0, idxMotion + 9) << "bike; ";
    todo << viewBike.motionRequestCommand.substr(idxSpecialAction, idxBMotion + 14 - idxSpecialAction) << "newKick; mirror = ";
    todo << (mirror ? "true" : "false") << "; " << viewBike.motionRequestCommand.substr(idxDynamical);
    commands.push_back(todo.str());
  }
}

void ViewBikeWidget::playMotion(int phase)
{
  std::string setNewBMotion = "set module:BIKE:newBMotion footOrigin = { x = "
                              + floatToStr(parameters.footOrigin.x) + "; y = "
                              + floatToStr(parameters.footOrigin.y) + "; z = "
                              + floatToStr(parameters.footOrigin.z) + "; }; footRotOrigin = { x = "
                              + floatToStr(parameters.footRotOrigin.x) + "; y = "
                              + floatToStr(parameters.footRotOrigin.y) + "; z = "
                              + floatToStr(parameters.footRotOrigin.z) + "; }; armOrigin = { x = "
                              + floatToStr(parameters.armOrigin.x)  + "; y = "
                              + floatToStr(parameters.armOrigin.y)  + "; z = "
                              + floatToStr(parameters.armOrigin.z)  + "; }; handRotOrigin = { x = "
                              + floatToStr(parameters.handRotOrigin.x)  + "; y = "
                              + floatToStr(parameters.handRotOrigin.y)  + "; z = "
                              + floatToStr(parameters.handRotOrigin.z)  + "; }; comOrigin = { x = "
                              + floatToStr(parameters.comOrigin.x)  + "; y = "
                              + floatToStr(parameters.comOrigin.y)  + "; }; kpx = "
                              + floatToStr(parameters.kpx)         + "; kix = "
                              + floatToStr(parameters.kix)         + "; kdx = "
                              + floatToStr(parameters.kdx)         + "; kpy = "
                              + floatToStr(parameters.kpy)         + "; kiy = "
                              + floatToStr(parameters.kiy)         + "; kdy = "
                              + floatToStr(parameters.kdy)         + "; preview = "
                              + floatToStr(parameters.preview)     + "; loop = "
                              + boolToStr(parameters.loop)          + "; autoComTra = "
                              + boolToStr(parameters.autoComTra)         + "; phaseParameters = [";

  for(int i = 0; i < phase; i++)
  {
    if(i)
      setNewBMotion += ", ";
    setNewBMotion += " { duration = "
                     + intToStr(parameters.phaseParameters[i].duration) + "; "
                     + "leftFootTra1 = { x = "
                     + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::leftFootTra][1].x) + "; y = "
                     + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::leftFootTra][1].y) + "; z = "
                     + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::leftFootTra][1].z) + "; }; "
                     + "leftFootTra2 = { x = "
                     + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::leftFootTra][2].x) + "; y = "
                     + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::leftFootTra][2].y) + "; z = "
                     + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::leftFootTra][2].z) + "; }; "
                     + "leftFootRot1 = { x = "
                     + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::leftFootRot][1].x) + "; y = "
                     + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::leftFootRot][1].y) + "; z = "
                     + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::leftFootRot][1].z) + "; }; "
                     + "leftFootRot2 = { x = "
                     + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::leftFootRot][2].x) + "; y = "
                     + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::leftFootRot][2].y) + "; z = "
                     + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::leftFootRot][2].z) + "; }; "
                     + "rightFootTra1 = { x = "
                     + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::rightFootTra][1].x) + "; y = "
                     + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::rightFootTra][1].y) + "; z = "
                     + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::rightFootTra][1].z) + "; }; "
                     + "rightFootTra2 = { x = "
                     + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::rightFootTra][2].x) + "; y = "
                     + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::rightFootTra][2].y) + "; z = "
                     + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::rightFootTra][2].z) + "; }; "
                     + "rightFootRot1 = { x = "
                     + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::rightFootRot][1].x) + "; y = "
                     + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::rightFootRot][1].y) + "; z = "
                     + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::rightFootRot][1].z) + "; }; "
                     + "rightFootRot2 = { x = "
                     + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::rightFootRot][2].x) + "; y = "
                     + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::rightFootRot][2].y) + "; z = "
                     + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::rightFootRot][2].z) + "; }; "
                     + "leftArmTra1 = { x = "
                     + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::leftArmTra][1].x) + "; y = "
                     + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::leftArmTra][1].y) + "; z = "
                     + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::leftArmTra][1].z) + "; }; "
                     + "leftArmTra2 = { x = "
                     + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::leftArmTra][2].x) + "; y = "
                     + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::leftArmTra][2].y) + "; z = "
                     + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::leftArmTra][2].z) + "; }; "
                     + "leftHandRot1 = { x = "
                     + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::leftHandRot][1].x) + "; y = "
                     + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::leftHandRot][1].y) + "; z = "
                     + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::leftHandRot][1].z) + "; }; "
                     + "leftHandRot2 = { x = "
                     + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::leftHandRot][2].x) + "; y = "
                     + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::leftHandRot][2].y) + "; z = "
                     + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::leftHandRot][2].z) + "; }; "
                     + "rightArmTra1 = { x = "
                     + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::rightArmTra][1].x) + "; y = "
                     + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::rightArmTra][1].y) + "; z = "
                     + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::rightArmTra][1].z) + "; }; "
                     + "rightArmTra2 = { x = "
                     + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::rightArmTra][2].x) + "; y = "
                     + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::rightArmTra][2].y) + "; z = "
                     + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::rightArmTra][2].z) + "; }; "
                     + "rightHandRot1 = { x = "
                     + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::rightHandRot][1].x) + "; y = "
                     + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::rightHandRot][1].y) + "; z = "
                     + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::rightHandRot][1].z) + "; }; "
                     + "rightHandRot2 = { x = "
                     + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::rightHandRot][2].x) + "; y = "
                     + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::rightHandRot][2].y) + "; z = "
                     + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::rightHandRot][2].z) + "; }; "
                     + "comTra1 = { x = "
                     + floatToStr(parameters.phaseParameters[i].comTra[1].x) + "; y = "
                     + floatToStr(parameters.phaseParameters[i].comTra[1].y) + "; }; "
                     + "comTra2 = { x = "
                     + floatToStr(parameters.phaseParameters[i].comTra[2].x) + "; y = "
                     + floatToStr(parameters.phaseParameters[i].comTra[2].y) + "; }; "
                     + "odometryOffset = { x = "
                     + floatToStr(parameters.phaseParameters[i].odometryOffset.x) + "; y = "
                     + floatToStr(parameters.phaseParameters[i].odometryOffset.y) + "; z = "
                     + floatToStr(parameters.phaseParameters[i].odometryOffset.z) + "; }; }" ;


  }

  setNewBMotion += " ];";
  commands.push_back(setNewBMotion);
}

void ViewBikeWidget::resetRobot()
{
  std::string moveRobot = "mv 0 0 300 0 0 180";
  commands.push_back(moveRobot);
  commands.push_back(" ");
}

void ViewBikeWidget::standRobot()
{
  std::string command = viewBike.motionRequestCommand;
  std::string firstOfcom = command.substr(0, command.find("motion") + 9);
  std::string temp = command.substr(command.find("motion") + 10, command.size() - command.find("motion") + 10);
  std::string endOfcom = temp.substr(temp.find_first_of(";"), temp.size());
  std::string stand = firstOfcom + "stand" + endOfcom;
  commands.push_back(stand);
  commands.push_back(" ");
}

void ViewBikeWidget::setSelectedFromEditor(const int& index)
{
  selectedPoint.phaseNumber = index - 1;
}

void ViewBikeWidget::setDrawings(bool value)
{
  if(value)
  {
    phaseDrawings = true;
  }
  else
  {
    phaseDrawings = false;
  }
}

void ViewBikeWidget::setSingleDrawing(bool value)
{
  if(value)
  {
    singleDraw = true;
  }
  else
  {
    singleDraw = false;
  }
}

void ViewBikeWidget::setReachedDrawing(bool value)
{
  if(value)
  {
    reachedDraw = true;
  }
  else
  {
    reachedDraw = false;
  }
}

void ViewBikeWidget::setTra2d(bool value)
{
  if(value)
  {
    tra2dWindows = true;
  }
  else
  {
    tra2dWindows = false;
  }
}

void ViewBikeWidget::setTra1d(bool value)
{
  if(value)
  {
    tra1dWindows = true;
  }
  else
  {
    tra1dWindows = false;
  }
}

void ViewBikeWidget::setVelocity(bool value)
{
  if(value)
  {
    velocityWindows = true;
  }
  else
  {
    velocityWindows = false;
  }
}

void ViewBikeWidget::setAccel(bool value)
{
  if(value)
  {
    accelWindows = true;
  }
  else
  {
    accelWindows = false;
  }
}

void ViewBikeWidget::setEditor(bool value)
{
  if(value)
  {
    tabber->setHidden(false);
  }
  else
  {
    tabber->hide();
  }
}

void ViewBikeWidget::setFollowMode(bool value)
{
  if(value)
  {
    followMode = true;
  }
  else
  {
    followMode = false;
  }
}


void ViewBikeWidget::transparencyChanged(const int& i)
{
  ghost = i;
}

void ViewBikeWidget::updateCommon()
{
  disconnect(modelCommon, SIGNAL(itemChanged(QStandardItem*)), this, SLOT(updateCommonParameters(QStandardItem*)));
  QStandardItem* rootItem  = modelCommon->invisibleRootItem();
  rootItem->child(0, 1)->setData(QString(parameters.name), Qt::DisplayRole);
  rootItem->child(1, 1)->setData(QString::number(parameters.numberOfPhases), Qt::DisplayRole);

  rootItem->child(2, 0)->child(0, 2)->setData(QVariant(parameters.footOrigin.x), Qt::DisplayRole);
  rootItem->child(2, 0)->child(0, 3)->setData(QVariant(parameters.footOrigin.y), Qt::DisplayRole);
  rootItem->child(2, 0)->child(0, 4)->setData(QVariant(parameters.footOrigin.z), Qt::DisplayRole);

  rootItem->child(3, 0)->child(0, 2)->setData(QVariant(parameters.footRotOrigin.x), Qt::DisplayRole);
  rootItem->child(3, 0)->child(0, 3)->setData(QVariant(parameters.footRotOrigin.y), Qt::DisplayRole);
  rootItem->child(3, 0)->child(0, 4)->setData(QVariant(parameters.footRotOrigin.z), Qt::DisplayRole);

  rootItem->child(4, 0)->child(0, 2)->setData(QVariant(parameters.armOrigin.x), Qt::DisplayRole);
  rootItem->child(4, 0)->child(0, 3)->setData(QVariant(parameters.armOrigin.y), Qt::DisplayRole);
  rootItem->child(4, 0)->child(0, 4)->setData(QVariant(parameters.armOrigin.z), Qt::DisplayRole);

  rootItem->child(5, 0)->child(0, 2)->setData(QVariant(parameters.handRotOrigin.x), Qt::DisplayRole);
  rootItem->child(5, 0)->child(0, 3)->setData(QVariant(parameters.handRotOrigin.y), Qt::DisplayRole);
  rootItem->child(5, 0)->child(0, 4)->setData(QVariant(parameters.handRotOrigin.z), Qt::DisplayRole);

  rootItem->child(6, 0)->child(0, 2)->setData(QVariant(parameters.comOrigin.x), Qt::DisplayRole);
  rootItem->child(6, 0)->child(0, 3)->setData(QVariant(parameters.comOrigin.y), Qt::DisplayRole);

  rootItem->child(7, 0)->child(0, 2)->setData(QVariant(parameters.kpx), Qt::DisplayRole);
  rootItem->child(7, 0)->child(0, 3)->setData(QVariant(parameters.kix), Qt::DisplayRole);
  rootItem->child(7, 0)->child(0, 4)->setData(QVariant(parameters.kdx), Qt::DisplayRole);

  rootItem->child(8, 0)->child(0, 2)->setData(QVariant(parameters.kpy), Qt::DisplayRole);
  rootItem->child(8, 0)->child(0, 3)->setData(QVariant(parameters.kiy), Qt::DisplayRole);
  rootItem->child(8, 0)->child(0, 4)->setData(QVariant(parameters.kdy), Qt::DisplayRole);

  rootItem->child(9, 1)->setData(QVariant(parameters.preview), Qt::DisplayRole);
  rootItem->child(10, 1)->setData(QVariant(parameters.loop), Qt::DisplayRole);
  rootItem->child(11, 1)->setData(QVariant(parameters.autoComTra), Qt::DisplayRole);

  connect(modelCommon, SIGNAL(itemChanged(QStandardItem*)), this, SLOT(updateCommonParameters(QStandardItem*)));
}


void ViewBikeWidget::updateEditorView()
{
  //Change Common Tab

  updateCommon();

  //Clear and Destroy and Resize
  for(int i = 0; i < tabber->count(); i++)
  {
    tabber->removeTab(i);
  }
  //this->repaint();
  tabber->addTab(treeViewCommon, "Common");
  treeViewCommon->expandAll();
  std::vector<QTreeView*>::iterator itTree;
  std::vector<QStandardItemModel*>::iterator itModel;

  if(!treeView.empty())
  {
    for(itTree = treeView.begin(); itTree < treeView.end(); itTree++)
    {
      delete *itTree;
    }
    treeView.clear();
  }

  if(!phaseTab.empty())
  {
    for(itModel = phaseTab.begin(); itModel < phaseTab.end(); itModel++)
    {
      deleteKids(((QStandardItemModel*)(*itModel))->invisibleRootItem());
      disconnect(((QStandardItemModel*)(*itModel)), SIGNAL(itemChanged(QStandardItem*)), this, SLOT(updatePhaseParameters(QStandardItem*)));
      delete *itModel;
    }
    phaseTab.clear();
  }

  treeView.resize(parameters.numberOfPhases);
  phaseTab.resize(parameters.numberOfPhases);

  //readIn possible Phases
  for(int i = 0; i < parameters.numberOfPhases; i++)
  {
    treeView[i] = new QTreeView();
    treeView[i]->setDragDropMode(QTreeView::DragOnly);
    treeView[i]->setSelectionMode(QTreeView::ExtendedSelection);
    treeView[i]->header()->setVisible(true);
    treeView[i]->setRootIsDecorated(true);
    phaseTab[i] = makeNewModelWithLabels();
    fillModelWithPhaseData(i);
    treeView[i]->setDragEnabled(false);
    treeView[i]->setAcceptDrops(false);
    treeView[i]->setModel(phaseTab[i]);
    treeView[i]->expandToDepth(3);
    treeView[i]->header()->setResizeMode(0, QHeaderView::Interactive);
    treeView[i]->header()->setResizeMode(1, QHeaderView::ResizeToContents);
    treeView[i]->header()->setResizeMode(2, QHeaderView::ResizeToContents);
    treeView[i]->header()->setResizeMode(3, QHeaderView::ResizeToContents);
    treeView[i]->header()->setResizeMode(4, QHeaderView::ResizeToContents);
    tabber->addTab(treeView[i], QString("Phase %1").arg(i));
  }

  updateGL();
}

QList <QStandardItem*> ViewBikeWidget::makeStringAndValueRow(QString string, QVariant value)
{
  QList <QStandardItem*> list;
  QStandardItem* item1, *item2;
  item1 = makeLabel(string);
  item2 = makeValue(value);
  list.append(item1);
  list.append(item2);
  return list;
}

void ViewBikeWidget::addStringAndValueToList(QList <QStandardItem*> &list, QString string, QVariant value)
{
  QStandardItem* item1, *item2;
  item1 = makeLabel(string);
  item2 = makeValue(value);
  list.append(item1);
  list.append(item2);
}

QStandardItem* ViewBikeWidget::makeLabel(QString string)
{
  QStandardItem* item = new QStandardItem(string);
  item->setEditable(false);
  item->setDragEnabled(false);
  item->setDropEnabled(false);
  return item;
}

QStandardItem* ViewBikeWidget::makeValue(QVariant var)
{
  QStandardItem* item = new QStandardItem();
  item->setData(var, Qt::DisplayRole);
  item->setDragEnabled(false);
  item->setDropEnabled(false);
  return item;
}

void ViewBikeWidget::addControlPoints(QStandardItem* item)
{
  QList <QStandardItem*> list;
  list.append(makeLabel(QString("Point0")));
  list.append(makeLabel(QString("")));

  for(int j = 0; j < 3; j++)
  {
    QStandardItem* it = new QStandardItem();
    it->setData((double)0.0, Qt::DisplayRole);
    it->setDragEnabled(false);
    it->setDropEnabled(false);
    it->setEditable(false);
    list.append(it);
  }

  item->appendRow(list);

  for(unsigned int k = 1; k < NUM_OF_POINTS; k++)
  {
    QList <QStandardItem*> list;
    list.append(makeLabel(QString("Point%1").arg(k)));
    list.append(makeLabel(QString("")));


    for(int i = 0; i < 3; i++)
    {
      list.append(makeValue((double) 0.0));
    }
    item->appendRow(list);
  }
}

QList <QStandardItem*> ViewBikeWidget::addXYZLabel(QStandardItem* item)
{
  QList <QStandardItem*> list;
  list.append(item);
  list.append(makeLabel(QString("")));
  list.append(makeLabel(QString("x")));
  list.append(makeLabel(QString("y")));
  list.append(makeLabel(QString("z")));

  return list;
}

QStandardItemModel* ViewBikeWidget::makeNewModelWithLabels()
{
  QStandardItemModel* model = new QStandardItemModel();
  model->setColumnCount(4);
  model->setHeaderData(0, Qt::Horizontal,  QVariant(" "));
  model->setHeaderData(1, Qt::Horizontal,  QVariant(" "));
  model->setHeaderData(2, Qt::Horizontal,  QVariant(" "));
  model->setHeaderData(3, Qt::Horizontal,  QVariant(" "));
  model->setHeaderData(4, Qt::Horizontal,  QVariant(" "));
  model->setHeaderData(5, Qt::Horizontal,  QVariant(" "));

  QStandardItem* parentItem = model->invisibleRootItem();

  parentItem->appendRow(makeStringAndValueRow(QString("Duration in ms"), (int) 0));

  QStandardItem* lFoot = makeLabel(QString("Left Foot"));
  parentItem->appendRow(lFoot);

  QStandardItem* lFootControlPointsTra = makeLabel(QString("Translation"));
  addControlPoints(lFootControlPointsTra);
  lFoot->appendRow(addXYZLabel(lFootControlPointsTra));

  QStandardItem* lFootControlPointsRot = makeLabel(QString("Rotation"));
  addControlPoints(lFootControlPointsRot);
  lFoot->appendRow(addXYZLabel(lFootControlPointsRot));

  QStandardItem* rFoot = makeLabel(QString("Right Foot"));
  parentItem->appendRow(rFoot);

  QStandardItem* rFootControlPointsTra = makeLabel(QString("Translation"));
  addControlPoints(rFootControlPointsTra);
  rFoot->appendRow(addXYZLabel(rFootControlPointsTra));


  QStandardItem* rFootControlPointsRot = makeLabel(QString("Rotation"));
  addControlPoints(rFootControlPointsRot);
  rFoot->appendRow(addXYZLabel(rFootControlPointsRot));

  QStandardItem* lArm = makeLabel(QString("Left Hand"));
  parentItem->appendRow(lArm);

  QStandardItem* lArmTra = makeLabel(QString("Translation"));
  addControlPoints(lArmTra);
  lArm->appendRow(addXYZLabel(lArmTra));

  QStandardItem* lArmRot = makeLabel(QString("Rotation"));
  addControlPoints(lArmRot);
  lArm->appendRow(addXYZLabel(lArmRot));


  QStandardItem* rArm = makeLabel(QString("Right Hand"));
  parentItem->appendRow(rArm);

  QStandardItem* rArmTra = makeLabel(QString("Translation"));
  addControlPoints(rArmTra);
  rArm->appendRow(addXYZLabel(rArmTra));

  QStandardItem* rArmRot = makeLabel(QString("Rotation"));
  addControlPoints(rArmRot);
  rArm->appendRow(addXYZLabel(rArmRot));

  QStandardItem* comTrajectory = makeLabel(QString("COM Trajectory (used only when no auto COM)"));
  parentItem->appendRow(comTrajectory);

  QStandardItem* COMTra = makeLabel(QString("Translation"));
  comTrajectory->appendRow(addXYZLabel(COMTra));

  for(unsigned int k = 0; k < NUM_OF_POINTS; k++)
  {
    QList <QStandardItem*> list;
    list.append(makeLabel(QString("Point%1").arg(k)));
    list.append(makeLabel(QString("")));
    for(int i = 0; i < 2; i++)
    {
      list.append(makeValue((double) 0.0));
    }
    COMTra->appendRow(list);
  }

  QList <QStandardItem*> list;
  QStandardItem* odometry = makeLabel(QString("Odometry Change"));
  list.append(odometry);
  list.append(makeLabel(QString("")));
  list.append(makeLabel(QString("x")));
  list.append(makeLabel(QString("y")));
  list.append(makeLabel(QString("rotZ")));
  parentItem->appendRow(list);

  QList <QStandardItem*> list2;
  list2.append(makeLabel(QString("")));
  list2.append(makeLabel(QString("")));
  list2.append(makeValue(0.f));
  list2.append(makeValue(0.f));
  list2.append(makeValue(0.f));
  odometry->appendRow(list2);

  return model;
}


bool ViewBikeWidget::deleteKids(QStandardItem* rootItem)
{
  if(rootItem->hasChildren())
  {
    for(int i = 0; i < rootItem->rowCount(); i++)
    {
      deleteKids(rootItem->child(i, 0));
    }
    rootItem->removeRows(0, rootItem->rowCount());
  }
  return true;
}

void ViewBikeWidget::fillModelWithPhaseData(int i)
{
  if(i < parameters.numberOfPhases)
  {
    QStandardItem* rootItem = phaseTab[i]->invisibleRootItem();

    disconnect(phaseTab[i], SIGNAL(itemChanged(QStandardItem*)), this, SLOT(updatePhaseParameters(QStandardItem*)));

    //Duration im ms
    rootItem->child(0, 1)->setData(QVariant(parameters.phaseParameters[i].duration), Qt::DisplayRole);

    for(unsigned int k = 0; k < NUM_OF_POINTS; k++)
    {
      rootItem->child(5, 0)->child(0, 0)->child(k, 2)->setData(QVariant(parameters.phaseParameters[i].comTra[k].x), Qt::DisplayRole);
      rootItem->child(5, 0)->child(0, 0)->child(k, 3)->setData(QVariant(parameters.phaseParameters[i].comTra[k].y), Qt::DisplayRole);
    }

    rootItem->child(6, 0)->child(0, 2)->setData(QVariant(parameters.phaseParameters[i].odometryOffset.x), Qt::DisplayRole);
    rootItem->child(6, 0)->child(0, 3)->setData(QVariant(parameters.phaseParameters[i].odometryOffset.y), Qt::DisplayRole);
    rootItem->child(6, 0)->child(0, 4)->setData(QVariant(parameters.phaseParameters[i].odometryOffset.z), Qt::DisplayRole);

    int limb = 0, child1 = 1, child2 = 0;
    QStandardItem* item;
    while(limb < Phase::numOfLimbs)
    {
      switch(limb)
      {
      case 0:
        child1 = 1;
        child2 = 0;
        break;
      case 1:
        child1 = 1;
        child2 = 1;
        break;
      case 2:
        child1 = 2;
        child2 = 0;
        break;
      case 3:
        child1 = 2;
        child2 = 1;
        break;
      case 4:
        child1 = 3;
        child2 = 0;
        break;
      case 5:
        child1 = 3;
        child2 = 1;
        break;
      case 6:
        child1 = 4;
        child2 = 0;
        break;
      case 7:
        child1 = 4;
        child2 = 1;
        break;
      default:
        limb = 100;
        break;
      }

      for(unsigned int k = 0; k < NUM_OF_POINTS; k++)
      {
        item = rootItem->child(child1, 0)->child(child2, 0);
        item->child(k, 2)->setData(QVariant(parameters.phaseParameters[i].controlPoints[limb][k].x), Qt::DisplayRole);
        item->child(k, 3)->setData(QVariant(parameters.phaseParameters[i].controlPoints[limb][k].y), Qt::DisplayRole);
        item->child(k, 4)->setData(QVariant(parameters.phaseParameters[i].controlPoints[limb][k].z), Qt::DisplayRole);
      }
      limb++;
    }
    connect(phaseTab[i], SIGNAL(itemChanged(QStandardItem*)), this, SLOT(updatePhaseParameters(QStandardItem*)));
  }
}

void ViewBikeWidget::updateCommonParameters(QStandardItem* item)
{
  parent->addStateToUndoList();

  QStandardItem* mamaItem = item->parent();
  int row = item->row();
  if(!mamaItem)
  {
    if(row == 9) parameters.preview = item->data(Qt::DisplayRole).toInt();
    if(row == 10) parameters.loop = item->data(Qt::DisplayRole).toInt();
    if(row == 11) parameters.autoComTra = item->data(Qt::DisplayRole).toInt();
  }
  else
  {
    int col = item->column();
    if(QString("Foot Origin") == mamaItem->data(Qt::DisplayRole))
    {
      if(col == 2)
      {
        float diff = (float)item->data(Qt::DisplayRole).toDouble();
        diff -= parameters.footOrigin.x;
        parameters.footOrigin.x += diff;
        for(int p = 0; p < parameters.numberOfPhases; ++p)
          for(int k = 0; k < 3; ++k)
          {
            parameters.phaseParameters[p].controlPoints[Phase::leftFootTra][k].x += diff;
            parameters.phaseParameters[p].controlPoints[Phase::rightFootTra][k].x += diff;
          }
      }
      if(col == 3)
      {
        float diff = (float)item->data(Qt::DisplayRole).toDouble();
        diff -= parameters.footOrigin.y;
        parameters.footOrigin.y += diff;
        for(int p = 0; p < parameters.numberOfPhases; ++p)
          for(int k = 0; k < 3; ++k)
          {
            parameters.phaseParameters[p].controlPoints[Phase::leftFootTra][k].y += diff;
            parameters.phaseParameters[p].controlPoints[Phase::rightFootTra][k].y -= diff;
          }
      }
      if(col == 4)
      {
        float diff = (float)item->data(Qt::DisplayRole).toDouble();
        diff -= parameters.footOrigin.z;
        parameters.footOrigin.z += diff;
        for(int p = 0; p < parameters.numberOfPhases; ++p)
          for(int k = 0; k < 3; ++k)
          {
            parameters.phaseParameters[p].controlPoints[Phase::leftFootTra][k].z += diff;
            parameters.phaseParameters[p].controlPoints[Phase::rightFootTra][k].z += diff;
          }
      }
    }
    if(QString("Foot Rot Origin") == mamaItem->data(Qt::DisplayRole))
    {
      if(col == 2)
      {
        float diff = (float)item->data(Qt::DisplayRole).toDouble();
        diff -= parameters.footRotOrigin.x;
        parameters.footRotOrigin.x += diff;
        for(int p = 0; p < parameters.numberOfPhases; ++p)
          for(int k = 0; k < 3; ++k)
          {
            parameters.phaseParameters[p].controlPoints[Phase::leftFootRot][k].x += diff;
            parameters.phaseParameters[p].controlPoints[Phase::rightFootRot][k].x -= diff;
          }
      }
      if(col == 3)
      {
        float diff = (float)item->data(Qt::DisplayRole).toDouble();
        diff -= parameters.footRotOrigin.y;
        parameters.footRotOrigin.y += diff;
        for(int p = 0; p < parameters.numberOfPhases; ++p)
          for(int k = 0; k < 3; ++k)
          {
            parameters.phaseParameters[p].controlPoints[Phase::leftFootRot][k].y += diff;
            parameters.phaseParameters[p].controlPoints[Phase::rightFootRot][k].y += diff;
          }
      }
      if(col == 4)
      {
        float diff = (float)item->data(Qt::DisplayRole).toDouble();
        diff -= parameters.footRotOrigin.z;
        parameters.footRotOrigin.z += diff;
        for(int p = 0; p < parameters.numberOfPhases; ++p)
          for(int k = 0; k < 3; ++k)
          {
            parameters.phaseParameters[p].controlPoints[Phase::leftFootRot][k].z += diff;
            parameters.phaseParameters[p].controlPoints[Phase::rightFootRot][k].z -= diff;
          }
      }
    }
    if(QString("Hand Origin") == mamaItem->data(Qt::DisplayRole))
    {
      if(col == 2)
      {
        float diff = (float)item->data(Qt::DisplayRole).toDouble();
        diff -= parameters.armOrigin.x;
        parameters.armOrigin.x += diff;
        for(int p = 0; p < parameters.numberOfPhases; ++p)
          for(int k = 0; k < 3; ++k)
          {
            parameters.phaseParameters[p].controlPoints[Phase::leftArmTra][k].x += diff;
            parameters.phaseParameters[p].controlPoints[Phase::rightArmTra][k].x += diff;
          }
      }
      if(col == 3)
      {
        float diff = (float)item->data(Qt::DisplayRole).toDouble();
        diff -= parameters.armOrigin.y;
        parameters.armOrigin.y += diff;
        for(int p = 0; p < parameters.numberOfPhases; ++p)
          for(int k = 0; k < 3; ++k)
          {
            parameters.phaseParameters[p].controlPoints[Phase::leftArmTra][k].y += diff;
            parameters.phaseParameters[p].controlPoints[Phase::rightArmTra][k].y -= diff;
          }
      }
      if(col == 4)
      {
        float diff = (float)item->data(Qt::DisplayRole).toDouble();
        diff -= parameters.armOrigin.z;
        parameters.armOrigin.z += diff;
        for(int p = 0; p < parameters.numberOfPhases; ++p)
          for(int k = 0; k < 3; ++k)
          {
            parameters.phaseParameters[p].controlPoints[Phase::leftArmTra][k].z += diff;
            parameters.phaseParameters[p].controlPoints[Phase::rightArmTra][k].z += diff;
          }
      }
    }
    if(QString("Hand Rot Origin") == mamaItem->data(Qt::DisplayRole))
    {
      if(col == 2)
      {
        float diff = (float)item->data(Qt::DisplayRole).toDouble();
        diff -= parameters.handRotOrigin.x;
        parameters.handRotOrigin.x += diff;
        for(int p = 0; p < parameters.numberOfPhases; ++p)
          for(int k = 0; k < 3; ++k)
          {
            parameters.phaseParameters[p].controlPoints[Phase::leftHandRot][k].x += diff;
            parameters.phaseParameters[p].controlPoints[Phase::rightHandRot][k].x -= diff;
          }
      }
      if(col == 3)
      {
        float diff = (float)item->data(Qt::DisplayRole).toDouble();
        diff -= parameters.handRotOrigin.y;
        parameters.handRotOrigin.y += diff;
        for(int p = 0; p < parameters.numberOfPhases; ++p)
          for(int k = 0; k < 3; ++k)
          {
            parameters.phaseParameters[p].controlPoints[Phase::leftHandRot][k].y += diff;
            parameters.phaseParameters[p].controlPoints[Phase::rightHandRot][k].y += diff;
          }
      }
      if(col == 4)
      {
        float diff = (float)item->data(Qt::DisplayRole).toDouble();
        diff -= parameters.handRotOrigin.z;
        parameters.handRotOrigin.z += diff;
        for(int p = 0; p < parameters.numberOfPhases; ++p)
          for(int k = 0; k < 3; ++k)
          {
            parameters.phaseParameters[p].controlPoints[Phase::leftHandRot][k].z += diff;
            parameters.phaseParameters[p].controlPoints[Phase::rightHandRot][k].z -= diff;
          }
      }
    }
    if(QString("COM Origin (only for no auto COM)") == mamaItem->data(Qt::DisplayRole))
    {
      if(col == 2)
      {
        float diff = (float)item->data(Qt::DisplayRole).toDouble();
        diff -= parameters.comOrigin.x;
        parameters.comOrigin.x += diff;
        for(int p = 0; p < parameters.numberOfPhases; ++p)
          for(int k = 0; k < 3; ++k)
          {
            parameters.phaseParameters[p].comTra[k].x += diff;
          }
      }
      if(col == 3)
      {
        float diff = (float)item->data(Qt::DisplayRole).toDouble();
        diff -= parameters.comOrigin.y;
        parameters.comOrigin.y += diff;
        for(int p = 0; p < parameters.numberOfPhases; ++p)
          for(int k = 0; k < 3; ++k)
          {
            parameters.phaseParameters[p].comTra[k].y += diff;
          }
      }
    }
    if(QString("COM Balance X") == mamaItem->data(Qt::DisplayRole))
    {
      if(col == 2) parameters.kpx = (float)item->data(Qt::DisplayRole).toDouble();
      if(col == 3) parameters.kix = (float)item->data(Qt::DisplayRole).toDouble();
      if(col == 4) parameters.kdx = (float)item->data(Qt::DisplayRole).toDouble();
    }
    if(QString("COM Balance Y") == mamaItem->data(Qt::DisplayRole))
    {
      if(col == 2) parameters.kpy = (float)item->data(Qt::DisplayRole).toDouble();
      if(col == 3) parameters.kiy = (float)item->data(Qt::DisplayRole).toDouble();
      if(col == 4) parameters.kdy = (float)item->data(Qt::DisplayRole).toDouble();
    }
  }

  updateEditorView();
  parameters.initFirstPhase();
}

void ViewBikeWidget::updatePhaseParameters(QStandardItem* item)
{
  QStandardItem* mamaItem = item->parent();
  int row = item->row();
  int phaseNumber = tabber->currentIndex() - 1;
  int col = item->column();
  int limb = -1;

  parent->addStateToUndoList();

  if(!mamaItem)
  {
    if(row == 0) parameters.phaseParameters[phaseNumber].duration = item->data(Qt::DisplayRole).toInt();
  }
  else
  {
    if(QString("Left Foot") == mamaItem->parent()->data(Qt::DisplayRole))
    {
      if(QString("Translation") == mamaItem->data(Qt::DisplayRole))
      {
        limb = Phase::leftFootTra;
      }
      if(QString("Rotation") == mamaItem->data(Qt::DisplayRole))
      {
        limb = Phase::leftFootRot;
      }
    }

    if(QString("Right Foot") == mamaItem->parent()->data(Qt::DisplayRole))
    {
      if(QString("Translation") == mamaItem->data(Qt::DisplayRole))
      {
        limb = Phase::rightFootTra;

      }
      if(QString("Rotation") == mamaItem->data(Qt::DisplayRole))
      {
        limb = Phase::rightFootRot;
      }
    }
    if(QString("Right Hand") == mamaItem->parent()->data(Qt::DisplayRole))
    {
      if(QString("Translation") == mamaItem->data(Qt::DisplayRole))
      {
        limb = Phase::rightArmTra;
      }
      if(QString("Rotation") == mamaItem->data(Qt::DisplayRole))
      {
        limb = Phase::rightHandRot;
      }
    }
    if(QString("Left Hand") == mamaItem->parent()->data(Qt::DisplayRole))
    {
      if(QString("Translation") == mamaItem->data(Qt::DisplayRole))
      {
        limb = Phase::leftArmTra;

      }
      if(QString("Rotation") == mamaItem->data(Qt::DisplayRole))
      {
        limb = Phase::leftHandRot;
      }
    }

    if(QString("COM Trajectory (used only when no auto COM)") == mamaItem->parent()->data(Qt::DisplayRole))
    {
      switch(col)
      {
      case 2:
        parameters.phaseParameters[phaseNumber].comTra[row].x = (float)item->data(Qt::DisplayRole).toDouble();
        break;
      case 3:
        parameters.phaseParameters[phaseNumber].comTra[row].y = (float)item->data(Qt::DisplayRole).toDouble();
        break;
      default:
        break;
      }
    }

    if(QString("Odometry Change") == mamaItem->parent()->data(Qt::DisplayRole))
    {
      switch(col)
      {
      case 2:
        parameters.phaseParameters[phaseNumber].odometryOffset.x = (float)item->data(Qt::DisplayRole).toDouble();
        break;
      case 3:
        parameters.phaseParameters[phaseNumber].odometryOffset.y = (float)item->data(Qt::DisplayRole).toDouble();
        break;
      case 4:
        parameters.phaseParameters[phaseNumber].odometryOffset.z = (float)item->data(Qt::DisplayRole).toDouble();
        break;
      default:
        break;
      }
    }
  }

  selectedPoint.limb = limb;
  if(limb > -1)
  {
    if(row < NUM_OF_POINTS)
    {
      selectedPoint.pointNumber = row;
      switch(col)
      {
      case 2:
        parameters.phaseParameters[phaseNumber].controlPoints[limb][row].x = (float)item->data(Qt::DisplayRole).toDouble();
        break;
      case 3:
        parameters.phaseParameters[phaseNumber].controlPoints[limb][row].y = (float)item->data(Qt::DisplayRole).toDouble();
        break;
      case 4:
        parameters.phaseParameters[phaseNumber].controlPoints[limb][row].z = (float)item->data(Qt::DisplayRole).toDouble();
        break;
      default:
        break;
      }
    }
  }
  parameters.initFirstPhase();
}


void ViewBikeWidget::recordPose()
{
  int phaseNumber = tabber->currentIndex() - 1;
  const JointData& jointData = viewBike.jointData;

  if(phaseNumber > -1)
  {

    Pose3D p;

    if(lhtra->isChecked())
    {
      p =  ViewBikeMath::calculateHandPos(viewBike.jointData, JointData::LShoulderPitch, viewBike.robotDimensions);

      parameters.phaseParameters[phaseNumber].controlPoints[Phase::leftArmTra][2] = p.translation;
      parameters.phaseParameters[phaseNumber].controlPoints[Phase::leftArmTra][1] = p.translation;
    }

    if(rhtra->isChecked())
    {
      p =  ViewBikeMath::calculateHandPos(viewBike.jointData, JointData::RShoulderPitch, viewBike.robotDimensions);

      parameters.phaseParameters[phaseNumber].controlPoints[Phase::rightArmTra][2] = p.translation;
      parameters.phaseParameters[phaseNumber].controlPoints[Phase::rightArmTra][1] = p.translation;
    }

    if(lhrot->isChecked())
    {
      p =  ViewBikeMath::calculateHandPos(viewBike.jointData, JointData::LShoulderPitch, viewBike.robotDimensions);

      parameters.phaseParameters[phaseNumber].controlPoints[Phase::leftHandRot][2] = p.rotation.getAngleAxis();
      parameters.phaseParameters[phaseNumber].controlPoints[Phase::leftHandRot][1] = p.rotation.getAngleAxis();
    }

    if(rhrot->isChecked())
    {
      p =  ViewBikeMath::calculateHandPos(viewBike.jointData, JointData::RShoulderPitch, viewBike.robotDimensions);

      parameters.phaseParameters[phaseNumber].controlPoints[Phase::rightHandRot][2] = p.rotation.getAngleAxis();
      parameters.phaseParameters[phaseNumber].controlPoints[Phase::rightHandRot][1] = p.rotation.getAngleAxis();
    }

    if(lfrot->isChecked())
    {

      float sign = 1.f;
      JointData::Joint joint = JointData::LHipYawPitch;

      RotationMatrix rotateBecauseOfHip = RotationMatrix().rotateZ(jointData.angles[joint]).rotateX(-sign * pi_4);
      RotationMatrix footRot = RotationMatrix().rotateX((jointData.angles[joint + 1] + pi_4) * -sign).rotateY(jointData.angles[joint + 2] + jointData.angles[joint + 3]);
      footRot = footRot.invert() * rotateBecauseOfHip;

      float leg5 = jointData.angles[joint + 5] - (asin(-footRot[2].y) * sign * -1);
      float leg4 = jointData.angles[joint + 4] - (-atan2(footRot[2].x, footRot[2].z) * -1);

      parameters.phaseParameters[phaseNumber].controlPoints[Phase::leftFootRot][2].x = leg5;
      parameters.phaseParameters[phaseNumber].controlPoints[Phase::leftFootRot][2].y = leg4;

      parameters.phaseParameters[phaseNumber].controlPoints[Phase::leftFootRot][1].x = leg5;
      parameters.phaseParameters[phaseNumber].controlPoints[Phase::leftFootRot][1].y = leg4;

      parameters.phaseParameters[phaseNumber].controlPoints[Phase::leftFootRot][2].z = jointData.angles[joint] * sign;
      parameters.phaseParameters[phaseNumber].controlPoints[Phase::leftFootRot][1].z = jointData.angles[joint] * sign;

      parameters.phaseParameters[phaseNumber].controlPoints[Phase::rightFootRot][2].z = jointData.angles[joint] * -sign;
      parameters.phaseParameters[phaseNumber].controlPoints[Phase::rightFootRot][1].z = jointData.angles[joint] * -sign;

    }

    if(rfrot->isChecked())
    {

      float sign = -1.f;
      JointData::Joint joint = JointData::RHipYawPitch;

      RotationMatrix rotateBecauseOfHip = RotationMatrix().rotateZ(jointData.angles[joint] * sign).rotateX(-sign * pi_4);
      RotationMatrix footRot = RotationMatrix().rotateX((jointData.angles[joint + 1] + pi_4) * -sign).rotateY(jointData.angles[joint + 2] + jointData.angles[joint + 3]);
      footRot = footRot.invert() * rotateBecauseOfHip;

      float leg5 = jointData.angles[joint + 5] - (asin(-footRot[2].y) * sign * -1);
      float leg4 = jointData.angles[joint + 4] - (-atan2(footRot[2].x, footRot[2].z) * -1);

      parameters.phaseParameters[phaseNumber].controlPoints[Phase::rightFootRot][2].x = leg5;
      parameters.phaseParameters[phaseNumber].controlPoints[Phase::rightFootRot][2].y = leg4;

      parameters.phaseParameters[phaseNumber].controlPoints[Phase::rightFootRot][1].x = leg5;
      parameters.phaseParameters[phaseNumber].controlPoints[Phase::rightFootRot][1].y = leg4;

      parameters.phaseParameters[phaseNumber].controlPoints[Phase::leftFootRot][2].z = jointData.angles[joint] * -sign;
      parameters.phaseParameters[phaseNumber].controlPoints[Phase::leftFootRot][1].z = jointData.angles[joint] * -sign;

      parameters.phaseParameters[phaseNumber].controlPoints[Phase::rightFootRot][2].z = jointData.angles[joint] * sign;
      parameters.phaseParameters[phaseNumber].controlPoints[Phase::rightFootRot][1].z = jointData.angles[joint] * sign;
    }

    if(lftra->isChecked())
    {
      p =  ViewBikeMath::calculateFootPos(viewBike.sensorData, viewBike.jointData, JointData::LHipYawPitch, viewBike.robotDimensions).translation;

      parameters.phaseParameters[phaseNumber].controlPoints[Phase::leftFootTra][2] = p.translation;
      parameters.phaseParameters[phaseNumber].controlPoints[Phase::leftFootTra][1] = p.translation;
    }
    if(rftra->isChecked())
    {
      p =  ViewBikeMath::calculateFootPos(viewBike.sensorData, viewBike.jointData, JointData::RHipYawPitch, viewBike.robotDimensions).translation;

      parameters.phaseParameters[phaseNumber].controlPoints[Phase::rightFootTra][2] = p.translation;
      parameters.phaseParameters[phaseNumber].controlPoints[Phase::rightFootTra][1] = p.translation;
    }

    updateCommon();
    for(int i = 0; i < parameters.numberOfPhases; i++)
    {
      fillModelWithPhaseData(i);
      treeView[i]->expandToDepth(3);
    }
  }
}
