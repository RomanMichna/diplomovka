/**
* @file USObstacleGridProvider.cpp
*
* This file implements a module that provides information about occupied space in the robot's environment.
* The module computes an occupancy grid based on ultrasonic measurements.
* It includes parts of the implementation of the PolygonLocalMapper module of the
* autonomous wheelchair Rolland.
*
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
* @author <a href="mailto:cman@tzi.de">Christian Mandel</a>
*/

#include <algorithm>
#include "USObstacleGridProvider.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Settings.h"

const int CELL_SIZE   = USObstacleGrid::CELL_SIZE;
const int GRID_LENGTH = USObstacleGrid::GRID_LENGTH;
const int GRID_SIZE   = USObstacleGrid::GRID_SIZE;;


USObstacleGridProvider::USObstacleGridProvider():
  lastTimePenalized(0), measuredCenterLeft(false), measuredCenterRight(false), 
  lastUsActuatorMode(SensorData::UsActuatorMode(-1)), lastUsTimeStamp(0), initialized(false), gameInfoGameStateLastFrame(STATE_INITIAL)
{
  InConfigMap stream(Global::getSettings().expandLocationFilename("usObstacleGrid.cfg"));
  if(stream.exists())
    stream >> parameters;
  ASSERT(USObstacleGrid::GRID_LENGTH / 2 * USObstacleGrid::CELL_SIZE > parameters.maxValidUSDist + sqrt(2.f) * CELL_SIZE);
}

void USObstacleGridProvider::update(USObstacleGrid& usObstacleGrid)
{
  if(!initialized)
  {
    lastOdometry = theOdometryData;
    cells = usObstacleGrid.cells;
    for(int i = 0; i < GRID_SIZE; ++i)
      cells[i].state = 0;
    initialized = true;
  }
  else if((gameInfoGameStateLastFrame == STATE_SET) && (theGameInfo.state == STATE_PLAYING))
  {
    for(int i = 0; i < GRID_SIZE; ++i)
      cells[i].state = 0;
  }
  MODIFY("parameters:USObstacleGridProvider", parameters);
  DECLARE_DEBUG_DRAWING("module:USObstacleGridProvider:us", "drawingOnField");
  ageCellState();
  if((theRobotInfo.penalty == PENALTY_NONE) &&
     (theMotionRequest.motion != MotionRequest::specialAction) &&
     (theMotionRequest.motion != MotionRequest::bike) &&
     (theFrameInfo.getTimeSince(lastTimePenalized) > 3000))
  {
    moveGrid();
    checkUS();
  }
  else if(theRobotInfo.penalty != PENALTY_NONE)
  {
    lastTimePenalized = theFrameInfo.time;
  }
  usObstacleGrid.gridOffset = Pose2D(-accumulatedOdometry.rotation, -accumulatedOdometry.translation);
  usObstacleGrid.cellOccupiedThreshold = parameters.cellOccupiedThreshold;
  usObstacleGrid.cellMaxOccupancy = parameters.cellMaxOccupancy;
  gameInfoGameStateLastFrame = theGameInfo.state;

  Pose2D drawingOrigin = theRobotPose + Pose2D(usObstacleGrid.gridOffset.rotation) + Pose2D(usObstacleGrid.gridOffset.translation);
  DECLARE_DEBUG_DRAWING("origin:USObstacleGrid", "drawingOnField");
  ORIGIN("origin:USObstacleGrid", drawingOrigin.translation.x,
         drawingOrigin.translation.y, drawingOrigin.rotation);
  
#ifdef TARGET_SIM
  drawingOrigin = theGroundTruthRobotPose + Pose2D(usObstacleGrid.gridOffset.rotation) + Pose2D(usObstacleGrid.gridOffset.translation);
  DECLARE_DEBUG_DRAWING("origin:GroundTruthUSObstacleGrid", "drawingOnField");
  ORIGIN("origin:GroundTruthUSObstacleGrid", drawingOrigin.translation.x,
         drawingOrigin.translation.y, drawingOrigin.rotation);
#endif
}

void USObstacleGridProvider::checkUS()
{
  if(theFilteredSensorData.usTimeStamp == lastUsTimeStamp)
    return;
  lastUsTimeStamp = theFilteredSensorData.usTimeStamp;

  lastUsActuatorMode = theFilteredSensorData.usActuatorMode;

  switch(theFilteredSensorData.usActuatorMode)
  {

  case SensorData::leftToBoth:

    break;
  case SensorData::rightToBoth:
    break;

//  case SensorData::leftToLeft:
//  case SensorData::rightToRight:
//    addUsMeasurement(actuatorChanged, theFilteredSensorData.data[SensorData::usL], SensorData::leftToLeft, theFilteredSensorData.usTimeStamp);
//    addUsMeasurement(actuatorChanged, theFilteredSensorData.data[SensorData::usR], SensorData::rightToRight, theFilteredSensorData.usTimeStamp);
//    break;
//  case SensorData::leftToRight:
//  case SensorData::rightToLeft:
//    addUsMeasurement(actuatorChanged, theFilteredSensorData.data[SensorData::usL], SensorData::rightToLeft, theFilteredSensorData.usTimeStamp);
//    addUsMeasurement(actuatorChanged, theFilteredSensorData.data[SensorData::usR], SensorData::leftToRight, theFilteredSensorData.usTimeStamp);
//    break;
  default:
    ASSERT(false);
    break;
  }
}

void USObstacleGridProvider::addUsMeasurement(bool actuatorChanged, float m, ActuatorMode mappedActuatorMode, unsigned int timeStamp)
{
  ASSERT(mappedActuatorMode >= 0);
  ASSERT(mappedActuatorMode < 4);

  bool isNew = timeStamp == theFilteredSensorData.usTimeStamp;
  if(timeStamp > bufferedMeasurements[mappedActuatorMode].timeStamp)
  {
    if(actuatorChanged || m < bufferedMeasurements[mappedActuatorMode].value)
      bufferedMeasurements[mappedActuatorMode].value = m;
    bufferedMeasurements[mappedActuatorMode].timeStamp = timeStamp;
  }

  if(m < parameters.minValidUSDist)
    return;
  switch(mappedActuatorMode)
  {
  case leftToLeft:
    for(std::vector<UsMeasurement>::iterator it = postponedLeftToRightMeasurements.begin(); it != postponedLeftToRightMeasurements.end();)
    {
      if(timeStamp - it->timeStamp > 1000) // postponed measurement is to old
        it = postponedLeftToRightMeasurements.erase(it);
      else if(abs(m - it->value) < 200)
      {
        addUsMeasurement(false, it->value, leftToRight, it->timeStamp);
        it = postponedLeftToRightMeasurements.erase(it);
      }
      else
        ++it;
    }
    break;
  case rightToRight:
    for(std::vector<UsMeasurement>::iterator it = postponedRightToLeftMeasurements.begin(); it != postponedRightToLeftMeasurements.end();)
    {
      if(timeStamp - it->timeStamp > 1000) // postponed measurement is to old
        it = postponedRightToLeftMeasurements.erase(it);
      else if(abs(m - it->value) < 200)
      {
        addUsMeasurement(false, it->value, rightToLeft, it->timeStamp);
        it = postponedRightToLeftMeasurements.erase(it);
      }
      else
        ++it;
    }
    break;
  case leftToRight:
    measuredCenterLeft = false; // reset, set again if an actual obstacle will be detected
    if(isNew && (timeStamp - bufferedMeasurements[leftToLeft].timeStamp > 1000 || abs(bufferedMeasurements[leftToLeft].value - m) > 200))
    {
      postponedLeftToRightMeasurements.push_back(UsMeasurement(m, timeStamp));
      return; // Don't fill cells, as leftToRight measurements are only valid if left Sensor measured something
    }
    break;
  case rightToLeft:
    measuredCenterRight = false; // reset, set again if an actual obstacle will be detected
    if(isNew && (timeStamp - bufferedMeasurements[rightToRight].timeStamp > 1000 || abs(bufferedMeasurements[rightToRight].value - m) > 200))
    {
      postponedRightToLeftMeasurements.push_back(UsMeasurement(m, timeStamp));
      return; // Don't fill cells, as rightToLeft measurements are only valid if right Sensor measured something
    }
    break;
  default:
    ASSERT(false);
    break;
  }

  // Compute and draw relevant area:
  if(m > parameters.maxValidUSDist)
    m = static_cast<float>(parameters.maxValidUSDist);
  Vector2<> measurement(m, 0.0f);
  Vector2<> base, leftOfCone(measurement), rightOfCone(measurement);
  switch(mappedActuatorMode)
  {
  case leftToLeft: // left transmitter, left sensor => left
    base = parameters.usLeftPose.translation;
    leftOfCone.rotate(parameters.usOuterOpeningAngle);
    rightOfCone.rotate(parameters.usInnerOpeningAngle);
    leftOfCone = parameters.usLeftPose * leftOfCone;
    rightOfCone = parameters.usLeftPose * rightOfCone;
    break;

  case rightToRight: // right transmitter, right sensor => right
    base = parameters.usRightPose.translation;
    leftOfCone.rotate(-parameters.usInnerOpeningAngle);
    rightOfCone.rotate(-parameters.usOuterOpeningAngle);
    leftOfCone = parameters.usRightPose * leftOfCone;
    rightOfCone = parameters.usRightPose * rightOfCone;
    break;

  case leftToRight: // left transmitter, right sensor => center left
    base = parameters.usCenterPose.translation;
    leftOfCone.rotate(parameters.usCenterOpeningAngle);
    rightOfCone.rotate(parameters.usCenterOpeningAngle * 0.25f);
    leftOfCone = parameters.usCenterPose * leftOfCone;
    rightOfCone = parameters.usCenterPose * rightOfCone;
    break;

  case rightToLeft: // right transmitter, left sensor => center right
    base = parameters.usCenterPose.translation;
    leftOfCone.rotate(-parameters.usCenterOpeningAngle * 0.25f);
    rightOfCone.rotate(-parameters.usCenterOpeningAngle);
    leftOfCone = parameters.usCenterPose * leftOfCone;
    rightOfCone = parameters.usCenterPose * rightOfCone;
    break;

  default:
    ASSERT(false);
    break;
  }

  // Draw current (positive) measurement:
  if(m < parameters.maxValidUSDist)
  {
    LINE("module:USObstacleGridProvider:us", base.x, base.y, leftOfCone.x, leftOfCone.y,
         30, Drawings::ps_solid, ColorRGBA(255, 0, 0));
    LINE("module:USObstacleGridProvider:us", base.x, base.y, rightOfCone.x, rightOfCone.y,
         30, Drawings::ps_solid, ColorRGBA(255, 0, 0));
    LINE("module:USObstacleGridProvider:us", rightOfCone.x, rightOfCone.y, leftOfCone.x, leftOfCone.y,
         30, Drawings::ps_solid, ColorRGBA(255, 0, 0));
  }

  // Compute additional cones:
  // *Free is used for clearing cells between the robot and the obstacle
  // *Far is used to add a second obstacle line to the grid (should help in case of high walk speeds and imprecision)
  Vector2<> leftOfConeFree(leftOfCone - base);
  Vector2<> rightOfConeFree(rightOfCone - base);
  Vector2<> leftOfConeFar(leftOfCone - base);
  Vector2<> rightOfConeFar(rightOfCone - base);
  float cellDiameter = sqrt(2.f) * (float) USObstacleGrid::CELL_SIZE;
  leftOfConeFree.normalize(leftOfConeFree.abs() - cellDiameter);
  rightOfConeFree.normalize(rightOfConeFree.abs() - cellDiameter);
  leftOfConeFar.normalize(leftOfConeFar.abs() + cellDiameter);
  rightOfConeFar.normalize(rightOfConeFar.abs() + cellDiameter);
  leftOfConeFree += base;
  rightOfConeFree += base;
  leftOfConeFar += base;
  rightOfConeFar += base;
  // Transfer cones to grid coordinate system:
  const Vector2<int> leftOfConeCells = 
    theUSObstacleGrid.worldToGrid(Vector2<int>(static_cast<int>(leftOfCone.x), static_cast<int>(leftOfCone.y)));
  const Vector2<int> leftOfConeCellsFree = 
    theUSObstacleGrid.worldToGrid(Vector2<int>(static_cast<int>(leftOfConeFree.x), static_cast<int>(leftOfConeFree.y)));
  const Vector2<int> leftOfConeCellsFar = 
    theUSObstacleGrid.worldToGrid(Vector2<int>(static_cast<int>(leftOfConeFar.x), static_cast<int>(leftOfConeFar.y)));
  const Vector2<int> rightOfConeCells = 
    theUSObstacleGrid.worldToGrid(Vector2<int>(static_cast<int>(rightOfCone.x), static_cast<int>(rightOfCone.y)));
  const Vector2<int> rightOfConeCellsFree = 
    theUSObstacleGrid.worldToGrid(Vector2<int>(static_cast<int>(rightOfConeFree.x), static_cast<int>(rightOfConeFree.y)));
  const Vector2<int> rightOfConeCellsFar = 
    theUSObstacleGrid.worldToGrid(Vector2<int>(static_cast<int>(rightOfConeFar.x), static_cast<int>(rightOfConeFar.y)));

  // Free empty space until obstacle:
  polyPoints.clear();
  // Origin (sensor position):
  Vector2<int> p1(static_cast<int>(base.x), static_cast<int>(base.y));
  p1 = theUSObstacleGrid.worldToGrid(p1);
  polyPoints.push_back(p1);
  // Left corner of "cone":
  polyPoints.push_back(Point(leftOfConeCellsFree, Point::NO_OBSTACLE));
  // Right corner of "cone":
  const Vector2<> ll(rightOfConeFree);
  float f1 = ll.abs(),
        f2 = f1 ? leftOfConeFree.abs() / f1 : 0;
  Vector2<> gridPoint(ll);
  gridPoint *= f2;
  const Vector2<int> gridPointInt(static_cast<int>(gridPoint.x), static_cast<int>(gridPoint.y));
  polyPoints.push_back(Point(theUSObstacleGrid.worldToGrid(gridPointInt), polyPoints.back().flags));
  polyPoints.push_back(Point(rightOfConeCellsFree, Point::NO_OBSTACLE));
  // Sensor position again:
  polyPoints.push_back(p1);
  // Clip and fill:
  for(int j = 0; j < (int) polyPoints.size(); ++j)
    clipPointP2(p1, polyPoints[j]);
  fillScanBoundary();

  // Enter obstacle to grid:
  // If the sensor measures a high value, cells are cleared but
  // no obstacles are entered
  // Obstacles to the left and to the right of the robot might be skipped, if there 
  // have recently been measurements in the center.
  if(m != parameters.maxValidUSDist)
  {
    if(mappedActuatorMode == leftToRight)
      measuredCenterLeft = true;
    else if(mappedActuatorMode == rightToLeft)
      measuredCenterRight = true;
    else if(mappedActuatorMode == leftToLeft && measuredCenterLeft)
      return;
    else if(mappedActuatorMode == rightToRight && measuredCenterRight)
      return;
    line(leftOfConeCells, rightOfConeCells);
    line(leftOfConeCellsFar, rightOfConeCellsFar);
  }
}

void USObstacleGridProvider::ageCellState()
{
  for(int i = 0; i < GRID_SIZE; ++i)
  {
    USObstacleGrid::Cell& c = cells[i];
    if(c.state)
    {
      if(theFrameInfo.getTimeSince(c.lastUpdate) > parameters.cellFreeInterval)
      {
        c.state--;
        c.lastUpdate = theFrameInfo.time;
      }
      c.cluster = 0;
    }
  }
}

void USObstacleGridProvider::moveGrid()
{
  accumulatedOdometry += theOdometryData - lastOdometry;
  lastOdometry = theOdometryData;
  // Move grid backwards in x direction (robot moves forward):
  if(accumulatedOdometry.translation.x >= USObstacleGrid::CELL_SIZE / 2)
  {
    accumulatedOdometry.translation.x -= USObstacleGrid::CELL_SIZE;
    for(int y = 0; y < GRID_LENGTH; ++y)
    {
      USObstacleGrid::Cell* cStartNew = &cells[y * GRID_LENGTH];
      USObstacleGrid::Cell* cStartOld = cStartNew + 1;
      memmove(cStartNew, cStartOld, sizeof(USObstacleGrid::Cell) * (GRID_LENGTH - 1));
      cStartNew[GRID_LENGTH - 1] = USObstacleGrid::Cell();
    }
  }
  // Move grid forward in x direction (robot moves backwards):
  else if(accumulatedOdometry.translation.x <= -USObstacleGrid::CELL_SIZE / 2)
  {
    accumulatedOdometry.translation.x += USObstacleGrid::CELL_SIZE;
    for(int y = 0; y < GRID_LENGTH; ++y)
    {
      USObstacleGrid::Cell* cStartOld = &cells[y * GRID_LENGTH];
      USObstacleGrid::Cell* cStartNew = cStartOld + 1;
      memmove(cStartNew, cStartOld, sizeof(USObstacleGrid::Cell) * (GRID_LENGTH - 1));
      cStartOld[0] = USObstacleGrid::Cell();
    }
  }
  // Move grid backwards in y direction (robot moves to the left):
  if(accumulatedOdometry.translation.y >= USObstacleGrid::CELL_SIZE / 2)
  {
    accumulatedOdometry.translation.y -= USObstacleGrid::CELL_SIZE;
    USObstacleGrid::Cell* cStartOld = &cells[GRID_LENGTH];
    USObstacleGrid::Cell* cStartNew = &cells[0];
    memmove(cStartNew, cStartOld, sizeof(USObstacleGrid::Cell)*GRID_LENGTH * (GRID_LENGTH - 1));
    USObstacleGrid::Cell* c = &cells[(GRID_LENGTH - 1) * GRID_LENGTH];
    for(int x = 0; x < GRID_LENGTH; ++x)
      c[x] = USObstacleGrid::Cell();
  }
  // Move grid forward in y direction (robot moves to the right):
  else if(accumulatedOdometry.translation.y <= -USObstacleGrid::CELL_SIZE / 2)
  {
    accumulatedOdometry.translation.y += USObstacleGrid::CELL_SIZE;
    USObstacleGrid::Cell* cStartNew = &cells[GRID_LENGTH];
    USObstacleGrid::Cell* cStartOld = &cells[0];
    memmove(cStartNew, cStartOld, sizeof(USObstacleGrid::Cell) * GRID_LENGTH * (GRID_LENGTH - 1));
    USObstacleGrid::Cell* c = &cells[0];
    for(int x = 0; x < GRID_LENGTH; ++x)
      c[x] = USObstacleGrid::Cell();
  }
}

void USObstacleGridProvider::clipPointP2(const Vector2<int>& p1, Point& p2) const
{
  if(p2.x < 0)
  {
    p2.y = p1.y + (p2.y - p1.y) * -p1.x / (p2.x - p1.x);
    p2.x = 0;
  }
  else if(p2.x >= GRID_LENGTH)
  {
    p2.y = p1.y + (p2.y - p1.y) * (GRID_LENGTH - 1 - p1.x) / (p2.x - p1.x);
    p2.x = GRID_LENGTH - 1;
  }

  if(p2.y < 0)
  {
    p2.x = p1.x + (p2.x - p1.x) * -p1.y / (p2.y - p1.y);
    p2.y = 0;
  }
  else if(p2.y >= GRID_LENGTH)
  {
    p2.x = p1.x + (p2.x - p1.x) * (GRID_LENGTH - 1 - p1.y) / (p2.y - p1.y);
    p2.y = GRID_LENGTH - 1;
  }
}

void USObstacleGridProvider::fillScanBoundary()
{
  // generate boundary of polygon
  std::list<Line> lines;
  int j;
  for(j = 1; j < (int) polyPoints.size() - 1; ++j)
  {
    if(polyPoints[j - 1].y < polyPoints[j].y && polyPoints[j + 1].y < polyPoints[j].y)
      polyPoints[j].flags |= Point::PEAK;
    lines.push_back(Line(polyPoints[j - 1], polyPoints[j]));
  }
  lines.push_back(Line(polyPoints[j - 1], polyPoints[j]));

  // sort the lines by their y coordinates
  lines.sort();

  // run through lines in increasing y order
  for(int y = (int) lines.front().a.y; !lines.empty(); ++y)
  {
    inter.clear();
    for(std::list<Line>::iterator it = lines.begin(); it != lines.end() && (*it).a.y <= y;)
      if((*it).b.y > y || ((*it).peak && (*it).b.y == y))
      {
        // line is not finished yet
        inter.push_back(int((*it).a.x));
        (*it).a.x += (*it).ils;
        ++it;
      }
      else if((*it).b.y == y && (*it).a.y == y && y != 0)
      {
        // horizontal line, is not drawn when y == 0, because overlapping lines clutter the map
        inter.push_back(int((*it).a.x));
        inter.push_back(int((*it).b.x));
        ++it;
      }
      else if(it != lines.begin())
      {
        // line is finished -> remove it
        std::list<Line>::iterator it_help = it;
        --it;
        lines.erase(it_help);
        ++it;
      }
      else
      {
        // special case: remove begin of list
        lines.erase(it);
        it = lines.begin();
      }

    // fill the line on an even/odd basis
    bool paint = false;
    int goalX = -1;
    std::sort(inter.begin(), inter.end());
    std::vector<int>::iterator iIt = inter.begin();
    if(iIt != inter.end())
      for(;;)
      {
        int startX = *iIt;
        if(++iIt == inter.end())
          break;
        paint ^= true;
        if(paint)
        {
          if(startX == goalX)
            ++startX;
          goalX = *iIt;
          USObstacleGrid::Cell* cell = &cells[y * GRID_LENGTH + startX];
          for(int x = startX; x <= goalX; ++x)
          {
            if((*cell).state > 0)
              (*cell).state--;
            (*cell).lastUpdate = theFrameInfo.time;
            ++cell;
          }
        }
      }
  }
}

void USObstacleGridProvider::line(const Vector2<int>& start, const Vector2<int>& end)
{
  Vector2<int> diff = end - start,
               inc(diff.x > 0 ? 1 : -1, (diff.y > 0 ? 1 : -1) * (&cells[GRID_LENGTH] - &cells[0])),
               absDiff(abs(diff.x), abs(diff.y));
  USObstacleGrid::Cell* p = &cells[start.y * GRID_LENGTH + start.x];

  if(absDiff.y < absDiff.x)
  {
    int error = -absDiff.x;
    for(int i = 0; i <= absDiff.x; ++i)
    {
      if((p->state < parameters.cellMaxOccupancy) && (p->lastUpdate != theFrameInfo.time))
        p->state++;
      p->lastUpdate = theFrameInfo.time;
      p += inc.x;
      error += 2 * absDiff.y;
      if(error > 0)
      {
        p += inc.y;
        error -= 2 * absDiff.x;
      }
    }
  }
  else
  {
    int error = -absDiff.y;
    for(int i = 0; i <= absDiff.y; ++i)
    {
      if((p->state < parameters.cellMaxOccupancy) && (p->lastUpdate != theFrameInfo.time))
        p->state++;
      p->lastUpdate = theFrameInfo.time;
      p += inc.y;
      error += 2 * absDiff.x;
      if(error > 0)
      {
        p += inc.x;
        error -= 2 * absDiff.y;
      }
    }
  }
}

MAKE_MODULE(USObstacleGridProvider, Modeling)
