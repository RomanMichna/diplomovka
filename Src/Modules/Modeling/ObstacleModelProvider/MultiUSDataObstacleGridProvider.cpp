/**
* @author Nico Lehmann
* @author Arne Böckmann
* @author Marcel Steinbeck
*/

#include <algorithm>
#include "MultiUSDataObstacleGridProvider.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Settings.h"
#include <limits>

const int CELL_SIZE   = USObstacleGrid::CELL_SIZE;
const int GRID_LENGTH = USObstacleGrid::GRID_LENGTH;
const int GRID_SIZE   = USObstacleGrid::GRID_SIZE;;


MultiUSDataObstacleGridProvider::MultiUSDataObstacleGridProvider()
{
  lastTimePenalized=0;
  lastUsTimeStamp=0;
  gameInfoGameStateLastFrame=STATE_INITIAL;

  intervals.reserve(7);
  measurements.reserve(4);

  {
    InConfigMap stream(Global::getSettings().expandLocationFilename("usObstacleGrid.cfg"));
    if(stream.exists())
    {
      stream >> parameters;
    }
  }
  
  {
    InConfigMap stream(Global::getSettings().expandRobotFilename("usCalibration.cfg"));
    if(stream.exists())
    {
      stream >> usCalibration;
    }
  }

  usCalibration.llLeftOpeningAngle = fromDegrees(usCalibration.llLeftOpeningAngle);
  usCalibration.llRightOpeningAngle = fromDegrees(usCalibration.llRightOpeningAngle);
  usCalibration.lrLeftOpeningAngle = fromDegrees(usCalibration.lrLeftOpeningAngle);
  usCalibration.lrRightOpeningAngle = fromDegrees(usCalibration.lrRightOpeningAngle);
  usCalibration.rrLeftOpeningAngle = fromDegrees(usCalibration.rrLeftOpeningAngle);
  usCalibration.rrRightOpeningAngle = fromDegrees(usCalibration.rrRightOpeningAngle);
  usCalibration.rlLeftOpeningAngle = fromDegrees(usCalibration.rlLeftOpeningAngle);
  usCalibration.rlRightOpeningAngle = fromDegrees(usCalibration.rlRightOpeningAngle);

  angles.push_back(Angle(usCalibration.llLeftOpeningAngle, llLeft));
  angles.push_back(Angle(usCalibration.llRightOpeningAngle, llRight));
  angles.push_back(Angle(usCalibration.lrLeftOpeningAngle, lrLeft));
  angles.push_back(Angle(usCalibration.lrRightOpeningAngle, lrRight));
  angles.push_back(Angle(usCalibration.rrLeftOpeningAngle, rrLeft));
  angles.push_back(Angle(usCalibration.rrRightOpeningAngle, rrRight));
  angles.push_back(Angle(usCalibration.rlLeftOpeningAngle, rlLeft));
  angles.push_back(Angle(usCalibration.rlRightOpeningAngle, rlRight));

  sort(angles.begin(), angles.end());
  //Reverse the angles to get a desc sorting.
  reverse(angles.begin(), angles.end());
  ASSERT(angles.size() == 8);

  //Initialze intervals based on the angles.
  Angles::iterator left = angles.begin();
  Angles::iterator right = left+1;
  do
  {
    intervals.push_back(Interval(*left, *right));
    left++;
    right++;

  }while (right != angles.end());

  //initialize mapping between us measurement modes and their covered intervals
  initializeIntervalMapping(leftToLeft, llLeft, llRight);
  initializeIntervalMapping(leftToRight, lrLeft, lrRight);
  initializeIntervalMapping(rightToRight, rrLeft, rrRight);
  initializeIntervalMapping(rightToLeft, rlLeft, rlRight);

  lastOdometry = theOdometryData;
}

void MultiUSDataObstacleGridProvider::update(USObstacleGrid &usObstacleGrid)
{
  MODIFY("parameters:MultiUSDataGridProvider", parameters);
  MODIFY("calibration:MultiUSDataGridProvider", usCalibration);
  DECLARE_DEBUG_DRAWING("module:MultiUSDataGridProvider:us", "drawingOnField");

  grid = &usObstacleGrid;
  if((gameInfoGameStateLastFrame == STATE_SET) && (theGameInfo.state == STATE_PLAYING))
  {
    grid->clear();
  }

  if(theFilteredSensorData.usChanged)
  {
    ageCellState();

    if((theRobotInfo.penalty == PENALTY_NONE) &&
        (theMotionRequest.motion != MotionRequest::specialAction) &&
        (theFrameInfo.getTimeSince(lastTimePenalized) > 3000))
    {
      moveGrid();
      checkUS();
    }
    else if(theRobotInfo.penalty != PENALTY_NONE)
    {
      lastTimePenalized = theFrameInfo.time;
    }
  }
  usObstacleGrid.gridOffset = Pose2D(-accumulatedOdometry.rotation, -accumulatedOdometry.translation);
  usObstacleGrid.cellOccupiedThreshold = parameters.cellOccupiedThreshold;
  usObstacleGrid.cellMaxOccupancy = parameters.cellMaxOccupancy;
  gameInfoGameStateLastFrame = theGameInfo.state;

  Pose2D drawingOrigin = theRobotPose + usObstacleGrid.gridOffset;
  DECLARE_DEBUG_DRAWING("origin:USObstacleGrid", "drawingOnField");
  ORIGIN("origin:USObstacleGrid", drawingOrigin.translation.x,
         drawingOrigin.translation.y, drawingOrigin.rotation);
}

void MultiUSDataObstacleGridProvider::checkUS()
{

  loadMeasurements(measurements);//clear us measurements and load new ones.

  groupMeasurements(measurements, groups);//group measurements by distance

  //put the center of each group to the grid.
  for(MeasurementGroups::iterator group = groups.begin(); group != groups.end(); group++)
  {
    resetIntervals();

    //determine which sensor modes where used within this group and which where not.
    bool usedModes[numOfUSSensorModes] = {false};

    for(Measurements::iterator measurement = group->begin(); measurement != group->end(); measurement++)
    {
      usedModes[measurement->mode] = true;
    }
    //increase the hitCount of every interval hit by this group
    for (USSensorMode m = leftToLeft; m < numOfUSSensorModes; m = (USSensorMode) (m + 1))
    {
      if(usedModes[m])
      {
        increaseHitCount(sensorModeToInterval[m]);
      }
    }
    //reset the hitCount of every interval not hit by this group
    for (USSensorMode m = leftToLeft; m < numOfUSSensorModes; m = (USSensorMode) (m + 1))
    {
      if(!usedModes[m])
      {
        resetHitCount(sensorModeToInterval[m]);
      }
    }

    //After the increase and reset stage only the the intervals with possible
    //obstacles remain.
    //Now we need to identify hitCount maxima within the remaining intervals.

    Intervals::iterator i = intervals.begin();
    unsigned int max = i->hitCount;
    Angle start = i->left;
    Angle end = i->right;
    i++;

    bool increased = max > 0;
    for(; i != intervals.end(); i++)
    {
      if(i->hitCount > max)
      {//found new maximum, last one was not a maximum
        max = i->hitCount;
        start = i->left;
        end = i->right;
        increased = true; //true as long as we are on a maximum

        if(i+1 == intervals.end()) //special case for maximum at last element
        {//draw it
          enterLine(group->front().distance, start.angle, end.angle);
        }
      }
      else if(i->hitCount < max)
      {//end of current maximum, draw it
        enterLine(group->front().distance, start.angle, end.angle);

        max = i->hitCount;
        start = i->left;
        end = i->right;
        increased = false;
      }
      else
      {
        //previous hitCount == current hitCount, found plateau.
        end = i->right;
        if(i+1 == intervals.end() && increased) //special case for plateau at last element
        {//draw it
          enterLine(group->front().distance, start.angle, end.angle);
        }
      }
    }
  }
}

void MultiUSDataObstacleGridProvider::resetIntervals()
{
  for(Intervals::iterator it = intervals.begin(); it != intervals.end(); ++it)
  {
    it->hitCount = 0;
  }
}

void MultiUSDataObstacleGridProvider::moveGrid()
{
  accumulatedOdometry += theOdometryData - lastOdometry;
  lastOdometry = theOdometryData;
  // Move grid backwards in x direction (robot moves forward):
  if(accumulatedOdometry.translation.x >= USObstacleGrid::CELL_SIZE / 2)
  {
    accumulatedOdometry.translation.x -= USObstacleGrid::CELL_SIZE;
    for(int y = 0; y < GRID_LENGTH; ++y)
    {
      USObstacleGrid::Cell* cStartNew = &grid->cells[y * GRID_LENGTH];
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
      USObstacleGrid::Cell* cStartOld = &grid->cells[y * GRID_LENGTH];
      USObstacleGrid::Cell* cStartNew = cStartOld + 1;
      memmove(cStartNew, cStartOld, sizeof(USObstacleGrid::Cell) * (GRID_LENGTH - 1));
      cStartOld[0] = USObstacleGrid::Cell();
    }
  }
  // Move grid backwards in y direction (robot moves to the left):
  if(accumulatedOdometry.translation.y >= USObstacleGrid::CELL_SIZE / 2)
  {
    accumulatedOdometry.translation.y -= USObstacleGrid::CELL_SIZE;
    USObstacleGrid::Cell* cStartOld = &grid->cells[GRID_LENGTH];
    USObstacleGrid::Cell* cStartNew = &grid->cells[0];
    memmove(cStartNew, cStartOld, sizeof(USObstacleGrid::Cell)*GRID_LENGTH * (GRID_LENGTH - 1));
    USObstacleGrid::Cell* c = &grid->cells[(GRID_LENGTH - 1) * GRID_LENGTH];
    for(int x = 0; x < GRID_LENGTH; ++x)
      c[x] = USObstacleGrid::Cell();
  }
  // Move grid forward in y direction (robot moves to the right):
  else if(accumulatedOdometry.translation.y <= -USObstacleGrid::CELL_SIZE / 2)
  {
    accumulatedOdometry.translation.y += USObstacleGrid::CELL_SIZE;
    USObstacleGrid::Cell* cStartNew = &grid->cells[GRID_LENGTH];
    USObstacleGrid::Cell* cStartOld = &grid->cells[0];
    memmove(cStartNew, cStartOld, sizeof(USObstacleGrid::Cell) * GRID_LENGTH * (GRID_LENGTH - 1));
    USObstacleGrid::Cell* c = &grid->cells[0];
    for(int x = 0; x < GRID_LENGTH; ++x)
      c[x] = USObstacleGrid::Cell();
  }
}

void MultiUSDataObstacleGridProvider::enterLine(float distance, float leftAngle, float rightAngle)
{

  Vector2<float> leftCone(distance, 0.0f);
  Vector2<float> rightCone(distance, 0.0f);
  incLine(theFrameInfo.time, grid->worldToGrid(leftCone.rotate(leftAngle)),
    grid->worldToGrid(rightCone.rotate(rightAngle)), parameters.usCenterPose.translation);

  //draw a second line right behind the actual line to simulate the obstacles thickness.
  Vector2<float> leftCone2(distance + grid->CELL_SIZE + 1, 0.0f);
  Vector2<float> rightCone2(distance + grid->CELL_SIZE +1, 0.0f);
  incLine(theFrameInfo.time, grid->worldToGrid(leftCone2.rotate(leftAngle)),
    grid->worldToGrid(rightCone2.rotate(rightAngle)), parameters.usCenterPose.translation);
  
  freeEmptySpace(grid->worldToGrid(leftCone.rotate(leftAngle)),
      grid->worldToGrid(rightCone.rotate(rightAngle)), parameters.usCenterPose.translation);

}


void MultiUSDataObstacleGridProvider::ageCellState()
{
  for(int i = 0; i < GRID_SIZE; ++i)
  {
    USObstacleGrid::Cell& c = grid->cells[i];
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

void MultiUSDataObstacleGridProvider::incLine(unsigned int time, Vector2<float> p1, Vector2<float> p2, Vector2<float> base)
{
  Vector2<int> left(static_cast<int>(p1.x),static_cast<int>(p1.y));
  Vector2<int> right(static_cast<int>(p2.x),static_cast<int>(p2.y));

  Geometry::PixeledLine line(left,right);

  for(int i = 0; i < line.getNumberOfPixels(); ++i)
  {
    int pos = line.getPixelX(i) + line.getPixelY(i) * GRID_LENGTH;
    if(grid->cells[pos].state < grid->cellMaxOccupancy)
    {
      ++(grid->cells[pos].state);
    }
    grid->cells[pos].lastUpdate = time;
  }

}

void MultiUSDataObstacleGridProvider::clipPointP2(const Vector2<int>& p1, Point& p2) const
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

void MultiUSDataObstacleGridProvider::fillScanBoundary()
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
          USObstacleGrid::Cell* cell = &(grid->cells[y * GRID_LENGTH + startX]);
          for(int x = startX; x <= goalX; ++x)
          {
            if((*cell).state > 0) {
              agingValueOnFreeDrawing > (*cell).state ? (*cell).state = 0 : (*cell).state -= static_cast<unsigned char>(agingValueOnFreeDrawing);
            }
            (*cell).lastUpdate = theFrameInfo.time;
            ++cell;
          }
        }
      }
  }
}


void MultiUSDataObstacleGridProvider::loadMeasurements(Measurements& outMeasurements)
{
  outMeasurements.clear();
  for (std::vector<float>::const_iterator it = theFilteredSensorData.leftToRight.begin();
    it != theFilteredSensorData.leftToRight.end(); ++it)
  {
    outMeasurements.push_back(UsMeasurment(leftToRight, *it));
  }

  for (std::vector<float>::const_iterator it = theFilteredSensorData.leftToLeft.begin();
    it != theFilteredSensorData.leftToLeft.end(); ++it)
  {
    outMeasurements.push_back(UsMeasurment(leftToLeft, *it));
  }

  for (std::vector<float>::const_iterator it = theFilteredSensorData.rightToLeft.begin();
    it != theFilteredSensorData.rightToLeft.end(); ++it)
  {
    outMeasurements.push_back(UsMeasurment(rightToLeft,*it));
  }

  for (std::vector<float>::const_iterator it = theFilteredSensorData.rightToRight.begin();
    it != theFilteredSensorData.rightToRight.end(); ++it)
  {
    outMeasurements.push_back(UsMeasurment(rightToRight,*it));
  }
}


void MultiUSDataObstacleGridProvider::groupMeasurements(Measurements & measurements, MeasurementGroups & outGroups)
{
   float pivot;
   Measurements buffer;
   float tolerance = parameters.groupingDistance;

   outGroups.clear();

    //The measurements need to be sorted by distance to be able to use an efficient grouping algorithm.
   stable_sort(measurements.begin(), measurements.end());

   for (vector<UsMeasurment>::iterator it = measurements.begin(); it != measurements.end(); )
   {
     buffer.clear();
     pivot = it->distance;
     buffer.push_back(*it);
     it = measurements.erase(it);

     while (it!= measurements.end())
     {
       if(it->distance - pivot < tolerance)
       {
         buffer.push_back(*it);
         it = measurements.erase(it);
       }
       else
       {
         break;
       }
     }
     outGroups.push_back(buffer);
   }
}

void MultiUSDataObstacleGridProvider::increaseHitCount(vector<Interval*> intervals)
{
  for(unsigned int i = 0; i < intervals.size(); i++)
  {
    intervals[i]->hitCount++;
  }
}


void MultiUSDataObstacleGridProvider::resetHitCount(vector<Interval*> intervals)
{
  for(unsigned int i = 0; i < intervals.size(); i++)
  {
    intervals[i]->hitCount = 0;
  }
}



void MultiUSDataObstacleGridProvider::initializeIntervalMapping(USSensorMode key, AngleName begin, AngleName end)
{
  //initialize mapping between sensorMode and intervals
  //left to left
  bool isOpen = false;
  for(Intervals::iterator i = intervals.begin(); i != intervals.end(); i++)
  {
    if(i->left.name == begin)
    {
      sensorModeToInterval[key].push_back(&(*i));
      isOpen = true;
    }

    if(isOpen && i->right.name == end)
    {
      sensorModeToInterval[key].push_back(&(*i));
      break;
    }
    else if(isOpen && i->left.name != begin)
    {
      sensorModeToInterval[key].push_back(&(*i));
    }
  }
}



void MultiUSDataObstacleGridProvider::freeEmptySpace(Vector2<float> p1, Vector2<float> p2, Vector2<float> base)
{
  // Free empty space until obstacle:
  polyPoints.clear();

  // fixme
  float cellDiameter = sqrt(2.f) * (float) USObstacleGrid::CELL_SIZE;
  agingValueOnFreeDrawing = parameters.cellMaxOccupancy*parameters.agingFactorOnFreeDrawing;

  // Origin (sensor position):
  Vector2<int> point1(static_cast<int>(base.x), static_cast<int>(base.y));
  point1 = grid->worldToGrid(point1);
  polyPoints.push_back(point1);

  // Left corner of "cone":
  Vector2<float> p1Close = p1;
  p1Close = grid->gridToWorld(p1Close);
  float d = p1Close.abs();
  d -= cellDiameter;
  p1Close.normalize(d);
  p1Close = grid->worldToGrid(p1Close);
  polyPoints.push_back(Point(static_cast<int>(p1Close.x), static_cast<int>(p1Close.y), Point::NO_OBSTACLE));

  // right corner of "cone"
  Vector2<float> p2Close = p2;
  p2Close = grid->gridToWorld(p2Close);
  d = p2Close.abs();
  d -= cellDiameter;
  p2Close.normalize(d);
  p2Close = grid->worldToGrid(p2Close);
  polyPoints.push_back(Point(static_cast<int>(p2Close.x), static_cast<int>(p2Close.y), Point::NO_OBSTACLE));

  // Sensor position again:
  polyPoints.push_back(point1);

  // Clip and fill:
  for(int j = 0; j < (int) polyPoints.size(); ++j)
    clipPointP2(point1, polyPoints[j]);
  fillScanBoundary();
}



MAKE_MODULE(MultiUSDataObstacleGridProvider, Modeling)
