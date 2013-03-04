/**
* @file Regionizer.cpp
* @author jeff
* @author benny
*/

#include "Regionizer.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Debugging/Modify.h"
#include "Tools/Settings.h"

Regionizer::Regionizer()
{
  InConfigMap stream(Global::getSettings().expandLocationFilename("regionizer.cfg"));
  if(stream.exists())
    stream >> parameters;
  else
  {
    InConfigMap stream("regionizer.cfg");
    ASSERT(stream.exists());
    stream >> parameters;
  }
}

void Regionizer::update(RegionPercept& rPercept)
{
  regionPercept = &rPercept;
  MODIFY("parameters:Regionizer", parameters);
  DECLARE_DEBUG_DRAWING("module:Regionizer:borders", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:Regionizer:borders2", "drawingOnImage");

  DECLARE_DEBUG_DRAWING("module:Regionizer:scans", "drawingOnImage");

  pointExplorer.initFrame(&theImage, &theColorTable64, parameters.exploreStepSize, parameters.gridStepSize, parameters.skipOffset, parameters.minSegSize);

  if(parameters.searchBallGridStepSize > parameters.gridStepSize || parameters.searchBallGridStepSize < 1 || parameters.gridStepSize % parameters.searchBallGridStepSize != 0)
    parameters.searchBallGridStepSize = parameters.gridStepSize;

  regionPercept->fieldBorders.clear();

  regionPercept->segmentsCounter = 0;
  regionPercept->regionsCounter = 0;

  regionPercept->gridStepSize = parameters.gridStepSize;

  STOP_TIME_ON_REQUEST("scanVert", scanVertically(););
  STOP_TIME_ON_REQUEST("buildRegions", buildRegions(););
}

bool Regionizer::uniteRegions(RegionPercept::Segment* seg1, RegionPercept::Segment* seg2)
{
  ASSERT(seg1->region);
  //we want straight white regions (lines) so don't unite white regions which would not be straight
  if(seg1->color == ColorClasses::white)
  {
    ASSERT(seg1->region->childs.size() >= 1);
    if(seg1->region->childs.at(seg1->region->childs.size() - 1)->x == seg2->x)
      return false;

    if(seg1->link)
    {
      Vector2<int> linkDiff = Vector2<int>(seg1->x - seg1->link->x, (seg1->y + seg1->length / 2) - (seg1->link->y + seg1->link->length / 2)),
                   thisDiff = Vector2<int>(seg2->x - seg1->x, (seg2->y + seg2->length / 2) - (seg1->y + seg1->length / 2));
      if(abs(linkDiff.angle() - thisDiff.angle()) > parameters.maxAngleDiff)
        return false;

    }
  }

  if(seg1->length * parameters.regionLengthFactor[seg1->color] < seg2->length ||
     seg2->length * parameters.regionLengthFactor[seg1->color] < seg1->length)
    return false;

  //seg1 always has a region
  if(!seg2->region)
  {
    RegionPercept::Segment* sWithRegion = seg1,
                          * sWithoutRegion = seg2;

    if((int)sWithRegion->region->childs.size() < parameters.regionMaxSize ||
       seg1->color == ColorClasses::yellow || //yellow, blue and orange regions can grow unlimited
       seg1->color == ColorClasses::blue ||
       seg1->color == ColorClasses::orange)
    {
      sWithRegion->region->childs.push_back(sWithoutRegion);
      sWithoutRegion->region = sWithRegion->region;
      sWithRegion->region->size += sWithoutRegion->explored_size;

      if(sWithoutRegion->y < sWithRegion->region->min_y)
        sWithRegion->region->min_y = sWithoutRegion->y;
      if(sWithoutRegion->y + sWithoutRegion->length > sWithRegion->region->max_y)
        sWithRegion->region->max_y = sWithoutRegion->y + sWithoutRegion->length;
      seg2->link = seg1;
      return true;
    }
    ASSERT(seg1->region == seg2->region || !seg2->region);
  }
  //both segments already have a region
  else
  {
    //don't unite two white regions (since we want straight white regions -> lines)
    if(seg1->color != ColorClasses::white)
    {
      if(seg1->region != seg2->region && ((int)(seg1->region->childs.size() + seg2->region->childs.size()) < parameters.regionMaxSize || seg1->color == ColorClasses::yellow || seg1->color == ColorClasses::blue || seg1->color == ColorClasses::orange))
      {
        RegionPercept::Region* oldRegion = seg2->region;
        seg1->region->mergeWithRegion(seg2->region);
        oldRegion->childs.clear();
        oldRegion->root = seg1->region;
        seg2->link = seg1;
        return true;
      }
    }
  }
  return false;
}

inline RegionPercept::Segment* Regionizer::addSegment(int x, int y, int length, ColorClasses::Color color, int yEnd)
{
  if(!(regionPercept->segmentsCounter < MAX_SEGMENTS_COUNT - 1))
    return NULL;

  RegionPercept::Segment* seg = regionPercept->segments + regionPercept->segmentsCounter++;
  seg->color = color;
  seg->x = x;
  seg->y = y;
  seg->explored_min_y = y;
  seg->explored_max_y = y + length;
  seg->explored_size = 0;
  seg->length = length;
  seg->region = NULL;
  seg->link = NULL;

  return seg;
}

RegionPercept::Segment* Regionizer::connectToRegions(RegionPercept::Segment* newSegment, RegionPercept::Segment* lastColumPointer, int xDiff)
{
  ASSERT(!newSegment->region);
  if(!lastColumPointer)
  {
    createNewRegionForSegment(newSegment);
    return NULL;
  }

  ASSERT(lastColumPointer->x == newSegment->x - xDiff); //parameters.gridStepSize);
  //if lastColumPointer needs to move on
  // _
  //|_|
  //   _    this case
  //  |_|
  //
  while(lastColumPointer->y + lastColumPointer->length < newSegment->explored_min_y && lastColumPointer->x == newSegment->x - xDiff)
    lastColumPointer++;
  //lastColumPointer is now either the first segment in the last line
  //which ends after the start of newSegment or is already in the next line -> return NULL

  if(lastColumPointer->x != newSegment->x - xDiff)
  {
    createNewRegionForSegment(newSegment);
    return NULL;
  }

  std::vector<RegionPercept::Region*> neighborRegions;

  if(lastColumPointer->y + lastColumPointer->length >= newSegment->explored_min_y)
    if(lastColumPointer->y <= newSegment->explored_max_y)
    {
      //TOUCHING
      if(lastColumPointer->color == newSegment->color)
      {
        if(!uniteRegions(lastColumPointer, newSegment))
          neighborRegions.push_back(lastColumPointer->region);
      }
      else
      {
        if(lastColumPointer->region)
          neighborRegions.push_back(lastColumPointer->region);
      }
    }

  //  _
  // | |_
  // |_| |
  //   |_| this case
  RegionPercept::Segment* tmpLastColumPointer;
  tmpLastColumPointer = lastColumPointer;
  while(tmpLastColumPointer->y + tmpLastColumPointer->length <= newSegment->explored_max_y)
  {
    tmpLastColumPointer++;
    //if the lastColumPointer moved to the next colum, move one back and break
    if(tmpLastColumPointer->x != newSegment->x - xDiff)
    {
      if(!newSegment->region)
        if(!createNewRegionForSegment(newSegment))
          return NULL;
      break;
    }

    ASSERT(tmpLastColumPointer->y + tmpLastColumPointer->length >= newSegment->explored_min_y);
    if(tmpLastColumPointer->y <= newSegment->explored_max_y)
    {
      //TOUCHING
      if(tmpLastColumPointer->color == newSegment->color)
      {
        if(!uniteRegions(tmpLastColumPointer, newSegment))
          neighborRegions.push_back(tmpLastColumPointer->region);
      }
      else
      {
        if(tmpLastColumPointer->region)
          neighborRegions.push_back(tmpLastColumPointer->region);
      }
    }
    else
      break;
  }

  if(!newSegment->region)
    if(!createNewRegionForSegment(newSegment))
      return NULL;

  for(std::vector<RegionPercept::Region*>::iterator nb_reg = neighborRegions.begin(); nb_reg != neighborRegions.end(); nb_reg++)
  {
    ASSERT(newSegment->region);
    (*nb_reg)->neighborRegions.push_back(newSegment->region);
    newSegment->region->neighborRegions.push_back(*nb_reg);
  }
  return lastColumPointer;
}

bool Regionizer::createNewRegionForSegment(RegionPercept::Segment* seg)
{
  if(regionPercept->regionsCounter < MAX_REGIONS_COUNT)
  {
    seg->region = regionPercept->regions + regionPercept->regionsCounter++;
    seg->region->color = seg->color;
    seg->region->childs.clear();
    seg->region->neighborRegions.clear();
    seg->region->min_y = seg->y;
    seg->region->max_y = seg->y + seg->length;
    seg->region->root = NULL;
    seg->region->size = seg->explored_size;
    seg->region->childs.push_back(seg);
    return true;
  }
  return false;
}

void Regionizer::calcBorders(int xStart, int xEnd)
{
  regionPercept->fieldBorders.clear();

  Vector2<int> p1 = theImageCoordinateSystem.fromHorizonBased(Vector2<>());
  Vector2<int> b;
  int min_y_border = 0;

  //if horizon is below cameraimage
  if(p1.y >= theImage.resolutionHeight)
  {
    regionPercept->fieldBorders.push_back(Vector2<int>(0, theImage.resolutionHeight - 1));
    regionPercept->fieldBorders.push_back(Vector2<int>(theImage.resolutionWidth - 1, theImage.resolutionHeight - 1));
  }
  else
  {
    if(p1.y <= 0)
      min_y_border = 0;
    else
      min_y_border = p1.y;

    int x = xStart;
    int run_end;
    for(; x < xEnd; x += parameters.gridStepSize)
    {
      b.x = x;
      b.y = min_y_border;

      run_end = b.y;
      while(b.y < theImage.resolutionHeight)
      {
        b.y = pointExplorer.findDown(b.x, b.y, ColorClasses::green, theImage.resolutionHeight, Drawings::ps_dot);
        if(b.y >= theImage.resolutionHeight)
          break;
        const int tyEnd = theImage.resolutionHeight < b.y + parameters.borderMinGreen + 1 ? theImage.resolutionHeight : b.y + parameters.borderMinGreen + 1;
        run_end = pointExplorer.runDown(b.x, b.y, ColorClasses::green, tyEnd, Drawings::ps_dot);

        if(run_end - b.y > parameters.borderMinGreen)
          break;

        b.y = run_end;
      }
      if(b.y >= theImage.resolutionHeight)
        b.y =  theImage.resolutionHeight - 1;
      regionPercept->fieldBorders.push_back(b);
    }
  }
  regionPercept->fieldBorders = getConvexFieldBorders(regionPercept->fieldBorders);

  std::vector<Vector2<int> > newBorders;
  int x = xStart;
  CI p = regionPercept->fieldBorders.begin();
  CI last_p = p++;
  for(; p != regionPercept->fieldBorders.end(); p++)
  {
    ASSERT(p->x >= 0);
    ASSERT(p->y >= 0);
    ASSERT(p->x < theImage.resolutionWidth);
    ASSERT(p->y < theImage.resolutionHeight);
    while(x <= p->x)
    {
      const Vector2<int> v_m = (*p) - (*last_p);
      b.x = x;
      b.y = (x - last_p->x) * v_m.y / v_m.x + last_p->y;
      if(b.y < 0)
        b.y = 0;
      if(b.y >= theImage.resolutionHeight)
        b.y = theImage.resolutionHeight - 1;
      ASSERT(b.x >= 0);
      ASSERT(b.x < theImage.resolutionWidth);
      newBorders.push_back(b);
      x += parameters.gridStepSize;
    }
    last_p = p;
  }
  regionPercept->fieldBorders = newBorders;

  COMPLEX_DRAWING("module:Regionizer:borders",
  {
    CI last_p;
    for(CI p = regionPercept->fieldBorders.begin(); p != regionPercept->fieldBorders.end(); p++)
    {
      if(p != regionPercept->fieldBorders.begin())
      {LINE("module:Regionizer:borders", p->x, p->y, last_p->x, last_p->y, 0, Drawings::ps_solid, ColorRGBA(255, 0, 0));}
      last_p = p;
    }
    LINE("module:Regionizer:borders", 0, min_y_border, theImage.resolutionWidth, min_y_border, 0, Drawings::ps_dot, ColorRGBA(255, 0, 0));
  });
}

void Regionizer::buildRegions()
{
  int lastx = -1;

  RegionPercept::Segment* firstInColum = NULL, *lastColumPointer = NULL, *newSegment, *lastSegment = NULL;

  for(int i = 0; i < regionPercept->segmentsCounter; i++)
  {
    newSegment = regionPercept->segments + i;

    if(newSegment->color == ColorClasses::green)
      continue;
    //a new colum started
    if(lastx == -1 || lastx != newSegment->x)
    {
      if(lastx == newSegment->x - parameters.gridStepSize)
        lastColumPointer = firstInColum;
      else
        lastColumPointer = NULL;
      firstInColum = newSegment;
      lastSegment = NULL;
    }

    lastColumPointer = connectToRegions(newSegment, lastColumPointer, parameters.gridStepSize);
    ASSERT(newSegment->region != NULL || regionPercept->regionsCounter >= MAX_REGIONS_COUNT);
    if(newSegment->region == NULL) //MAX_REGIONS_COUNT
      break;

    if(lastSegment != NULL)
    {
      if(newSegment-> y - (lastSegment->y + lastSegment->length) < parameters.skipOffset)
      {
        lastSegment->region->neighborRegions.push_back(newSegment->region);
        newSegment->region->neighborRegions.push_back(lastSegment->region);
      }
      else
        lastSegment = NULL;
    }

    lastSegment = newSegment;
    lastx = newSegment->x;
  }
}

void Regionizer::scanVertically()
{
  int xStart = parameters.gridStepSize - 1 + ((theImage.resolutionWidth) %  parameters.gridStepSize) / 2,
      xEnd = theImage.resolutionWidth,
      yEnd = theImage.resolutionHeight,
      yStart = 0, yFullStart = 0;
  int yHorizon = theImageCoordinateSystem.fromHorizonBased(Vector2<>()).y;


  RegionPercept::Segment* newSegment;
  ColorClasses::Color curColor;
  int run_end_y, explored_min_y, explored_max_y, explored_size;

  STOP_TIME_ON_REQUEST("calcBorders", calcBorders(xStart, xEnd););

  CI border = regionPercept->fieldBorders.begin();

  bool perceptFull = false;
  bool useExtraScanlinesForBall = parameters.searchBallGridStepSize < parameters.gridStepSize && theFrameInfo.getTimeSince(theBallModel.timeWhenDisappeared) > parameters.searchBallStartTime;
  bool ballScanline = false;
  int x = xStart,
      y = 0;
  for(; x < xEnd && !perceptFull ; x += parameters.gridStepSize)
  {
    // calculate yEnd based on stopPolygon
    yEnd = theImage.resolutionHeight;

    theBodyContour.clipBottom(x, yEnd);

    if(border != regionPercept->fieldBorders.end())
    {
      ASSERT(x == border->x);
      yStart = std::max(yHorizon, 0);
      yFullStart = border->y;
      ASSERT(yFullStart <= yFullStart);
      border++;
    }
    else
    {
      yStart = 0;
      yFullStart = 0;
    }

    for(int xb = x; useExtraScanlinesForBall ? xb < x + parameters.gridStepSize && xb < xEnd && !perceptFull : xb == x; xb += parameters.searchBallGridStepSize)
    {
      if(useExtraScanlinesForBall)
        ballScanline = xb != x;

      y = yStart;

      while(y < yEnd)
      {
        if(ballScanline)
        {
          curColor = ColorClasses::orange;
          const int byEnd = ((yEnd - yStart) >> 1) + yStart;
          y = pointExplorer.findDown(xb, y, curColor, byEnd, Drawings::ps_solid);
          if(y == byEnd)
            break;
        }
        else
          curColor = pointExplorer.getColor(xb, y);

        explored_size = pointExplorer.explorePoint(xb, y, curColor, std::max(0, xb - parameters.gridStepSize), yEnd, yStart, run_end_y, explored_min_y, explored_max_y);

        if(run_end_y - y >= parameters.minSegSize[curColor])
        {
          if(run_end_y > yFullStart || curColor == ColorClasses::blue || curColor == ColorClasses::red)
          {
            if(y < yFullStart && curColor != ColorClasses::blue && curColor != ColorClasses::red)
            {
              y = yFullStart;
              explored_min_y = y;
              explored_size = explored_max_y - explored_min_y;
            }
            newSegment = addSegment(xb, y, run_end_y - y, curColor, yEnd);

            if(!newSegment) //MAX_SEGMENTS_COUNT
            {
              ASSERT(regionPercept->segmentsCounter == MAX_SEGMENTS_COUNT - 1);
              perceptFull = true;
              break;
            }

            newSegment->explored_min_y = explored_min_y;
            newSegment->explored_max_y = explored_max_y;
            newSegment->explored_size = explored_size;
          }
        }
        y = run_end_y;
      }
    }
  }
}

#define LEFT_OF(x0, x1, x2) ((x1.x-x0.x)*(-x2.y+x0.y)-(x2.x-x0.x)*(-x1.y+x0.y) > 0)

std::vector<Vector2<int> > Regionizer::getConvexFieldBorders(std::vector<Vector2<int> >& fieldBorders)
{
  //Andrew's Monotone Chain Algorithm to compute the upper hull
  std::vector<Vector2<int> > hull;
  const CI pmin = fieldBorders.begin(), pmax = fieldBorders.end() - 1;
  hull.push_back(*pmin);
  for(CI pi = pmin + 1; pi != pmax + 1; pi++)
  {
    if(!LEFT_OF((*pmin), (*pmax), (*pi)) && pi != pmax)
      continue;

    while((int)hull.size() > 1)
    {
      const CI p1 = hull.end() - 1, p2 = hull.end() - 2;
      if(LEFT_OF((*p1), (*p2), (*pi)))
        break;
      hull.pop_back();
    }
    hull.push_back(*pi);
  }
  return hull;
}

MAKE_MODULE(Regionizer, Perception)
