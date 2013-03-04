#include "Representations/Perception/RegionPercept.h"
#include "PointExplorer.h"
#include "Tools/Debugging/Modify.h"
#include "Tools/Debugging/Asserts.h"

void PointExplorer::initFrame(const Image* image, const ColorTable64* colorTable, int exploreStepSize, int gridStepSize, int skipOffset, int* minSegLength)
{
  theImage = image;
  theColorTable64 = colorTable;
  parameters.exploreStepSize = exploreStepSize;
  parameters.gridStepSize = gridStepSize;
  parameters.skipOffset = skipOffset;
  parameters.minSegSize = minSegLength;
  DECLARE_DEBUG_DRAWING("module:PointExplorer:runs", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:PointExplorer:blob", "drawingOnImage");
}

ColorClasses::Color PointExplorer::getColor(int x, int y)
{
  ASSERT_COORDINATES_WITHIN_IMAGE(x, y, *theImage);
  unsigned col(theImage->image[(size_t) y][(size_t) x].color);
  return (ColorClasses::Color)theColorTable64->colorClasses[col >> 18 & 0x3f][col >> 10 & 0x3f][col >> 26 & 0x3f];
}

int PointExplorer::explorePoint(int x, int y, ColorClasses::Color col, int xMin, int yEnd, int yMin, int& run_end, int& explored_min_y, int& explored_max_y, bool force_detailed)
{
  int size = 0;
  if(col != ColorClasses::green || force_detailed)
  {
    run_end = runDown(x, y, col, yEnd, Drawings::ps_solid);
    explored_min_y = y;
    explored_max_y = run_end - 1;

    size = (run_end - y) * parameters.gridStepSize;

    if(!(run_end - y >= parameters.minSegSize[col]))
      return -1;

    for(x -= parameters.exploreStepSize; x > xMin; x -= parameters.exploreStepSize)
    {
      if(getColor(x, explored_min_y) == col)
      {
        const int expl_run_end = runUp(x, explored_min_y, col, yMin, Drawings::ps_dot);
        if(expl_run_end < explored_min_y)
          explored_min_y = expl_run_end + 1;
      }
      if(getColor(x, explored_max_y) == col)
      {
        const int expl_run_end = runDown(x, explored_max_y, col, yEnd, Drawings::ps_dot);
        if(expl_run_end > explored_max_y)
          explored_max_y = expl_run_end - 1;
      }
    }
  }
  else
  {
    run_end = runDown(x, y, col, yEnd, Drawings::ps_solid);
    size = (run_end - y) * parameters.gridStepSize;
    explored_min_y = y;
    explored_max_y = run_end;
  }
  return size;
}

int PointExplorer::runDown(int x, int yStart, ColorClasses::Color col, int yEnd, Drawings::PenStyle draw)
{
  int y = yStart;
  int tmp;

  for(y += parameters.skipOffset; y < yEnd; y += parameters.skipOffset)
  {
    if(getColor(x, y) != col)
    {
      tmp = y - parameters.skipOffset;
      for(--y; y > tmp; y--)
        if(getColor(x, y) == col)
          break;

      if(y == tmp)
      {
        y++;
        break;
      }
    }
  }
  if(y > yEnd)
    y = yEnd;

  LINE("module:PointExplorer:runs", x, yStart, x, y, 0, draw, getOnFieldDrawColor(col));

  return y;
}

int PointExplorer::findDown(int x, int yStart, ColorClasses::Color col, int yEnd, Drawings::PenStyle draw)
{
  int y = yStart;
  int tmp;

  for(y += parameters.skipOffset; y < yEnd; y += parameters.skipOffset)
  {
    if(getColor(x, y) == col)
    {
      tmp = y - parameters.skipOffset;
      for(--y; y > tmp; y--)
        if(getColor(x, y) != col)
          break;

      if(y != tmp)
        ++y;
      break;
    }
  }
  if(y > yEnd)
    y = yEnd;

  LINE("module:PointExplorer:runs", x, yStart, x, y, 0, draw, getOnFieldDrawColor(col));

  return y;
}

int PointExplorer::runUp(int x, int yStart, ColorClasses::Color col, int yEnd, Drawings::PenStyle draw)
{
  int y = yStart;
  int tmp;

  for(y -= parameters.skipOffset; y > yEnd; y -= parameters.skipOffset)
  {
    if(getColor(x, y) != col)
    {
      tmp = y + parameters.skipOffset;
      for(++y; y < tmp; y++)
        if(getColor(x, y) == col)
          break;

      if(y == tmp)
      {
        y--;
        break;
      }
    }
  }
  if(y < yEnd)
    y = yEnd;

  LINE("module:PointExplorer:runs", x, yStart, x, y, 0, draw, getOnFieldDrawColor(col));

  return y;
}

float PointExplorer::colorRatio(const ColorClasses::Color& col, const Boundary<int>& area, int stepSize)
{
  const float numberOfPixels(float((area.x.max + 1 - area.x.min) * (area.y.max + 1 - area.y.min)));
  if(numberOfPixels == 0.0)
  {
    return 0.0;
  }
  const int stepSize2 = stepSize * stepSize;
  int sameColor(0);
  const int xStart(std::max(area.x.min, 0)),
        xEnd(std::min(area.x.max, theImage->resolutionWidth - 1));
  const int yStart(std::max(area.y.min, 0)),
        yEnd(std::min(area.y.max, theImage->resolutionHeight - 1));
  const int lastColStepSize = (xEnd - xStart) % stepSize + 1;
  const int lastColStepSize2 = lastColStepSize * stepSize;
  const int lastRowStepSize = (yEnd - yStart) % stepSize + 1;
  const int lastRowStepSize2 = lastRowStepSize * stepSize;
  for(int x(xStart); x <= xEnd; x += stepSize)
  {
    bool lastCol = x + stepSize > xEnd;
    for(int y(yStart); y <= yEnd; y += stepSize)
    {
      bool lastRow = y + stepSize > yEnd;
      if(getColor(x, y) == col)
      {
        sameColor += lastCol ? (lastRow ? lastColStepSize * lastRowStepSize : lastColStepSize2) : (lastRow ? lastRowStepSize2 : stepSize2);
        DOT("module:PointExplorer:blob", x, y, col, col);
      }
    }
  }
  return static_cast<float>(sameColor) / numberOfPixels;
}

