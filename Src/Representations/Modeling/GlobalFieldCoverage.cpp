/*
 * GlobalFieldCoverage.cpp
 *
 * Implementation of GlobalFieldCoverage methods.
 *
 * Author: Felix Wenk
 */

#include <cstring>

#include "GlobalFieldCoverage.h"
#include "Platform/BHAssert.h"
#include "Tools/Math/Common.h"

GlobalFieldCoverage::Grid::Grid()
{
  memset(cells, 0, sizeof(cells));
}

GlobalFieldCoverage::Grid::Grid(const GlobalFieldCoverage::Grid& other)
{
  *this = other;
}

GlobalFieldCoverage::Grid& GlobalFieldCoverage::Grid::operator=(const GlobalFieldCoverage::Grid& other)
{
  memcpy(cells, other.cells, sizeof(cells));
  return *this;
}

unsigned char GlobalFieldCoverage::Grid::coverage(int index, unsigned time) const
{
  int sub = (time - cells[index]) / FieldCoverage::GridInterval::tick;
  if(sub < 0) sub = 0;
  return sub >= FieldCoverage::GridInterval::maxCoverage ? 0 : FieldCoverage::GridInterval::maxCoverage - sub;
}

void GlobalFieldCoverage::Grid::setCoverage(int index, unsigned time, unsigned char coverage)
{
  ASSERT(coverage <= FieldCoverage::GridInterval::maxCoverage);
  ASSERT(index < FieldCoverage::GridInterval::xSteps * FieldCoverage::GridInterval::ySteps);
  cells[index] = std::max(0, static_cast<int>(time - FieldCoverage::GridInterval::tick * (FieldCoverage::GridInterval::maxCoverage - coverage)));
}
