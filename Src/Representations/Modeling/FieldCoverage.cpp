/**
 * FieldCoverage.cpp
 *
 * Contains implementations of FieldCoverage and FieldCoverage grid methods.
 *
 * Author: Felix Wenk
 */

#include <cstring>

#include "Platform/BHAssert.h"
#include "FieldCoverage.h"

FieldCoverage::FieldCoverage()
  : mean(0.0f), stddev(0.0f)
{
  memset(cells, 0, sizeof(cells));
}

unsigned char FieldCoverage::coverage(int cellIdx, unsigned time) const
{
  int sub = (time - cells[cellIdx]) / GridInterval::tick;
  ASSERT(sub >= 0);
  return sub >= GridInterval::maxCoverage ? 0 : static_cast<unsigned char>(GridInterval::maxCoverage - sub);
}

FieldCoverage::GridInterval::GridInterval()
  : timestamp(0), interval(0)
{
  memset(cells, 0, xSteps * ySteps / intervals);
}

FieldCoverage::GridInterval::GridInterval(const FieldCoverage::GridInterval& other)
{
  *this = other;
}

FieldCoverage::GridInterval& FieldCoverage::GridInterval::operator=(const FieldCoverage::GridInterval& other)
{
  timestamp = other.timestamp;
  interval = other.interval;
  memcpy(cells, other.cells, xSteps * ySteps / intervals);
  return *this;
}
