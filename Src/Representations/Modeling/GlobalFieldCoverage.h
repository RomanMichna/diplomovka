/*
 * GlobalFieldCoverage.h
 *
 * Definition of a class representing the global field coverage among all robots.
 * A part of a field is considered to be 'covered' if at least one robot looks at it.
 *
 *      Author: Felix Wenk
 */

#pragma once

#include "Tools/Streams/Streamable.h"
#include "Representations/Modeling/FieldCoverage.h"

class GlobalFieldCoverage: public Streamable
{
public:
  class Grid : public Streamable
  {
  private:
    virtual void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN;
      STREAM(cells);
      STREAM_REGISTER_FINISH;
    }
  public:
    Grid();
    Grid(const Grid& other);
    Grid& operator=(const Grid& other);

    unsigned char coverage(int index, unsigned time) const;
    void setCoverage(int index, unsigned time, unsigned char coverage);

    /* cells[i] is the timestamp when the cell has been looked at the last time by one robot. */
    unsigned int cells[FieldCoverage::GridInterval::xSteps * FieldCoverage::GridInterval::ySteps];
  };
private:
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(grid);
    STREAM(worstCoveredCellIndex);
    STREAM(threshold);
    STREAM(patrolTarget);
    STREAM(patrolTargetValid);
    STREAM_REGISTER_FINISH;
  }

public:
  Grid grid;
  int worstCoveredCellIndex;
  int threshold; /**< The coverage value above which a cell is considered to be covered. */
  Vector2<> patrolTarget; /**< Target the robot should try to approach while patrolling. */
  bool patrolTargetValid; /**< True if the patrol target is valid, otherwise false. */

  GlobalFieldCoverage() : worstCoveredCellIndex(0), threshold(0), patrolTargetValid(false) {}
  virtual ~GlobalFieldCoverage() {}
};
