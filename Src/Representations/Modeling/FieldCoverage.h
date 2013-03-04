
#pragma once

#include "Tools/Streams/Streamable.h"
#include "Tools/Math/Vector2.h"

class FieldCoverage : public Streamable
{
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(worstNoTurnTarget);
    STREAM(worstTarget);
    STREAM(cells);
    STREAM(mean);
    STREAM(stddev);
    STREAM(throwIn);
    STREAM_REGISTER_FINISH;
  }

public:
  class GridInterval : public Streamable
  {
  private:
    virtual void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN;
      STREAM(timestamp);
      STREAM(interval);
      STREAM(cells);
      STREAM_REGISTER_FINISH;
    }

  public:
    static const int xSteps = 12;
    static const int ySteps = 8;
    static const int intervals = 3; /* Number of intervals into the coverage grid is divided up. */
    static const int intervalSize = xSteps* ySteps / intervals;
    static const int maxCoverage = 255;
    static const unsigned tick = 300; /* Milliseconds one coverage tick is worth. */
    unsigned timestamp; /* The timestamp when this grid has been updated. The coverage values are relative to this timestamp. */
    /* Interval of the grid currently represented.
     * Interval i represents cells from [i * (xSteps*ySteps/intervals), (i+1) * (xSteps*ySteps/intervals) - 1].
     * interval < intervals must always hold. */
    unsigned char interval;
    unsigned char cells[intervalSize];

    GridInterval();
    GridInterval(const GridInterval& other);
    GridInterval& operator=(const GridInterval& other);

    static inline bool assertIntervalSize() { return xSteps * ySteps % intervals == 0; }
    inline void nextInterval() { interval = (interval + 1) % intervals; }

    inline static Vector2<int> index2CellCoordinates(int idx) { return Vector2<int>(idx / ySteps, idx % ySteps); }
    inline static int cellCoordinates2Index(unsigned x, unsigned y) { return x * ySteps + y; }
  };

  class Target : public Streamable
  {
    virtual void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN;
      STREAM(target);
      STREAM(coverage);
      STREAM(isValid);
      STREAM_REGISTER_FINISH;
    }

  public:
    Target() : target(0.0f, 0.0f), coverage(0), isValid(false) {}

    Target(float x, float y, unsigned short coverage = 0, bool valid = true)
      : target(x, y), coverage(coverage), isValid(valid) {}

    Target(const Vector2<>& target, unsigned short coverage, bool valid = true)
      : target(target), coverage(coverage), isValid(valid) {}

    Target(const Target& o) { *this = o; }

    Target& operator=(const Target& o) { target = o.target; coverage = o.coverage; isValid = o.isValid; return *this; }

    Vector2<> target;
    unsigned short coverage;
    bool isValid; /* True if this is a valid target. The coordinates and coverage should be ignored if valid is false. */
  };

  FieldCoverage();
  unsigned char coverage(int cellIdx, unsigned time) const;

  Target worstNoTurnHalfRangeTarget; /* Worst covered cell in relative coordinates which is closer than the half the fieldcoverage range. */
  Target worstNoTurnRangeTarget; /* Worst covered cell in relative coordinates which is 'close' and for which you do not have to turn around. */
  Target worstNoTurnTarget; /* Worst covered cell on the field in relative coordinates for which you do not have to turn around. */
  Target worstTarget; /* Worst covered cell on the field in field coordinates. */
  unsigned cells[GridInterval::xSteps* GridInterval::ySteps]; /* cells[i] contains the last-seen timestamp of the i-th cell. */
  float mean; /* The mean coverage value. */
  float stddev; /* The variance of the coverage values.. */
  bool throwIn; /* True if the field coverage has been modified because the ball has been thrown in. */
};
