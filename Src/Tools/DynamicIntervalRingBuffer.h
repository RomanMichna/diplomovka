/*
 * DynimicRingBufferWithSum.h
 * @author Marcel Steinbeck
 */

#pragma once
#include "DynamicOrdinalRingBuffer.h"

template <class V>
class DynamicIntervalRingBuffer : public DynamicOrdinalRingBuffer<V>
{
private:
  V sum;

public:
  DynamicIntervalRingBuffer(int n) : DynamicOrdinalRingBuffer<V>(n), sum(V()) {}

  virtual ~DynamicIntervalRingBuffer() {}

  /**
   * adds an entry to the buffer.
   */
  virtual inline void add(const V& v)
  {
    DynamicRingBuffer<V>::add(v);
    sum += v;
  }

  /**
   * removes the first added element to the buffer
   */
  virtual inline void removeFirst()
  {
    DynamicRingBuffer<V>::removeFirst();
    sum -= DynamicRingBuffer<V>::get(DynamicRingBuffer<V>::size()-1);
  }

  /**
   * returns the sum of all elements
   */
  inline V getSum() const
  {
    return sum;
  }

  /**
   * returns the average value of all element (integer division)
   */
  inline V getAverage() const
  {
    // Return 0 if buffer is empty
    if(0 == DynamicRingBuffer<V>::numberOfEntries) return V();
    return (sum / DynamicRingBuffer<V>::numberOfEntries);
  }

  /**
   * returns the average value of all elements (float division)
   */
  inline V getAverageFloat()
  {
    // Return 0 if buffer is empty
    if (0==DynamicRingBuffer<V>::numberOfEntries) return V();
    return (sum / static_cast<float>(DynamicRingBuffer<V>::numberOfEntries));
  }
};
