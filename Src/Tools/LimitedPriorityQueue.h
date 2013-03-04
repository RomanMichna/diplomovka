
#pragma once

#include "Platform/BHAssert.h"
#include "Tools/Streams/Streamable.h"

#include <vector>

/**
 * A buffer that contains n elements ordered by their priority. If the size
 * exceeds n, the elements with the lowest priority will be deleted.
 */
template <class V, class Compare = std::less<V> >
class LimitedPriorityQueue : public Streamable
{
  std::vector<V> buffer;
  unsigned maxSize;
  Compare compare;

public:
  LimitedPriorityQueue()
    : buffer(), maxSize(1), compare()
  {
    buffer.reserve(maxSize);
  }

  LimitedPriorityQueue(unsigned maxSize)
    : buffer(), maxSize(maxSize), compare()
  {
    buffer.reserve(maxSize);
  }

  LimitedPriorityQueue(const LimitedPriorityQueue<V, Compare>& queue)
    : buffer(queue.buffer), maxSize(queue.maxSize), compare(queue.compare)
  {
    buffer.reserve(maxSize);
  }

  /**
   * Adds an element to the buffer. The position depends on the priority.
   */
  void add(const V& v)
  {
    bool last = true;
    for(typename std::vector<V>::iterator it = buffer.begin(); it != buffer.end(); it++)
      if(compare(*it, v))
      {
        buffer.insert(it, v);
        last = false;
        break;
      }
    if(last)
      buffer.push_back(v);

    if(buffer.size() > maxSize)
      buffer.resize(maxSize);
    ASSERT(buffer.size() <= maxSize);
  }

  /**
   * @return The current number of entries.
   */
  unsigned getNumberOfEntries() const
  {
    return (unsigned) buffer.size();
  }

  /**
   * @return The maximal number of entries.
   */
  unsigned getMaxEntries() const
  {
    return maxSize;
  }

  /**
   * Sets the maximal number of entries and resizes the buffer if necessary.
   */
  void setMaxEntries(unsigned maxEntries)
  {
    ASSERT(maxEntries > 0);
    if(maxEntries < maxSize)
      buffer.resize(maxEntries);
    else
      buffer.reserve(maxEntries);
    maxSize = maxEntries;
  }

  /**
   * @return The element with the highest priority.
   */
  V& top()
  {
    return buffer[0];
  }

  /**
   * @return The element with the highest priority.
   */
  const V& top() const
  {
    return buffer[0];
  }

  unsigned size() const
  {
    return (unsigned) buffer.size();
  }

  V operator[](unsigned index)
  {
    ASSERT(index < maxSize);
    return buffer[index];
  }

  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(buffer);
    STREAM(maxSize);
    STREAM_REGISTER_FINISH;
  }
};
