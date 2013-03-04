/**
 * @file DynamicRingBuffer.h
 * @author Marcel Steinbeck
 */

#pragma once
#include <vector>

/**
 * @class DynamicRingBuffer
 */
template <class V>
class DynamicRingBuffer
{
protected:
  int current;
  int numberOfElements;

  int capacity;
  std::vector<V> buffer;

public:
  /** Constructor */
  DynamicRingBuffer(int n)
  : current(n - 1), numberOfElements(0), capacity(n)
  {
    buffer.resize(n);
    ASSERT(n > 0);    
  }

  virtual ~DynamicRingBuffer() {}

  /**
   * adds an entry to the buffer.
   */
  virtual inline void add(const V& v)
  {
    current++;
    current %= capacity;
    if(++numberOfElements >= capacity) numberOfElements = capacity;
    buffer[current] = v;
  }

  /**
   * returns an element
   * \parameter i index of entry counting from last added (last=0, last-1=1, ...)
   */
  inline const V& get(int i) const
  {
    return buffer[(capacity + current - i) % capacity];
  }

  /**
   * removes the first added element to the buffer
   */
  virtual inline void removeFirst()
  {
    --numberOfElements;
  }

  /**
   * Returns the number of elements that are currently in the ring buffer
   */
  inline int size() const
  {
    return numberOfElements;
  }

  /**
   * Returns the maximum element count.
   */
  inline int getCapacity() const
  {
    return capacity;
  }

  /**
   * loot at #get
   */
  inline const V& operator[](int i) const
  {
    return buffer[(capacity + current - i) % capacity];
  }
};
