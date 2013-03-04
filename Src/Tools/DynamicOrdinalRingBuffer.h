/*
 * DynamicMedianRingBuffer.h
 *
 *  Created on: 19.06.2012
 *      Author: marcel
 */

#pragma once
#include "DynamicRingBuffer.h"

template <class V>
class DynamicOrdinalRingBuffer : public DynamicRingBuffer<V>
{
private:
  V partition(V* input, int p, int r)
  {
    int pivot = input[r];

    while (p < r)
    {
      while (input[p] < pivot)
      {
        p++;
      }

      while (input[r] > pivot)
      {
        r--;
      }

      if (input[p] == input[r])
      {
        p++;
      }
      else if (p < r) {
          int tmp = input[p];
          input[p] = input[r];
          input[r] = tmp;
      }
    }

    return r;
  }

  V quick_select(V* input, int p, int r, int k)
  {
    if (p == r)
    {
      return input[p];
    }

    int j = partition(input, p, r);
    int length = j - p + 1;

    if (length == k)
    {
      return input[j];
    }
    else if (k < length)
    {
      return quick_select(input, p, j - 1, k);
    }
    else
    {
      return quick_select(input, j + 1, r, k - length);
    }
  }

public:
  DynamicOrdinalRingBuffer(int n) : DynamicRingBuffer<V>(n) {}

  virtual ~DynamicOrdinalRingBuffer() {}

  V select(int k)
  {
    int size = DynamicRingBuffer<V>::size();
    V arr[size];
    for (int i = 0; i < size; ++i)
    {
      arr[i] = (DynamicRingBuffer<V>::get(i));
    }

    return quick_select(arr, 0, DynamicRingBuffer<V>::size(), k);
  }

  inline V getMin()
  {
    return select(1);
  }

  inline V getMax()
  {
    return select(DynamicRingBuffer<V>::size());
  }

  inline V getMedian()
  {
    return select(DynamicRingBuffer<V>::size() / 2);
  }
};
