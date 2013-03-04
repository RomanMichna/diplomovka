/**
* @file BoolArray.cpp
*
* This file implements the class BoolArray
*
* @author Alexander Härtl
*/

#include "BoolArray.h"
#include <algorithm>

BoolArray::BoolReference::BoolReference(BoolArrayBaseType* datum, unsigned int index) : datum(datum), index(index) {}

BoolArray::BoolReference::operator bool() const
{
  return (*datum & (1 << index)) != 0;
}

bool BoolArray::BoolReference::operator=(bool value)
{
  if(value)
  {
    *datum |= (1 << index);
  }
  else
  {
    *datum &= ~(1 << index);
  }
  return value;
}

BoolArray::BoolArray(unsigned int size) : numOfElements((size + sizeof(BoolArrayBaseType) * 8  - 1) / (sizeof(BoolArrayBaseType) * 8))
{
  data = new BoolArrayBaseType[numOfElements];
  std::fill(data, data + numOfElements, 0);
}

BoolArray::~BoolArray()
{
  delete[] data;
}

BoolArray::BoolReference BoolArray::operator[](int index)
{
  /* the index of the base type is given by the size of it, for the index within the base type the lower n bits are masked */
  return BoolReference(data + (index / (sizeof(BoolArrayBaseType) * 8)), index & (sizeof(BoolArrayBaseType) * 8  - 1));
}
