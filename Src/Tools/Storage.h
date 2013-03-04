/**
* @file Storage.h
* @author jeff
*/

#pragma once

template<class D, int SIZE>
class Storage
{
protected:
  //null needs to be directly before data, this
  //ensures, that getIdx(null) returns -1 in RELEASE
  D null;
  D data[SIZE];
  int dataCounter;

public:
  Storage() : dataCounter(0) {};

  inline D& at(int idx)
  {
    ASSERT(idx < dataCounter);
    ASSERT(idx >= 0);
    return data[idx];
  }

  inline const D& atConst(int idx) const
  {
    ASSERT(idx < dataCounter);
    ASSERT(idx >= 0);
    return data[idx];
  }

  inline D& getNull()
  {
    return null;
  }

  inline bool isNull(D& d) const
  {
    return &d == &null;
  }

  virtual inline D& getNew()
  {
    ASSERT(dataCounter >= 0);
    if(dataCounter >= SIZE)
      return null;

    return data[dataCounter++];
  }

  virtual inline D& addCopy(const D& d)
  {
    ASSERT(dataCounter >= 0);
    if(dataCounter >= SIZE)
      return null;

    const int idx = dataCounter++;

    data[idx] = d;
    return data[idx];
  }

  inline void reset()
  {
    dataCounter = 0;
  }

  inline int getIdx(const D& d) const
  {
    ASSERT(&d != &null);
    ASSERT(&d - data >= 0);
    ASSERT(&d - data < dataCounter);
    return &d - data;
  }

  inline int size() const
  {
    return dataCounter;
  }
};

template<class D, int SIZE>
class StorageWDataReset : public Storage<D, SIZE>
{
public:
  virtual inline D& getNew()
  {
    D& d = Storage<D, SIZE>::getNew();
    d.reset();
    return d;
  }
};

template<class D, int SIZE>
class StoragePtr : public Storage<D, SIZE>
{
public:
  virtual inline D& getNew() {ASSERT(false); return Storage<D, SIZE>::null;}

};
