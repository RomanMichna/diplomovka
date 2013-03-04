/**
* @file BoolArray.h
*
* This file declares a class for space efficiently storing an array of boolean values.
* It is 8 times faster in initialization compared to an array of bools
* and approx 10 times faster in element access compared to std::vector<bool>
* and approx 10% slower in element access compared to an array of bools
*
* @author Alexander Härtl
*/

class BoolArray
{
  /**
  * this type is used to store the boolean values
  */
  typedef int BoolArrayBaseType;

private:
  /**
  * The class is used as a wrapper to access and modify the boolean values
  */
  class BoolReference
  {
  private:
    BoolArrayBaseType* datum; /* pointer to the base type the referenced boolean value is contained in */
    int index; /* index of the boolean value within the base type */
  public:
    BoolReference(BoolArrayBaseType* datum, unsigned index);
    operator bool() const; /* for usage in if-statements etc. */
    bool operator=(bool value); /* assignment operator */
  };

  BoolArrayBaseType* data; /* the actual array storing the boolean values */
  const unsigned int numOfElements; /* number of elements of base type (sizeof(data) / sizeof(BoolArrayBaseType)) */


public:
  BoolArray(unsigned int size); /* the only constructor needs the size to be allocated, like an array */
  ~BoolArray();
  BoolReference operator[](int index); /* array-like access to the boolean values */
};
