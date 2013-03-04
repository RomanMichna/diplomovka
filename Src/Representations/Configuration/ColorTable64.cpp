/**
* @file ColorTable64.cpp
* Implementation of class ColorTable64.
*
* @author <A href="mailto:juengel@informatik.hu-berlin.de">Matthias Jüngel</A>
*/

#include <cstring>
#include <cstdio>
#include <string>
#include <sstream>

#include "ColorTable64.h"
#include "Platform/BHAssert.h"
#include "Tools/ImageProcessing/ColorModelConversions.h"
#include "Tools/Sha1.h"

ColorTable64::ColorTable64()
{
  clear();
}

ColorTable64::ColorTable64(const ColorTable64& ct)
{
  memcpy(&colorClasses, ct.colorClasses, sizeof(colorClasses));
}

ColorTable64& ColorTable64::operator=(const ColorTable64& other)
{
  memcpy(&colorClasses, other.colorClasses, sizeof(colorClasses));
  return *this;
}

void ColorTable64::generateDiffImage(const ColorTable64 colorTable, const Image& source, const Image& view, Image& image) const
{
  ASSERT(source.resolutionWidth == view.resolutionWidth);
  ASSERT(source.resolutionHeight == view.resolutionHeight);
  ASSERT(source.resolutionWidth == image.resolutionWidth);
  ASSERT(source.resolutionHeight == image.resolutionHeight);

  for(int x = source.resolutionWidth - 1; x >= 0 ; --x)
    for(int y = source.resolutionHeight - 1; y >= 0; --y)
    {
      const Image::Pixel& cur = source.image[y][x];
      const unsigned char cy = cur.y >> 2,
                          ccr = cur.cr >> 2,
                          ccb = cur.cb >> 2;
      if(colorClasses[cy][ccb][ccr] != colorTable.colorClasses[cy][ccb][ccr])
        image.image[y][x].color = view.image[y][x].color ^ 0xffffff00;
      else
        image.image[y][x] = view.image[y][x];
    }
}

void ColorTable64::addColorClass(ColorClasses::Color colorClass,
                                 unsigned char y,
                                 unsigned char u,
                                 unsigned char v)
{
  y /= 4;
  u /= 4;
  v /= 4;

  unsigned char& c = colorClasses[y][u][v];
  if(c & 0x80 && colorClass != ColorClasses::none)
    c |= 1 << (colorClass - 1);
  else if(c == ColorClasses::none || colorClass == ColorClasses::none)
    c = colorClass;
  else if(c != colorClass)
    c = 1 << (c - 1) | 1 << (colorClass - 1) | 0x80;
}

void ColorTable64::addColorClass(ColorClasses::Color colorClass,
                                 unsigned char y,
                                 unsigned char u,
                                 unsigned char v,
                                 unsigned char range)
{

  y /= 4;
  u /= 4;
  v /= 4;

  if(y < range / 2) y = range / 2;
  if(u < range / 2) u = range / 2;
  if(v < range / 2) v = range / 2;

  for(unsigned char currentY = y - range / 2; currentY < y - range / 2 + range && currentY < 64; currentY++)
    for(unsigned char currentU = u - range / 2; currentU < u - range / 2 + range && currentU < 64; currentU++)
      for(unsigned char currentV = v - range / 2; currentV < v - range / 2 + range && currentV < 64; currentV++)
      {
        ASSERT(currentY < 64);
        ASSERT(currentU < 64);
        ASSERT(currentV < 64);

        unsigned char& c = colorClasses[currentY][currentU][currentV];
        if(c & 0x80 && colorClass != ColorClasses::none)
          c |= 1 << (colorClass - 1);
        else if(c == ColorClasses::none || colorClass == ColorClasses::none)
          c = colorClass;
        else if(c != colorClass)
          c = 1 << (c - 1) | 1 << (colorClass - 1) | 0x80;
      }
}

void ColorTable64::replaceColorClass(ColorClasses::Color from,
                                     ColorClasses::Color to,
                                     unsigned char y,
                                     unsigned char u,
                                     unsigned char v)
{
  y /= 4;
  u /= 4;
  v /= 4;

  unsigned char& c = colorClasses[y][u][v];
  if(c == from)
    c = to;
  else if(from != ColorClasses::none && (c & 0x80) && (c & 1 << (from - 1)))
  {
    c &= ~(1 << (from - 1));
    if(to & 0x80)
      c |= to;
    else if(to != ColorClasses::none)
      c |= 1 << (to - 1);
    for(int i = 0; i < 7; ++i)
      if(1 << i == (c & 0x7f))
      {
        c = i + 1; // single color
        break;
      }
  }

}


void ColorTable64::replaceColorClass(ColorClasses::Color from,
                                     ColorClasses::Color to,
                                     unsigned char y,
                                     unsigned char u,
                                     unsigned char v,
                                     unsigned char range)
{
  y /= 4;
  u /= 4;
  v /= 4;

  if(y < range / 2) y = range / 2;
  if(u < range / 2) u = range / 2;
  if(v < range / 2) v = range / 2;

  for(unsigned char currentY = y - range / 2; currentY < y - range / 2 + range && currentY < 64; currentY++)
    for(unsigned char currentU = u - range / 2; currentU < u - range / 2 + range && currentU < 64; currentU++)
      for(unsigned char currentV = v - range / 2; currentV < v - range / 2 + range && currentV < 64; currentV++)
      {
        ASSERT(currentY < 64);
        ASSERT(currentU < 64);
        ASSERT(currentV < 64);
        unsigned char& c = colorClasses[currentY][currentU][currentV];
        if(c == from)
          c = to;
        else if(from != ColorClasses::none && (c & 0x80) && (c & 1 << (from - 1)))
        {
          c &= ~(1 << (from - 1));
          if(to & 0x80)
            c |= to;
          else if(to != ColorClasses::none)
            c |= 1 << (to - 1);
          for(int i = 0; i < 7; ++i)
            if(1 << i == (c & 0x7f))
            {
              c = i + 1; // single color
              break;
            }
        }
      }
}

void ColorTable64::clearChannel(ColorClasses::Color colorClass)
{
  if(colorClass != ColorClasses::none)
    for(unsigned char y = 0; y < 64; y++)
      for(unsigned char u = 0; u < 64; u++)
        for(unsigned char v = 0; v < 64; v++)
        {
          unsigned char& c = colorClasses[y][u][v];
          if(c == colorClass)
            c = ColorClasses::none;
          else if((c & 0x80) && (c & 1 << (colorClass - 1)))
          {
            c &= ~(1 << (colorClass - 1));
            for(int i = 0; i < 7; ++i)
              if(1 << i == (c & 0x7f))
              {
                c = i + 1; // single color
                break;
              }
          }
        }
}

void ColorTable64::clear()
{
  memset(colorClasses, ColorClasses::none, sizeof(colorClasses));
}

Out& operator<<(Out& stream, const ColorTable64& colorTable64)
{
  // the current color class in the color table
  unsigned char currentColorClass = colorTable64.colorClasses[0][0][0];

  // the next color class to be compared with the last one
  unsigned char nextColorClass;

  // a pointer to the begin of the color table
  const unsigned char* colorTable = &colorTable64.colorClasses[0][0][0];

  // the number of bytes in one sequence that have the same color class
  int currentLength = 1;

  for(unsigned int i = 1; i < sizeof(colorTable64.colorClasses); ++i)
  {
    nextColorClass = colorTable[i];

    if(nextColorClass != currentColorClass)
    {
      stream << currentLength << currentColorClass;
      currentColorClass = nextColorClass;
      currentLength = 1;
    }
    else
      ++currentLength;
  }
  return stream << currentLength << currentColorClass;
}

In& operator>>(In& stream, ColorTable64& colorTable64)
{
  // a pointer to the begin of the color table
  unsigned char* colorTable = &colorTable64.colorClasses[0][0][0];

  // the current color class in the color table
  unsigned char currentColorClass;

  // the number of bytes in one sequence that have the same color class
  unsigned int currentLength;

  // the position in the color table
  unsigned int i = 0;

  while(i < sizeof(colorTable64.colorClasses))
  {
    stream >> currentLength;
    ASSERT(currentLength);
    stream >> currentColorClass;

    for(unsigned int end = std::min(i + currentLength, (unsigned) sizeof(colorTable64.colorClasses)); i < end; ++i)
      colorTable[i] = currentColorClass;
  }
  return stream;
}

bool ColorTable64::hasXNeighbors(unsigned char y, unsigned char u, unsigned char v, int x, unsigned char cc[64][64][64])
{
  int neighbours = 0;
  unsigned char c = cc[y][u][v];
  if(y > 0)
  {
    if(cc[y - 1][u][v] == c)
    {
      neighbours++;
    }
  }
  if(u > 0)
  {
    if(cc[y][u - 1][v] == c)
    {
      neighbours++;
    }
  }
  if(v > 0)
  {
    if(cc[y][u][v - 1] == c)
    {
      neighbours++;
    }
  }
  if(y < 63)
  {
    if(cc[y + 1][u][v] == c)
    {
      neighbours++;
    }
  }
  if(u < 63)
  {
    if(cc[y][u + 1][v] == c)
    {
      neighbours++;
    }
  }
  if(v < 63)
  {
    if(cc[y][u][v + 1] == c)
    {
      neighbours++;
    }
  }
  if(neighbours >= x)
  {
    return true;
  }
  else
  {
    return false;
  }
}

unsigned char ColorTable64::getNeighborColor(unsigned char y, unsigned char u, unsigned char v, unsigned char cc[64][64][64])
{
  unsigned char returnColor = cc[y][u][v];
  if(returnColor != ColorClasses::none)
  {
    return returnColor;
  }
  else
  {
    if(y > 0)
    {
      if(cc[y - 1][u][v] != ColorClasses::none)
      {
        returnColor = cc[y - 1][u][v];
      }
    }
    if(u > 0)
    {
      if(cc[y][u - 1][v] != ColorClasses::none)
      {
        returnColor = cc[y][u - 1][v];
      }
    }
    if(v > 0)
    {
      if(cc[y][u][v - 1] != ColorClasses::none)
      {
        returnColor = cc[y][u][v - 1];
      }
    }
    if(y < 63)
    {
      if(cc[y + 1][u][v] != ColorClasses::none)
      {
        returnColor = cc[y + 1][u][v];
      }
    }
    if(u < 63)
    {
      if(cc[y][u + 1][v] != ColorClasses::none)
      {
        returnColor = cc[y][u + 1][v];
      }
    }
    if(v < 63)
    {
      if(cc[y][u][v + 1] != ColorClasses::none)
      {
        returnColor = cc[y][u][v + 1];
      }
    }
    return returnColor;
  }
}

void ColorTable64::shrink()
{
  unsigned char cc2[64][64][64];
  memcpy(cc2, colorClasses, sizeof(colorClasses));
  unsigned char y, u, v;
  for(y = 0; y < 64; y++)
    for(u = 0; u < 64; u++)
      for(v = 0; v < 64; v++)
        if(!hasXNeighbors(y, u, v, 6, cc2))
          colorClasses[y][u][v] = ColorClasses::none;
}

void ColorTable64::shrink(unsigned char color)
{
  unsigned char cc2[64][64][64];
  memcpy(cc2, colorClasses, sizeof(colorClasses));
  unsigned char y, u, v;
  for(y = 0; y < 64; y++)
    for(u = 0; u < 64; u++)
      for(v = 0; v < 64; v++)
        if(!hasXNeighbors(y, u, v, 6, cc2) && cc2[y][u][v] == color)
          colorClasses[y][u][v] = ColorClasses::none;
}

void ColorTable64::grow()
{
  unsigned char cc2[64][64][64];
  memcpy(cc2, colorClasses, sizeof(colorClasses));
  unsigned char y, u, v;
  for(y = 0; y < 64; y++)
    for(u = 0; u < 64; u++)
      for(v = 0; v < 64; v++)
        if(hasXNeighbors(y, u, v, 1, cc2))
          colorClasses[y][u][v] = getNeighborColor(y, u, v, cc2);
}

void ColorTable64::grow(unsigned char color)
{
  unsigned char cc2[64][64][64];
  memcpy(cc2, colorClasses, sizeof(colorClasses));
  unsigned char y, u, v;
  for(y = 0; y < 64; y++)
    for(u = 0; u < 64; u++)
      for(v = 0; v < 64; v++)
        if(hasXNeighbors(y, u, v, 1, cc2) && getNeighborColor(y, u, v, cc2) == color)
          colorClasses[y][u][v] = getNeighborColor(y, u, v, cc2);
}

void ColorTable64::addHSIRange(unsigned char hMin, unsigned char hMax,
                               unsigned char sMin, unsigned char sMax,
                               unsigned char iMin, unsigned char iMax,
                               ColorClasses::Color c)
{
  for(unsigned char y = 0; y < 64; ++y)
    for(unsigned char u = 0; u < 64; ++u)
      for(unsigned char v = 0; v < 64; ++v)
      {
        unsigned char h, s, i;
        ColorModelConversions::fromYCbCrToHSI(y * 4, u * 4, v * 4, h, s, i);
        if(i >= iMin && i <= iMax && s >= sMin && s <= sMax &&
           ((h >= hMin && h <= hMax) || (hMin > hMax && (h >= hMin || h <= hMax))))
          colorClasses[y][u][v] = c;
      }
}

Image::Pixel ColorTable64::getAverageColor(ColorClasses::Color colorClass) const
{
  int sumY = 0, sumCb = 0, sumCr = 0, count = 0;
  for(int y = 0; y < 64; y++)
  {
    for(int cb = 0; cb < 64; cb++)
    {
      for(int cr = 0; cr < 64; cr++)
      {
        if(colorClasses[y][cb][cr] == colorClass)
        {
          sumY += y;
          sumCb += cb;
          sumCr += cr;
          ++count;
        }
      }
    }
  }

  Image::Pixel temp;
  if(count)
  {
    temp.y = sumY * 4 / count;
    temp.cb = sumCb * 4 / count;
    temp.cr = sumCr * 4 / count;
  }
  else
  {
    temp.y = temp.cb = temp.cr = 0;
  }
  temp.yCbCrPadding = 0;

  return temp;
}

#ifdef WIN32
#define snprintf _snprintf
#endif

std::string ColorTable64::hash() const
{
  const size_t SHA1_LEN = 20;
  unsigned char md[SHA1_LEN];
  Sha1::sha1(colorClasses[0][0], sizeof(colorClasses), md);
  std::stringstream buf(std::stringstream::in | std::stringstream::out);
  for(size_t i = 0; i < SHA1_LEN; ++i)
  {
    char b[3];
    snprintf(b, 3, "%02x", md[i]);
    buf << b;
  }
  return buf.str();
}
