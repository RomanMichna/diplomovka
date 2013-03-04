/**
 * @file Image.cpp
 *
 * Implementation of class Image.
 */

#include <cstring>

#include "Image.h"
#include "Tools/ImageProcessing/ColorModelConversions.h"
#include "Platform/BHAssert.h"

Image::Image(bool initialize) : timeStamp(0), isReference(false), resolutionWidth(cameraResolutionWidth), resolutionHeight(cameraResolutionHeight)
{
  image = new Pixel[cameraResolutionHeight][cameraResolutionWidth * 2];
  if(initialize)
    for(int y = 0; y < cameraResolutionHeight; ++y)
      for(int x = 0; x < cameraResolutionWidth; ++x)
        image[y][x].color = 0x80008000;
}

Image::Image(const Image& other)
  : isReference(true)
{
  *this = other;
}

Image::~Image()
{
  if(!isReference)
    delete [] image;
}

Image& Image::operator=(const Image& other)
{
  if(isReference)
  {
    image = new Pixel[cameraResolutionHeight][cameraResolutionWidth * 2];
    isReference = false;
  }
  resolutionHeight = other.resolutionHeight;
  resolutionWidth = other.resolutionWidth;
  timeStamp = other.timeStamp;
  for(int y = 0; y < resolutionHeight; ++y)
    memcpy(&image[y], &other.image[y], resolutionWidth * sizeof(Image::Pixel));
  return *this;
}

void Image::setImage(const unsigned char* buffer)
{
  if(!isReference)
  {
    delete [] image;
    isReference = true;
  }
  image = (Pixel (*)[cameraResolutionWidth * 2]) buffer;
}

void Image::convertFromYCbCrToRGB(const Image& ycbcrImage)
{
  resolutionHeight = ycbcrImage.resolutionHeight;
  resolutionWidth = ycbcrImage.resolutionWidth;
  for(int y = 0; y < resolutionHeight; ++y)
    for(int x = 0; x < resolutionWidth; ++x)
      ColorModelConversions::fromYCbCrToRGB(ycbcrImage.image[y][x].y,
                                            ycbcrImage.image[y][x].cb,
                                            ycbcrImage.image[y][x].cr,
                                            image[y][x].r,
                                            image[y][x].g,
                                            image[y][x].b);
}

void Image::convertFromRGBToYCbCr(const Image& rgbImage)
{
  resolutionHeight = rgbImage.resolutionHeight;
  resolutionWidth = rgbImage.resolutionWidth;
  for(int y = 0; y < resolutionHeight; ++y)
    for(int x = 0; x < resolutionWidth; ++x)
      ColorModelConversions::fromRGBToYCbCr(rgbImage.image[y][x].r,
                                            rgbImage.image[y][x].g,
                                            rgbImage.image[y][x].b,
                                            image[y][x].y,
                                            image[y][x].cb,
                                            image[y][x].cr);
}

void Image::convertFromYCbCrToHSI(const Image& ycbcrImage)
{
  resolutionHeight = ycbcrImage.resolutionHeight;
  resolutionWidth = ycbcrImage.resolutionWidth;
  for(int y = 0; y < resolutionHeight; ++y)
    for(int x = 0; x < resolutionWidth; ++x)
      ColorModelConversions::fromYCbCrToHSI(ycbcrImage.image[y][x].y,
                                            ycbcrImage.image[y][x].cb,
                                            ycbcrImage.image[y][x].cr,
                                            image[y][x].h,
                                            image[y][x].s,
                                            image[y][x].i);
}

void Image::convertFromHSIToYCbCr(const Image& hsiImage)
{
  resolutionHeight = hsiImage.resolutionHeight;
  resolutionWidth = hsiImage.resolutionWidth;
  for(int y = 0; y < resolutionHeight; ++y)
    for(int x = 0; x < resolutionWidth; ++x)
      ColorModelConversions::fromHSIToYCbCr(hsiImage.image[y][x].h,
                                            hsiImage.image[y][x].s,
                                            hsiImage.image[y][x].i,
                                            image[y][x].y,
                                            image[y][x].cb,
                                            image[y][x].cr);
}

void Image::convertFromYCbCrToTSL(const Image& ycbcrImage)
{
  resolutionHeight = ycbcrImage.resolutionHeight;
  resolutionWidth = ycbcrImage.resolutionWidth;
  for(int y = 0; y < resolutionHeight; ++y)
    for(int x = 0; x < resolutionWidth; ++x)
      ColorModelConversions::fromYCbCrToTSL(ycbcrImage.image[y][x].y,
                                            ycbcrImage.image[y][x].cb,
                                            ycbcrImage.image[y][x].cr,
                                            image[y][x].t,
                                            image[y][x].s,
                                            image[y][x].l);
}

void Image::serialize(In* in, Out* out)
{
  STREAM_REGISTER_BEGIN;
  STREAM(resolutionWidth);
  STREAM(resolutionHeight);

  STREAM(timeStamp);

  if(out)
    for(int y = 0; y < resolutionHeight; ++y)
      out->write(&image[y], resolutionWidth * sizeof(Pixel));
  else
  {
    for(int y = 0; y < resolutionHeight; ++y)
      in->read(&image[y], resolutionWidth * sizeof(Pixel));
  }

  STREAM_REGISTER_FINISH;
}

float Image::getColorDistance(const Image::Pixel& a, const Image::Pixel& b)
{
  int dy = int(a.y) - b.y;
  int dcb = int(a.cb) - b.cb;
  int dcr = int(a.cr) - b.cr;
  dy *= dy;
  dcb *= dcb;
  dcr *= dcr;
  return sqrt(float(dy + dcb + dcr));
}
