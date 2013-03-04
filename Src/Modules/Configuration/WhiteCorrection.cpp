/**
* @file WhiteCorrection.cpp
* This file implements a module that can provide white corrected cameraSettings
* @author Alexander Haertl
*/

#include "WhiteCorrection.h"
#include "Tools/ImageProcessing/ColorModelConversions.h"
#include <mmintrin.h>

WhiteCorrection::WhiteCorrection() : state(Idle), measurement(0), framesWaited(0) {}

void WhiteCorrection::getAverageYCbCrMMX(Image::Pixel& color)
{
  __m64* pSrc = (__m64*)theImage.image[0]; // pointer to image data
  __m64 zero = _mm_setzero_si64(); // constant 0x0
  __m64 curDoublePixel; // variable holding two consecutive pixels (8 bytes)
  __m64 curPixel; // variable holding current pixel (4 shorts)
  __m64 acc; // accumulator for at most 256 pixels (4 shorts)
  __m64 accLower = zero; // accumulator for lower part of pixel (2 ints)
  __m64 accUpper = zero; // accumulator for upper part of pixel (2 ints)

  const int width = theImage.resolutionWidth;
  const int height = theImage.resolutionHeight;

  for(int yy = 0; yy < height; yy++)
  {
    int remainingPixelsInRow = width;
    while(remainingPixelsInRow > 0)
    {
      // at most 256 pixels can be accumulated in inner loop
      const int limit = std::min(256, remainingPixelsInRow);
      remainingPixelsInRow -= limit;
      acc = zero; // clear accumulator
      for(int i = 0; i < limit; i += 2) // two pixels at a time are processed
      {
        curDoublePixel = *pSrc; // load pixel
        ++pSrc;
        curPixel = _mm_unpacklo_pi8(curDoublePixel, zero); // "cast" lower four bytes (one pixel) to shorts
        acc = _mm_add_pi16(acc, curPixel); // accumulate first pixel
        curDoublePixel = _mm_unpackhi_pi8(curDoublePixel, zero); // "cast" upper four bytes (other pixel) to shorts
        acc = _mm_add_pi16(acc, curDoublePixel); // accumulate other pixel
      }
      curPixel = _mm_unpacklo_pi16(acc, zero); // "cast" lower two shorts to ints
      acc = _mm_unpackhi_pi16(acc, zero); // "cast" upper two shorts to ints
      accLower = _mm_add_pi32(accLower, curPixel); // accumulate lower half
      accUpper = _mm_add_pi32(accUpper, acc); // accumulate upper half
    }
    pSrc += width / 2; // skip every second line
  }

  int colors[4];
  colors[0] = _mm_cvtsi64_si32(accLower); // ectract first value from MMX-register
  accLower = _mm_srli_si64(accLower, 32); // shift upper 32 bit to lower 32 bit
  colors[1] = _mm_cvtsi64_si32(accLower); // extract second value from MMX-register
  colors[2] = _mm_cvtsi64_si32(accUpper); // extract third value from MMX-register
  accUpper = _mm_srli_si64(accUpper, 32); // shift upper 32 bit to lower 32 bit
  colors[3] = _mm_cvtsi64_si32(accUpper); // extract fourth value from MMX-register

  // compute average
  const int size = width * height;
  for(int i = 0; i < 4; ++i)
  {
    color.channels[i] = colors[i] / size;
  }

  _mm_empty(); // clear MMX-registers
}

void WhiteCorrection::getAverageYCbCr(Image::Pixel& color)
{
  int y(0), cb(0), cr(0);
  // iterate all rows
  for(int yy = 0; yy < theImage.resolutionHeight; yy++)
  {
    Image::Pixel* p = theImage.image[yy];
    Image::Pixel* pEnd = p + theImage.resolutionWidth;
    while(p != pEnd) // iterate all pixels of a row
    {
      y += p->y;
      cb += p->cb;
      cr += p->cr;
      p++;
    }
  }
  const int size = theImage.resolutionHeight * theImage.resolutionWidth;
  y /= size;
  cb /= size;
  cr /= size;
  color.y = color.yCbCrPadding = y;
  color.cb = cb;
  color.cr = cr;
}


void WhiteCorrection::update(CameraSettings& cameraSettings)
{
  int measurements = 3; /**< number of consecutive frames that are averaged */
  int framesToWait = 3; /**< number of frames to wait for application of changed camera settings */

  MODIFY("module:WhiteCorrection:measurements", measurements);
  MODIFY("module:WhiteCorrection:framesToWait", framesToWait);

  DEBUG_RESPONSE_ONCE("module:WhiteCorrection:abortWhiteCorrection", state = Idle;);

  bool useMMX = true;
  MODIFY("module:WhiteCorrection:useMMX", useMMX);

  switch(state)
  {
  case(Measure):
    if(measurement == 0)
    {
      // start measurement
      accR = 0;
      accG = 0;
      accB = 0;
    }
    Image::Pixel avgColor;
    if(useMMX)
    {
      getAverageYCbCrMMX(avgColor);
    }
    else
    {
      getAverageYCbCr(avgColor);
    }
    Image::Pixel avgColorRGB;
    ColorModelConversions::fromYCbCrToRGB(avgColor.y, avgColor.cb, avgColor.cr, avgColorRGB.r, avgColorRGB.g, avgColorRGB.b);
    accR += avgColorRGB.r;
    accG += avgColorRGB.g;
    accB += avgColorRGB.b;
    if(++measurement >= measurements) // enough consecutive frames have been accumulated
    {
      accR /= measurements;
      accG /= measurements;
      accB /= measurements;
      state = Wait;
      framesWaited = 0;
    }
    break;
  case(Wait):
    if(framesWaited == 0) // before actual waiting set new camera settings
    {
      int diffB = accB - accG;
      int diffR = accR - accG;
      OUTPUT(idText, text, "diffB: " << diffB << " diffR: " << diffR);
    }
    if(++framesWaited >= framesToWait) // waited long enough so that new settings are surely applied
    {
      measurement = 0;
      if(abs(accB - accG) <= 1 && abs(accR - accG) <= 1) // converged
      {
        state = Idle;
        OUTPUT(idText, text, "converged");
      }
      else // not converged, continue measuring and refreshing settings
      {
        state = Measure;
      }
    }
    break;
  case(Idle):
    DEBUG_RESPONSE_ONCE("module:WhiteCorrection:startWhiteCorrection", state = Measure;);
    break;
  default:
    ASSERT(false);
  }
}

int WhiteCorrection::denormalizeGain(int gain)
{
  if(gain < 0)
  {
    return 0;
  }
  else if(gain < 16)
  {
    return gain;
  }
  else if(gain < 144)
  {
    return gain + 112;
  }
  else
    return 255;
}

int WhiteCorrection::normalizeGain(int gain)
{
  if(gain < 0)
  {
    return 0;
  }
  else if(gain < 32)
  {
    return gain;
  }
  else if(gain < 64)
  {
    return gain - 16;
  }
  else if(gain < 128)
  {
    return gain - 48;
  }
  else if(gain < 256)
  {
    return gain - 112;
  }
  else
    return 144;
}

MAKE_MODULE(WhiteCorrection, Infrastructure)
