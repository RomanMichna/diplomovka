/** 
* @file RunLengthImageProvider.cpp
* This file declares a class to compute a run length encoded image from the raw image.
* @author <a href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</a>
*/

#include "RunLengthImageProvider.h"
#if defined(TARGET_ROBOT) || defined(WIN32)
#include "../Util/assembler/Segmentation.h"
#endif
#include "Tools/Math/Common.h"
#include "Tools/Debugging/DebugDrawings.h"

MAKE_MODULE(RunLengthImageProvider, Perception);

RunLengthImageProvider::RunLengthImageProvider()
{
  createLutUsingC(lut, cameraResolutionWidth);
}

void RunLengthImageProvider::update(RunLengthImage& runLengthImage)
{
  DECLARE_PLOT("module:RunLengthImageProvider:runs");

  MODIFY("module:RunLengthImageProvider:parameters", parameters);
  Image::Pixel threshold;
  threshold.y = parameters.thresholdY;
  threshold.cb = parameters.thresholdCb;
  threshold.cr = parameters.thresholdCr;
  threshold.yCbCrPadding = 255;

#if defined(TARGET_ROBOT) || defined(WIN32)
    runLengthImage.numberOfRuns = segmentImageAverageWithGridUsingMMX((const void*)&theImage.image[0], (void*)runLengthImage.image, threshold.color, (const long long*)lut, (const short*)theImageGrid.lineInformation, cameraResolutionWidth, cameraResolutionHeight);
#else
    runLengthImage.numberOfRuns = segmentImageAverageUsingC((const void*)&theImage.image[0], (void*)runLengthImage.image, threshold.color, (const short*)theImageGrid.lineInformation, cameraResolutionWidth, cameraResolutionHeight);
#endif

  RunLengthImage::Run& guard = runLengthImage.image[runLengthImage.numberOfRuns];
  guard.color.color = 0;
  guard.firstOverlapAbove = guard.firstOverlapBelow = &runLengthImage.voidRun;
  guard.xStart = guard.xEnd = guard.y = guard.runLength = 0;
  runLengthImage.image[runLengthImage.numberOfRuns + 1] = guard;

  PLOT("module:RunLengthImageProvider:runs", runLengthImage.numberOfRuns);

  STOP_TIME_ON_REQUEST("module:RunLengthImageProvider:extendRunLengthImage",
    extendRunLengthImage(runLengthImage);
  );

  runLengthImage.timestamp = theImage.timeStamp;
  runLengthImage.draw(theImageGrid);
}

int RunLengthImageProvider::segmentImageAverageUsingC(const void* pSrc, void* pDst, const int threshold, const short* grid, const int width, const int height)
{
  const Image::Pixel* p = (const Image::Pixel*)pSrc;
  RunLengthImage::Run* dst = (RunLengthImage::Run*)pDst;
  const ImageGrid::LineInformation* pGrid = (const ImageGrid::LineInformation*)grid;
  Image::Pixel pThreshold;
  pThreshold.color = threshold;
  
  for(int y = 0; y < height;)
  {
    const Image::Pixel* pEnd = p + width; // points to end of line
    const int columnInterval = pGrid->columnInterval;
    int sumY = p->y;
    int sumCb = p->cb;
    int sumCr = p->cr;
    int runLength = 1;
    p += columnInterval;
    while(p < pEnd)
    {
      // compute difference to average and check if it is within threshold
      // use two multiplications instead of one division
      if(abs(p->y * runLength - sumY) > pThreshold.y * runLength ||
         abs(p->cb * runLength - sumCb) > pThreshold.cb * runLength ||
         abs(p->cr * runLength - sumCr) > pThreshold.cr * runLength) // end of run
      {
        // color of run is average color
        dst->color.y = (unsigned char)(sumY / runLength);
        dst->color.cb = (unsigned char)(sumCb / runLength);
        dst->color.cr = (unsigned char)(sumCr / runLength);
        dst->runLength = runLength * columnInterval;
        ++dst;
        sumY = p->y;
        sumCb = p->cb;
        sumCr = p->cr;
        runLength = 1;
      }
      else
      {
        sumY += p->y;
        sumCb += p->cb;
        sumCr += p->cr;
        ++runLength;
      }
      p += columnInterval;
    }
    // store line-ending run
    dst->color.y = (unsigned char)(sumY / runLength);
    dst->color.cb = (unsigned char)(sumCb / runLength);
    dst->color.cr = (unsigned char)(sumCr / runLength);
    dst->runLength = runLength * columnInterval - (p - pEnd);
    ++dst;
    p = pEnd + width * (pGrid->rowInterval * 2 - 1); // skip every second line
    y += pGrid->rowInterval;
    ++pGrid;
  }
  // mark end of runs
  dst->color.color = 0;
  dst->runLength = 0;
  return dst - (RunLengthImage::Run*)pDst;
}

void RunLengthImageProvider::createLutUsingC(__m64* lut, const int width)
{
  ASSERT(width >= 1);
  union LutEntry
  {
    unsigned short shorts[4];
    long long longLong;
  };
  LutEntry* lutEntries = (LutEntry*)lut;
  LutEntry& firstEntry = lutEntries[0];
  firstEntry.shorts[0] = firstEntry.shorts[1] = firstEntry.shorts[2] = firstEntry.shorts[3] = 65535;
  for(int i = 2; i <= width; i++)
  {
    unsigned short reciprocal = 65536 / i;
    LutEntry& e = lutEntries[i - 1];
    e.shorts[0] = e.shorts[1] = e.shorts[2] = e.shorts[3] = reciprocal;
  }
}

void RunLengthImageProvider::extendRunLengthImage(RunLengthImage& runLengthImage)
{
  const ImageGrid::LineInformation* pGrid = theImageGrid.lineInformation;
  int x = 0, y = 0;

  RunLengthImage::Run* p0 = runLengthImage.image;
  RunLengthImage::Run* p1 = p0;
  // process first line
  while(x < cameraResolutionWidth)
  {
    p1->y = y;
    p1->xStart = x;
    x += p1->runLength;
    p1->xEnd = x - 1;
    ++p1;
  }
  x = 0;
  y += pGrid->rowInterval;
  ++pGrid;

  // process second line
  // meanwhile set firstOverlapBelow of first line
  RunLengthImage::Run* p = p0;
  RunLengthImage::Run* p2 = p1;
  const RunLengthImage::Run* pEnd = p1;
  while(p < pEnd)
  {
    while(x <= p->xStart)
    {
      p2->y = y;
      p2->xStart = x;
      x += p2->runLength;
      p2->xEnd = x - 1;
      ++p2;
    }
    p->firstOverlapAbove = &runLengthImage.voidRun;
    p->firstOverlapBelow = p2 - 1;
    ++p;
  }
  // process p2 until beginning of next line
  while(x < cameraResolutionWidth)
  {
    p2->y = y;
    p2->xStart = x;
    x += p2->runLength;
    p2->xEnd = x - 1;
    ++p2;
  }

  x = 0;
  y += pGrid->rowInterval;
  ++pGrid;

  // p0 now points to first run of first line, p1 points to first run of second line,
  // and p2 points to first run of third line

  // process all remaining lines
  while(y < cameraResolutionHeight)
  {
    RunLengthImage::Run* const pEnd1 = p1;
    RunLengthImage::Run* const pEnd2 = p2;
    // process one line
    while(p1 < pEnd2)
    {
      // advance pointer in previous line until it overlaps
      while(p0->xEnd < p1->xStart)
      {
        ++p0;
      }
      p1->firstOverlapAbove = p0;
      // advance pointer in next line until it has overlapped and set coordinates simultaneously
      while(x <= p1->xStart)
      {
        p2->y = y;
        p2->xStart = x;
        x += p2->runLength;
        p2->xEnd = x - 1;
        ++p2;
      }
      p1->firstOverlapBelow = p2 - 1;
      ++p1;
    }
    // advance p0 to beginning of next line
    p0 = pEnd1;
    // process p2 until beginning of next line
    while(x < cameraResolutionWidth)
    {
      p2->y = y;
      p2->xStart = x;
      x += p2->runLength;
      p2->xEnd = x - 1;
      ++p2;
    }
    x = 0;
    y += pGrid->rowInterval;
    ++pGrid;
  }

  runLengthImage.beginOfLastLine = p1;

  // process last line (firstOverlap[Above|Below] is still missing)
  pEnd = runLengthImage.image + runLengthImage.numberOfRuns;
  while(p1 < pEnd)
  {
    while(p0->xEnd < p1->xStart)
    {
      ++p0;
    }
    p1->firstOverlapAbove = p0;
    p1->firstOverlapBelow = &runLengthImage.voidRun;
    ++p1;
  }
}
