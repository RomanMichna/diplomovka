/**
* @file BallPerceptorRLE2.cpp
* This file implements a module that provides a ball percept only using the RunLengthImage
* The ball center / radius calculation algorithm is based on the BallSpecialist in GT2005.
* @author Alexander Härtl
*/

#include "BallPerceptorRLE2.h"
#include "Tools/Debugging/DebugDrawings.h"
#include <algorithm>
#include <cstdlib>
#include "../Util/assembler/Segmentation.h"
#include "Tools/ConvexHull.h"
#include "Tools/Math/Matrix.h"
#include "Tools/ImageProcessing/ColorModelConversions.h"
#include "Tools/Math/Geometry.h"
#include "Modules/Perception/RunLengthImageProvider.h"

MAKE_MODULE(BallPerceptorRLE2, Perception)

void BallPerceptorRLE2::init()
{
  ASSERT(sizeof(int) == sizeof(ExtendedRun));

  // create color LUT
  const Vector2<int> center(127, 127);
  for(int cr = 0; cr < 256; ++cr)
  {
    for(int cb = 0; cb < 256; ++cb)
    {
      ColorLutEntry& e = colorLUT[cr * 256 + cb];

      Vector2<int> color(cb, cr);
      color -= center;

      e.saturation = color.abs();

      color.normalize(127);
      color += center;

      e.cb = color.x;
      e.cr = color.y;
    }
  }

  RunLengthImageProvider::createLutUsingC(divisionLUT, 128);

  DECLARE_PLOT("module:BallPerceptorRLE2:distance");
  DECLARE_PLOT("module:BallPerceptorRLE2:distanceRadius");
  DECLARE_PLOT("module:BallPerceptorRLE2:numberOfRuns");
  DECLARE_PLOT("module:BallPerceptorRLE2:totalNumberOfRuns");
  DECLARE_PLOT("module:BallPerceptorRLE2:neckAngle");

  const float focalLengthInv2 = sqr(theCameraInfo.focalLengthInv);
  P2 = Matrix<3, 3>(focalLengthInv2);
  P2[2][2] = 1.0f;
}

void BallPerceptorRLE2::preExecution()
{
  MODIFY("module:BallPerceptorRLE2:parameters", parameters);

  const Vector3<>& xAxis3D = theCameraMatrix.rotation.c0;
  const Vector2<> xAxis(Vector2<>(xAxis3D.x, xAxis3D.y).abs(), xAxis3D.z);

  cameraHeight = max(200.0f, theCameraMatrix.translation.z - theFieldDimensions.ballRadius);

  // precompute values to efficiently compute the projected width of the ball diameter in the image
  // this assumes that the camera is (almost) not rotated around the x-axis, which is usually the case
  // the width in image coordinates can be computed by widthSlope * y + widthIntercept
  widthSlope = (float)(theFieldDimensions.ballRadius * 2) * xAxis.x / cameraHeight;
  widthIntercept = (float)(theFieldDimensions.ballRadius * 2) * -(theCameraInfo.opticalCenter.y * xAxis.x + theCameraInfo.focalLength * xAxis.y) / cameraHeight;

  // project a point that is field diagonal distant into the image
  float fieldDiagonal = Vector2<>((float)theFieldDimensions.xPosOpponentGroundline, (float)theFieldDimensions.yPosLeftSideline).abs() * 2.0f;
  Vector2<> startInCameraCoords(xAxis.x * fieldDiagonal - xAxis.y * cameraHeight, -xAxis.y * fieldDiagonal - xAxis.x * cameraHeight);

  startCoord = (int)(theCameraInfo.opticalCenter.y - startInCameraCoords.y * theCameraInfo.focalLength / startInCameraCoords.x);
  
  ColorLutEntry threshold;
  threshold.cb = parameters.thresholdCb;
  threshold.cr = parameters.thresholdCr;
  threshold.saturation = parameters.thresholdSaturation;
  threshold.padding = parameters.thresholdY;
  mThreshold = _mm_unpacklo_pi8(_mm_cvtsi32_si64(threshold.dword), _mm_setzero_si64());
  MM_EMPTY();
}

void BallPerceptorRLE2::update(BallPercept& ballPercept)
{
  preExecution();
  ballPercept.ballWasSeen = false;

  if(!theCameraMatrix.isValid)
    return;

  Image::Pixel orange = theColorConfiguration.colors[ColorClasses::orange].color.convertToPixel();
  orange.yCbCrPadding = 0;

  if(startCoord < cameraResolutionHeight)
  { // only do something if a ball could be in sight
    // if the actual start coordinate is above the image, start at beginning of the image
    if(startCoord < 0)
    {
      startCoord = 0;
    }
    const RunLengthImage::Run* pRun = theRunLengthImage.image;
    // process at most maxNumberOfRuns runs
    const RunLengthImage::Run* pGuard = theRunLengthImage.image + max(0, theRunLengthImage.numberOfRuns - parameters.maxNumberOfRuns);

    // move pRun to the start (either defined by pGuard or the y-start-coordinate)
    if(startCoord >= theImageGrid.lastYCoord)
    {
      pRun = theRunLengthImage.image + theRunLengthImage.numberOfRuns - 1;
    }
    else
    {
      while(pRun <= pGuard || pRun->y <= startCoord)
      {
        pRun = pRun->firstOverlapBelow;
      }
    }
    while(pRun >= pGuard && pRun->y >= startCoord)
    {
      --pRun;
    }
    ++pRun;

    DECLARE_DEBUG_DRAWING("module:BallPerceptorRLE2:startCoord", "drawingOnImage");
    LINE("module:BallPerceptorRLE2:startCoord", 0, pRun->y, cameraResolutionWidth, pRun->y, 1, Drawings::ps_solid, ColorClasses::yellow);

    int numberOfRuns = theRunLengthImage.image + theRunLengthImage.numberOfRuns - pRun;
    PLOT("module:BallPerceptorRLE2:numberOfRuns", numberOfRuns);
    PLOT("module:BallPerceptorRLE2:totalNumberOfRuns", theRunLengthImage.numberOfRuns);
    PLOT("module:BallPerceptorRLE2:neckAngle", theFilteredJointData.angles[JointData::HeadPitch]);

    bool (BallPerceptorRLE2::*buildCircleFunc)(const RunLengthImage::Run*, BoolArray&, bool) = parameters.useMMX ? &BallPerceptorRLE2::buildCircleMMX : &BallPerceptorRLE2::buildCircle;
    if(parameters.preprocessing)
    {
      ExtendedRun* promisingRuns = new ExtendedRun[numberOfRuns + 2]; // additional space for MMX access

      void (BallPerceptorRLE2::*colorDifferenceFunc)(Image::Pixel, const RunLengthImage::Run*, ExtendedRun*, int numberOfRuns) const = 
#ifndef MACOSX
        parameters.useMMX ? &BallPerceptorRLE2::buildColorDifferencesMMXIntrinsics : 
#endif
        &BallPerceptorRLE2::buildColorDifferences;
      void (BallPerceptorRLE2::*widthDifferenceFunc)(const RunLengthImage::Run*, ExtendedRun*, int numberOfRuns) const = parameters.useMMX ? &BallPerceptorRLE2::buildWidthDifferencesMMX : &BallPerceptorRLE2::buildWidthDifferences2;

      // rate all processed runs and sort them by the rating
      STOP_TIME_ON_REQUEST_WITH_PLOT("module:BallPerceptorRLE2:buildColorDifferences", (this->*colorDifferenceFunc)(orange, pRun, promisingRuns, numberOfRuns); );
      STOP_TIME_ON_REQUEST_WITH_PLOT("module:BallPerceptorRLE2:buildWidthDifferences", (this->*widthDifferenceFunc)(pRun, promisingRuns, numberOfRuns); );
      STOP_TIME_ON_REQUEST_WITH_PLOT("module:BallPerceptorRLE2:sort", partial_sort(promisingRuns, promisingRuns + min(parameters.sortedRuns, numberOfRuns), promisingRuns + numberOfRuns); );

      drawRatingBallRuns(promisingRuns, pRun, numberOfRuns);

      // visitedRuns marks whether a run has already been visited
      const ExtendedRun* p = promisingRuns;
      const ExtendedRun* const pEnd = promisingRuns + numberOfRuns;
      const int offset = pRun - theRunLengthImage.image;
      STOP_TIME_ON_REQUEST_WITH_PLOT("module:BallPerceptorRLE2:buildCircles",
      {
        potentialBalls.clear();
        BoolArray visitedRuns(theRunLengthImage.numberOfRuns);
        for(int i = 0; i < parameters.promisingRuns; ++i)
        {
          // find first run that has not been visited
          while(p < pEnd && visitedRuns[p->runIndex + offset])
          {
            ++p;
          }
          if(p == pEnd)
          {
            break;
          }
          (this->*buildCircleFunc)(pRun + p->runIndex, visitedRuns, true);
          ++p;
        }
      });
      delete[] promisingRuns;
    }
    else
    {
      STOP_TIME_ON_REQUEST_WITH_PLOT("module:BallPerceptorRLE2:buildCircles",
      {
        potentialBalls.clear();
        const RunLengthImage::Run* const pRunEnd = theRunLengthImage.image + theRunLengthImage.numberOfRuns;
        const RunLengthImage::Run* p = pRun;
        BoolArray visitedRuns(theRunLengthImage.numberOfRuns);
        while(p < pRunEnd)
        {
          const int currentY = p->y;
          const int maxWidthForYCoord = (int)ceil(getWidthInImage((float)currentY) + parameters.additionalWidth);
          while(p->y == currentY)
          {
            if(p->runLength <= maxWidthForYCoord && !visitedRuns[p - theRunLengthImage.image])
            {
              (this->*buildCircleFunc)(p, visitedRuns, false);
            }
            ++p;
          }
        }
      });
    }
  }

  bool (BallPerceptorRLE2::*centerRadiusFunc)(BallPerceptorRLE2::PotentialBall&) const = parameters.useRANSAC ? &BallPerceptorRLE2::getCenterAndRadiusRANSAC : &BallPerceptorRLE2::getCenterAndRadius;
  STOP_TIME_ON_REQUEST_WITH_PLOT("module:BallPerceptorRLE2:sanityChecks",
    // filter out potential balls that are too small or that do not match the color
    // TODO maybe accelerate with MMX, although maybe not necessary
    const ColorConfiguration::ColorPrototype thresholdOrange = theColorConfiguration.colors[ColorClasses::orange].threshold;
    for(list<PotentialBall>::iterator i = potentialBalls.begin(); i != potentialBalls.end(); ++i)
    {
      i->valid = true;
      if(i->runs.size() < parameters.minRunsPerRegion ||
        abs(i->avgColor.y - orange.y) > thresholdOrange.y ||
        abs(i->avgColor.cb - orange.cb) > thresholdOrange.cb ||
        abs(i->avgColor.cr - orange.cr) > thresholdOrange.cr)
      {
        i->valid = false;
      }
      else
      {
        extendCircle(*i);
        if(!(this->*centerRadiusFunc)(*i) || !getRelativePosition(*i) || !checkBackProjection(*i))
        {
          i->valid = false;
        }
      }
    }
  );

  for(list<PotentialBall>::iterator i = potentialBalls.begin(); i != potentialBalls.end();)
  {
    if(i->valid)
    {
      ++i;
    }
    else
    {
      i = potentialBalls.erase(i);
    }
  }

  if(!potentialBalls.empty())
  {
    ballPercept.ballWasSeen = true;
    ballPercept.relativePositionOnField = potentialBalls.front().relPosOnField;
    ballPercept.radiusInImage = potentialBalls.front().radius;
    ballPercept.positionInImage = potentialBalls.front().centerInImage;

    PLOT("module:BallPerceptorRLE2:distance", ballPercept.relativePositionOnField.abs());
    PLOT("module:BallPerceptorRLE2:distanceRadius", potentialBalls.front().radiusBasedDistance);
  }

  draw();
  drawnormalizedImages();
}

void BallPerceptorRLE2::buildColorDifferences(Image::Pixel orange, const RunLengthImage::Run* pSrc, ExtendedRun* pDst, int numberOfRuns) const
{
  const RunLengthImage::Run* pRun = (const RunLengthImage::Run*)pSrc;
  ExtendedRun* p = pDst;
  for(int i = 0; i < numberOfRuns; ++i)
  {
    Image::Pixel tempPixel;
    ExtendedRun& temp = p[i];
    tempPixel = pRun[i].color;
    // color distance is the sum of differences of the single channels to the prototypical orange
    temp.colorDistance = abs(orange.y - tempPixel.y) + abs(orange.cb - tempPixel.cb) + abs(orange.cr - tempPixel.cr);
    temp.runIndex = i;
  }
}

void BallPerceptorRLE2::buildColorDifferencesMMXIntrinsics(Image::Pixel orange, const RunLengthImage::Run* pSrc, ExtendedRun* pDst, int numberOfRuns) const
{
  orange.yCbCrPadding = 0;
  Image::Pixel mask;
  mask.y = mask.cb = mask.cr = 255;
  mask.yCbCrPadding = 0;
  const __m64 mOrange = _mm_cvtsi32_si64(orange.color);
  const __m64 mMask = _mm_cvtsi32_si64(mask.color);
  int runIndex = 0;
  while(runIndex < numberOfRuns)
  {
    __m64 mColor = *(__m64*)&pSrc->color; // load color
    ++pSrc;
    __m64 mColorDiff = _mm_sad_pu8(_mm_and_si64(mColor, mMask), mOrange); // mask out the three relevant channels and comute sum of absolute differences
    mColorDiff = _mm_insert_pi16(mColorDiff, runIndex, 1); // insert the run index into the upper word of the result
    ++runIndex;
    *((int*)pDst) = _mm_cvtsi64_si32(mColorDiff); // write result
    ++pDst;
  }
  MM_EMPTY(); // clear MMX state
}

void BallPerceptorRLE2::buildWidthDifferences(const RunLengthImage::Run* runs, ExtendedRun* extendedRuns, const int numberOfRuns) const
{
  const ExtendedRun* const pEnd = extendedRuns + numberOfRuns;
  while(extendedRuns != pEnd)
  {
    extendedRuns->colorDistance += abs((int)getWidthInImage((float)runs->y) - runs->runLength) * parameters.widthPenalty;
    ++runs;
    ++extendedRuns;
  }
}

void BallPerceptorRLE2::buildWidthDifferences2(const RunLengthImage::Run* runs, ExtendedRun* extendedRuns, const int numberOfRuns) const
{
  const ExtendedRun* const pEnd = extendedRuns + numberOfRuns;
  while(extendedRuns != pEnd)
  {
    int y = runs->y;
    int width = (int)getWidthInImage((float)runs->y);
    do // process one line after the other
    {
      extendedRuns->colorDistance += abs(width - runs->runLength) * parameters.widthPenalty; // add scaled width difference to the color difference
      ++runs;
      ++extendedRuns;
    }
    while(runs->y == y);
  }
}

void BallPerceptorRLE2::buildWidthDifferencesMMX(const RunLengthImage::Run* runs, ExtendedRun* extendedRuns, const int numberOfRuns) const
{
  ASSERT(sizeof(ExtendedRun) == 4 && offsetof(ExtendedRun, colorDistance) == 0);
  // to avoid interference of float operations with MMX operations, the expected width computation must be done in integer arithmetics
  const int slope = (int)(widthSlope * (float)(1 << 24));
  const int intercept = (int)(widthIntercept * (float)(1 << 24));
  const ExtendedRun* const pEnd = extendedRuns + (numberOfRuns & 0xFFFFFFFE);
  const __m64 mFactor = _mm_set1_pi32(parameters.widthPenalty); // mFactor contains widthPenalty in the upper and lower DWORD
  const __m64 mMask = _mm_set1_pi32(0x0000FFFF); // to mask out the lower word of each DWORD
  int width = 0;
  while(extendedRuns < pEnd)
  {
    int y = runs->y;
    width = (slope * y + intercept) >> 24;
    ASSERT(width < 0x7FFF);
    const __m64 mWidth = _mm_set1_pi32(width); // load expected width into lower and upper word of mWidth
    do // process one line after the other, and always two runs at a time
    {
      const __m64 mRun1 = *(__m64*)runs; // load one run
      const __m64 mRun2 = *(__m64*)(runs + 1); // load second run
      __m64 mRunLengths = _mm_and_si64(_mm_unpackhi_pi32(mRun1, mRun2), mMask); // pack the run lengths of both runs into one register and mask out the run lengths
      // first compute the absolute difference of the run lengths and the expected width by two saturated subtractions and a logical or, and multiply the result by the width penalty factor
      const __m64 mScaledDifferences = _mm_mullo_pi16(_mm_or_si64(_mm_subs_pu16(mRunLengths, mWidth), _mm_subs_pu16(mWidth, mRunLengths)), mFactor);
      const __m64 mExtendedRuns = *(__m64*)extendedRuns; // load the extended run
      *(__m64*)extendedRuns = _mm_adds_pu16(mExtendedRuns, mScaledDifferences); // and add the width difference to the color difference (can be done packed because all but the lowest word are zero)
      runs += 2;
      extendedRuns += 2;
    }
    while(runs->y == y);
  }
  _mm_empty();

  // process last run if available
  if(numberOfRuns % 2)
  {
    extendedRuns->colorDistance += abs(width - runs->runLength) * parameters.widthPenalty;
  }
}

float BallPerceptorRLE2::getWidthInImage(float yCoord) const
{
  return widthSlope * yCoord + widthIntercept;
}

bool BallPerceptorRLE2::buildCircle(const RunLengthImage::Run* pRun, BoolArray& visitedRuns, bool searchUpAndDown)
{
  const RunLengthImage::Run* const pRunOrig = pRun;
  const RunLengthImage::Run* pRun2;

  PotentialBall ball;
  ball.runs.reserve(50); // avoid resizing
  const int y = pRun->y;
  ball.runs.push_back(pRun);
  ball.xMin = pRun->xStart;
  ball.xMax = pRun->xEnd;
  ball.yMin = ball.yMax = y;

  // a ball may be that wide for the current y coordinate
  // TODO compute correct length of a ball starting at that y-coordinate
  const float maxWidthForYCoord = getWidthInImage((float)y);
  const int maxWidth = (int)ceil(getWidthInImage((float)y + maxWidthForYCoord) + parameters.additionalWidth);

  visitedRuns[pRun - theRunLengthImage.image] = true; // mark run as visited
  // first search upwards
  ColorLutEntry lutEntry = colorLUT[pRun->color.cr * 256 + pRun->color.cb];
  // initialize accumulators
  int numOfPixels = pRun->runLength;
  int accY = pRun->color.y * numOfPixels, accCb = pRun->color.cb * numOfPixels, accCr = pRun->color.cr * numOfPixels, accNormCb = lutEntry.cb * numOfPixels, accNormCr = lutEntry.cr * numOfPixels, accSat = lutEntry.saturation * numOfPixels;

  unsigned int index = 0; // index of the currently processed run. An iterator cannot be used here because it could be invalidated in between
  if(searchUpAndDown)
  {
    pRun2 = pRun->firstOverlapAbove;
    if(pRun2 != &theRunLengthImage.voidRun)
    {
      for(;;)
      {
        // TODO speed up computation with MMX
        Image::Pixel temp = pRun2->color;
        ColorLutEntry lutEntry = colorLUT[temp.cr * 256 + temp.cb];
        if(!visitedRuns[pRun2 - theRunLengthImage.image] &&
          abs(temp.y * numOfPixels - accY) < parameters.thresholdY * numOfPixels &&
          abs(lutEntry.cb * numOfPixels - accNormCb) < parameters.thresholdCb * numOfPixels &&
          abs(lutEntry.cr * numOfPixels - accNormCr) < parameters.thresholdCr * numOfPixels &&
          abs(lutEntry.saturation * numOfPixels - accSat) < parameters.thresholdSaturation * numOfPixels)
        { // run is not visited and the color matches
          ball.runs.push_back(pRun2);
          ball.xMin = min(ball.xMin, pRun2->xStart);
          ball.xMax = max(ball.xMax, pRun2->xEnd);
          ball.yMin = min(ball.yMin, pRun2->y);
          if(ball.xMax - ball.xMin > maxWidth || ball.yMax - ball.yMin > maxWidth)
          {
            // a run that matches the color makes the bounding box too large, hence no ball
            return false;
          }
          visitedRuns[pRun2 - theRunLengthImage.image] = true; // mark run as visited
          const int runLength = pRun2->runLength;
          // update accumulators
          accY += temp.y * runLength;
          accCb += temp.cb * runLength;
          accCr += temp.cr * runLength;
          accNormCb += lutEntry.cb * runLength;
          accNormCr += lutEntry.cr * runLength;
          accSat += lutEntry.saturation * runLength;
          numOfPixels += runLength;
        }
        if(pRun2->xEnd < pRun->xEnd)
        {
          // there is another overlapping run in the previous row
          ++pRun2;
        }
        else
        {
          // check if there are remaining runs in the region
          if(++index >= ball.runs.size())
          {
            // there are no more matching runs, the region is finished preserving all constraints
            break;
          }
          pRun = ball.runs[index];
          pRun2 = pRun->firstOverlapAbove;
          if(pRun2 == &theRunLengthImage.voidRun)
          {
            // reached the end of image, the region is finished preserving all constraints
            break;
          }
        }
      }
    }

    // to keep the ascending order of runs, the runs found so far have to be reversed row-wise
    vector<const RunLengthImage::Run*> temp(ball.runs.size());
    vector<const RunLengthImage::Run*>::iterator i = ball.runs.end();
    while(i > ball.runs.begin())
    {
      int y = (*(i-1))->y;
      const vector<const RunLengthImage::Run*>::iterator iOld = i;
      while(i > ball.runs.begin() && (*(i - 1))->y == y)
      {
        --i;
      }
      copy(i, iOld, temp.begin() + (ball.runs.end() - iOld));
    }
    ball.runs.swap(temp);

    // now search downwards
    index = ball.runs.size() - 1;
  }

  pRun = pRunOrig;
  pRun2 = pRun->firstOverlapBelow;
  if(pRun2 != &theRunLengthImage.voidRun)
  {
    for(;;)
    {
      Image::Pixel temp = pRun2->color;
      ColorLutEntry lutEntry = colorLUT[temp.cr * 256 + temp.cb];
      if(!visitedRuns[pRun2 - theRunLengthImage.image] &&
        abs(temp.y * numOfPixels - accY) < parameters.thresholdY * numOfPixels &&
        abs(lutEntry.cb * numOfPixels - accNormCb) < parameters.thresholdCb * numOfPixels &&
        abs(lutEntry.cr * numOfPixels - accNormCr) < parameters.thresholdCr * numOfPixels &&
        abs(lutEntry.saturation * numOfPixels - accSat) < parameters.thresholdSaturation * numOfPixels)
      { // run is not visited and the color matches
        ball.runs.push_back(pRun2);
        ball.xMin = min(ball.xMin, pRun2->xStart);
        ball.xMax = max(ball.xMax, pRun2->xEnd);
        ball.yMax = max(ball.yMax, pRun2->y);
        if(ball.xMax - ball.xMin > maxWidth || ball.yMax - ball.yMin > maxWidth)
        {
          // a run that matches the color makes the bounding box too large
          return false;
        }
        visitedRuns[pRun2 - theRunLengthImage.image] = true; // mark run as visited
        const int runLength = pRun2->runLength;
        // update accumulators
        accY += temp.y * runLength;
        accCb += temp.cb * runLength;
        accCr += temp.cr * runLength;
        accNormCb += lutEntry.cb * runLength;
        accNormCr += lutEntry.cr * runLength;
        accSat += lutEntry.saturation * runLength;
        numOfPixels += runLength;
      }
      if(pRun2->xEnd < pRun->xEnd)
      {
        // there is another overlapping run in the next row
        ++pRun2;
      }
      else
      {
        // check if there are remaining runs in the region 
        if(++index >= ball.runs.size())
        {
          // there are no more matching runs, the region is finished preserving all constraints
          break;
        }
        pRun = ball.runs[index];
        pRun2 = pRun->firstOverlapBelow;
        if(pRun2 == &theRunLengthImage.voidRun)
        {
          // reached the end of image, the region is finished preserving all constraints
          break;
        }
      }
    }
  }

  ball.avgColor.y = accY / numOfPixels;
  ball.avgColor.cb = accCb / numOfPixels;
  ball.avgColor.cr = accCr / numOfPixels;

  potentialBalls.push_back(ball);
  return true;
}

bool BallPerceptorRLE2::buildCircleMMX(const RunLengthImage::Run* pRun, BoolArray& visitedRuns, bool searchUpAndDown)
{
  const RunLengthImage::Run* const pRunOrig = pRun;
  const RunLengthImage::Run* pRun2;

  PotentialBall ball;
  ball.runs.reserve(50); // avoid resizing
  const int y = pRun->y;
  ball.runs.push_back(pRun);
  ball.xMin = pRun->xStart;
  ball.xMax = pRun->xEnd;
  ball.yMin = ball.yMax = y;

  // a ball may be that wide for the current y coordinate
  // TODO compute correct length of a ball starting at that y-coordinate
  const float maxWidthForYCoord = getWidthInImage((float)y);
  const int maxWidth = (int)ceil(getWidthInImage((float)y + maxWidthForYCoord) + parameters.additionalWidth);

  visitedRuns[pRun - theRunLengthImage.image] = true; // mark run as visited
  // first search upwards
  ColorLutEntry lutEntry = colorLUT[pRun->color.cr * 256 + pRun->color.cb];
  lutEntry.padding = pRun->color.y;
  // initialize accumulators
  int numOfRuns = 1;

  MM_EMPTY();
  const __m64 mZero = _mm_setzero_si64();
  __m64 mAcc = _mm_unpacklo_pi8(_mm_cvtsi32_si64(lutEntry.dword), mZero);
  unsigned int accCb = pRun->color.cb, accCr = pRun->color.cr;

  unsigned int index = 0; // index of the currently processed run. An iterator cannot be used here because it could be invalidated in between
  if(searchUpAndDown)
  {
    pRun2 = pRun->firstOverlapAbove;
    if(pRun2 != &theRunLengthImage.voidRun)
    {
      for(;;)
      {
        if(!visitedRuns[pRun2 - theRunLengthImage.image])
        {
          Image::Pixel temp = pRun2->color;
          ColorLutEntry lutEntry = colorLUT[temp.cr * 256 + temp.cb];
          lutEntry.padding = temp.y;
          const __m64 mCount = _mm_shuffle_pi16(_mm_cvtsi32_si64(numOfRuns), 0);
          const __m64 mCurColor = _mm_unpacklo_pi8(_mm_cvtsi32_si64(lutEntry.dword), mZero);
          const __m64 mScaledColor = _mm_mullo_pi16(mCurColor, mCount);
          const __m64 mScaledThreshold = _mm_mullo_pi16(mThreshold, mCount);
          const __m64 absDifference = _mm_or_si64(_mm_subs_pu16(mAcc, mScaledColor), _mm_subs_pu16(mScaledColor, mAcc));
          const int mask = _mm_movemask_pi8(_mm_cmpgt_pi16(absDifference, mScaledThreshold));
          if(!mask)
          { // run is not visited and the color matches
            ball.runs.push_back(pRun2);
            ball.xMin = min(ball.xMin, pRun2->xStart);
            ball.xMax = max(ball.xMax, pRun2->xEnd);
            ball.yMin = min(ball.yMin, pRun2->y);
            if(ball.xMax - ball.xMin > maxWidth || ball.yMax - ball.yMin > maxWidth)
            {
              // a run that matches the color makes the bounding box too large, hence no ball
              MM_EMPTY();
              return false;
            }
            visitedRuns[pRun2 - theRunLengthImage.image] = true; // mark run as visited
            // update accumulators
            accCb += temp.cb;
            accCr += temp.cr;
            mAcc = _mm_add_pi16(mAcc, mCurColor);
            ++numOfRuns;
          }
        }
        if(pRun2->xEnd < pRun->xEnd)
        {
          // there is another overlapping run in the previous row
          ++pRun2;
        }
        else
        {
          // check if there are remaining runs in the region
          if(++index >= ball.runs.size())
          {
            // there are no more matching runs, the region is finished preserving all constraints
            break;
          }
          pRun = ball.runs[index];
          pRun2 = pRun->firstOverlapAbove;
          if(pRun2 == &theRunLengthImage.voidRun)
          {
            // reached the end of image, the region is finished preserving all constraints
            break;
          }
        }
      }
    }

    // to keep the ascending order of runs, the runs found so far have to be reversed row-wise
    vector<const RunLengthImage::Run*> temp(ball.runs.size());
    vector<const RunLengthImage::Run*>::iterator i = ball.runs.end();
    while(i > ball.runs.begin())
    {
      int y = (*(i-1))->y;
      const vector<const RunLengthImage::Run*>::iterator iOld = i;
      while(i > ball.runs.begin() && (*(i - 1))->y == y)
      {
        --i;
      }
      copy(i, iOld, temp.begin() + (ball.runs.end() - iOld));
    }
    ball.runs.swap(temp);

    // now search downwards
    index = ball.runs.size() - 1;
  }

  pRun = pRunOrig;
  pRun2 = pRun->firstOverlapBelow;
  if(pRun2 != &theRunLengthImage.voidRun)
  {
    for(;;)
    {
      if(!visitedRuns[pRun2 - theRunLengthImage.image])
      {
        Image::Pixel temp = pRun2->color;
        ColorLutEntry lutEntry = colorLUT[temp.cr * 256 + temp.cb];
        lutEntry.padding = temp.y;
        const __m64 mCount = _mm_shuffle_pi16(_mm_cvtsi32_si64(numOfRuns), 0);
        const __m64 mCurColor = _mm_unpacklo_pi8(_mm_cvtsi32_si64(lutEntry.dword), mZero);
        const __m64 mScaledColor = _mm_mullo_pi16(mCurColor, mCount);
        const __m64 mScaledThreshold = _mm_mullo_pi16(mThreshold, mCount);
        const __m64 absDifference = _mm_or_si64(_mm_subs_pu16(mAcc, mScaledColor), _mm_subs_pu16(mScaledColor, mAcc));
        const int mask = _mm_movemask_pi8(_mm_cmpgt_pi16(absDifference, mScaledThreshold));
        if(!mask)
        { // run is not visited and the color matches
          ball.runs.push_back(pRun2);
          ball.xMin = min(ball.xMin, pRun2->xStart);
          ball.xMax = max(ball.xMax, pRun2->xEnd);
          ball.yMax = max(ball.yMin, pRun2->y);
          if(ball.xMax - ball.xMin > maxWidth || ball.yMax - ball.yMin > maxWidth)
          {
            // a run that matches the color makes the bounding box too large, hence no ball
            MM_EMPTY();
            return false;
          }
          visitedRuns[pRun2 - theRunLengthImage.image] = true; // mark run as visited
          // update accumulators
          accCb += temp.cb;
          accCr += temp.cr;
          mAcc = _mm_add_pi16(mAcc, mCurColor);
          ++numOfRuns;
        }
      }
      if(pRun2->xEnd < pRun->xEnd)
      {
        // there is another overlapping run in the next row
        ++pRun2;
      }
      else
      {
        // check if there are remaining runs in the region 
        if(++index >= ball.runs.size())
        {
          // there are no more matching runs, the region is finished preserving all constraints
          break;
        }
        pRun = ball.runs[index];
        pRun2 = pRun->firstOverlapBelow;
        if(pRun2 == &theRunLengthImage.voidRun)
        {
          // reached the end of image, the region is finished preserving all constraints
          break;
        }
      }
    }
  }

  mAcc = _mm_insert_pi16(mAcc, accCb, offsetof(ColorLutEntry, cb));
  mAcc = _mm_insert_pi16(mAcc, accCr, offsetof(ColorLutEntry, cr));
  const __m64 mDivisor = divisionLUT[numOfRuns - 1];
  ColorLutEntry avgColor;
  avgColor.dword = _mm_cvtsi64_si32(_mm_packs_pu16(_mm_mulhi_pu16(mAcc, mDivisor), mAcc));
  ball.avgColor.y = avgColor.padding;
  ball.avgColor.cb = avgColor.cb;
  ball.avgColor.cr = avgColor.cr;
  MM_EMPTY();

  potentialBalls.push_back(ball);
  return true;
}

void BallPerceptorRLE2::extendCircle(PotentialBall& potentialBall) const
{
  ASSERT(potentialBall.runs.size() > 0);
  const int numOfRuns = potentialBall.runs.size();
  const RunLengthImage::Run* pRun = potentialBall.runs.front();
  ColorLutEntry avgLutEntry = colorLUT[potentialBall.avgColor.cr * 256 + potentialBall.avgColor.cb];

  // extend first run to the left
  while(pRun->xStart > 0)
  {
    --pRun;
    ColorLutEntry lutEntry = colorLUT[pRun->color.cr * 256 + pRun->color.cb];
    if(abs(pRun->y - potentialBall.avgColor.y) < parameters.thresholdY && abs(avgLutEntry.cb - lutEntry.cb) < parameters.thresholdCb && abs(avgLutEntry.cr - lutEntry.cr) < parameters.thresholdCr && abs(avgLutEntry.saturation - lutEntry.saturation) < parameters.thresholdSaturation)
    { // color of neighboring run matches average color of region
      potentialBall.runs.push_back(pRun);
    }
    else
    {
      break;
    }
  }

  // process remaining runs
  for(int i = 1; i < numOfRuns; ++i)
  {
    if(potentialBall.runs[i]->y != potentialBall.runs[i - 1]->y) // only process runs at the border of the region
    { // the runs are in different rows, so the i-1th is at the right border and the ith is at the left border
      // extend i-1th run to the right
      pRun = potentialBall.runs[i - 1];
      while(pRun->xEnd < cameraResolutionWidth - 1)
      {
        ++pRun;
        ColorLutEntry lutEntry = colorLUT[pRun->color.cr * 256 + pRun->color.cb];
        if(abs(pRun->y - potentialBall.avgColor.y) < parameters.thresholdY && abs(avgLutEntry.cb - lutEntry.cb) < parameters.thresholdCb && abs(avgLutEntry.cr - lutEntry.cr) < parameters.thresholdCr && abs(avgLutEntry.saturation - lutEntry.saturation) < parameters.thresholdSaturation)
        {
          potentialBall.runs.push_back(pRun);
        }
        else
        {
          break;
        }
      }

      // extend the ith run to the left
      pRun = potentialBall.runs[i];
      while(pRun->xStart > 0)
      {
        --pRun;
        ColorLutEntry lutEntry = colorLUT[pRun->color.cr * 256 + pRun->color.cb];
        if(abs(pRun->y - potentialBall.avgColor.y) < parameters.thresholdY && abs(avgLutEntry.cb - lutEntry.cb) < parameters.thresholdCb && abs(avgLutEntry.cr - lutEntry.cr) < parameters.thresholdCr && abs(avgLutEntry.saturation - lutEntry.saturation) < parameters.thresholdSaturation)
        {
          potentialBall.runs.push_back(pRun);
        }
        else
        {
          break;
        }
      }
    }
  }

  // extend last run to the right
  pRun = potentialBall.runs.back();
  while(pRun->xEnd < cameraResolutionWidth - 1)
  {
    ++pRun;
    ColorLutEntry lutEntry = colorLUT[pRun->color.cr * 256 + pRun->color.cb];
    if(abs(pRun->y - potentialBall.avgColor.y) < parameters.thresholdY && abs(avgLutEntry.cb - lutEntry.cb) < parameters.thresholdCb && abs(avgLutEntry.cr - lutEntry.cr) < parameters.thresholdCr && abs(avgLutEntry.saturation - lutEntry.saturation) < parameters.thresholdSaturation)
    {
      potentialBall.runs.push_back(pRun);
    }
    else
    {
      break;
    }
  }
}

bool BallPerceptorRLE2::getCenterAndRadius(PotentialBall& potentialBall) const
{
  // extract potential border points from runs
  vector<BallPoint> points;
  for(vector<const RunLengthImage::Run*>::const_iterator i = potentialBall.runs.begin(); i != potentialBall.runs.end(); ++i)
  {
    // check if the start and end point of the run is a corner of the image, if not they are potential border points
    const bool upperOrLowerBorder = (*i)->y == 0 || (*i)->y == theImageGrid.lastYCoord;
    points.push_back(BallPoint(Vector2<int>((*i)->xStart, (*i)->y), upperOrLowerBorder && (*i)->xStart == 0));
    points.push_back(BallPoint(Vector2<int>((*i)->xEnd, (*i)->y), upperOrLowerBorder && (*i)->xEnd == cameraResolutionWidth - 1));
  }
  // build convex hull from the border points
  // currently the slightly slower jarvis march algorithm is used because graham scan currently cannot deal with duplicate points
  ConvexHull::jarvisMarch(points);

  // this computation is completely taken from the current implementation of BallPerceptor, which is actually taken from GT2005
  // a linear equation system is solved to compute center and radius
  float Mx = 0, My = 0, Mxx = 0, Myy = 0, Mxy = 0, Mz = 0, Mxz = 0, Myz = 0;
  int numOfPoints = 0;

  for(vector<BallPoint>::const_iterator i = points.begin(); i != points.end(); ++i)
  {
    if(i->isCornerPoint)
    {
      continue;
    }
    const float x = (float)i->x;
    const float y = (float)i->y;
    const float xx = x*x;
    const float yy = y*y;
    const float z = xx + yy;
    Mx += x;
    My += y;
    Mxx += xx;
    Myy += yy;
    Mxy += x*y;
    Mz += z;
    Mxz += x*z;
    Myz += y*z;
    ++numOfPoints;
  }

  // Construct and solve matrix (might fail).
  // Result will be center and radius of ball in theImage.
  Matrix<3, 3> M(
    Vector<3>(Mxx, Mxy, Mx),
    Vector<3>(Mxy, Myy, My),
    Vector<3>(Mx, My,  static_cast<float>(numOfPoints)));

  Vector<3> v(-Mxz, -Myz, -Mz);

  Vector<3> BCD;

  if(!M.solve(v, BCD))
  {
    return false;
  }

  potentialBall.centerInImage.x = BCD[0] * -0.5f;
  potentialBall.centerInImage.y = BCD[1] * -0.5f;
  potentialBall.centerInImage = potentialBall.centerInImage;
  float radicand = BCD[0]*BCD[0]/4.0f + BCD[1]*BCD[1]/4.0f - BCD[2];
  if (radicand <= 0.0f)
    return false;
  potentialBall.radius = sqrt(radicand);

  // compute error (the sum of distances of the single points to the circle)
  float error = 0;
  for(vector<BallPoint>::const_iterator i = points.begin(); i != points.end(); ++i)
  {
    if(i->isCornerPoint)
    {
      continue;
    }
    error += abs((Vector2<>((float)i->x, (float)i->y) - potentialBall.centerInImage).abs() - potentialBall.radius);
  }
  error /= numOfPoints;
  if(error > parameters.roundnessThreshold)
  {
    return false;
  }

  return true;
}

bool BallPerceptorRLE2::getCenterAndRadiusRANSAC(PotentialBall& potentialBall) const
{
  // extract potential border points from runs
  vector< Vector2<int> > points;
  for(vector<const RunLengthImage::Run*>::const_iterator i = potentialBall.runs.begin(); i != potentialBall.runs.end(); ++i)
  {
    // check if the start and end point of the run is a corner of the image, if not they are potential border points
    points.push_back(Vector2<int>((*i)->xStart, (*i)->y));
    points.push_back(Vector2<int>((*i)->xEnd, (*i)->y));
  }
  // build convex hull from the border points
  // currently the slightly slower jarvis march algorithm is used because graham scan currently cannot deal with duplicate points
  //ConvexHull::jarvisMarch(points);

  // find relevant points using RANSAC
  // ensure determinism
  srand(theFrameInfo.time);
  const int numPoints = points.size();
  vector<const Vector2<int>*> bestConsensus, temp;
  bestConsensus.reserve(numPoints);
  temp.reserve(numPoints);
  for(unsigned int i = 0; i < parameters.iterationsRANSAC; ++i)
  {
    int index1 = rand() % numPoints;
    int index2 = rand() % (numPoints - 1);
    if(index2 == index1) ++index2;
    int index3 = rand() % (numPoints - 2);
    if(index3 == index1 || index3 == index2) ++index3;
    if(index3 == index2 || index3 == index1) ++index3;
    ASSERT(index1 != index2 && index1 != index3 && index2 != index3);
    Geometry::Circle circle = Geometry::getCircle(points[index1], points[index2], points[index3]);
    for(vector< Vector2<int> >::const_iterator j = points.begin(); j != points.end(); ++j)
    {
      if(sqrt(sqr((float)j->x - circle.center.x) + sqr((float)j->y - circle.center.y)) - circle.radius < parameters.thresholdRANSAC)
      {
        temp.push_back(&*j);
      }
    }
    if(temp.size() > bestConsensus.size())
    {
      bestConsensus.swap(temp);
    }
    temp.clear();
  }



  // this computation is completely taken from the current implementation of BallPerceptor, which is actually taken from GT2005
  // a linear equation system is solved to compute center and radius
  float Mx = 0, My = 0, Mxx = 0, Myy = 0, Mxy = 0, Mz = 0, Mxz = 0, Myz = 0;
  int numOfPoints = 0;

  for(vector<const Vector2<int>*>::const_iterator i = bestConsensus.begin(); i != bestConsensus.end(); ++i)
  {
    const float x = (float)(**i).x;
    const float y = (float)(**i).y;
    const float xx = x*x;
    const float yy = y*y;
    const float z = xx + yy;
    Mx += x;
    My += y;
    Mxx += xx;
    Myy += yy;
    Mxy += x*y;
    Mz += z;
    Mxz += x*z;
    Myz += y*z;
    ++numOfPoints;
  }

  // Construct and solve matrix (might fail).
  // Result will be center and radius of ball in theImage.
  Matrix<3, 3> M(
    Vector<3>(Mxx, Mxy, Mx),
    Vector<3>(Mxy, Myy, My),
    Vector<3>(Mx, My,  static_cast<float>(numOfPoints)));

  Vector<3> v(-Mxz, -Myz, -Mz);

  Vector<3> BCD;

  if(!M.solve(v, BCD))
  {
    return false;
  }

  potentialBall.centerInImage.x = BCD[0] * -0.5f;
  potentialBall.centerInImage.y = BCD[1] * -0.5f;
  potentialBall.centerInImage = potentialBall.centerInImage;
  float radicand = BCD[0]*BCD[0]/4.0f + BCD[1]*BCD[1]/4.0f - BCD[2];
  if (radicand <= 0.0f)
    return false;
  potentialBall.radius = sqrt(radicand);

  // compute error (the sum of distances of the single points to the circle)
  float error = 0;
  for(vector<const Vector2<int>*>::const_iterator i = bestConsensus.begin(); i != bestConsensus.end(); ++i)
  {
    error += abs((Vector2<>((float)(**i).x, (float)(**i).y) - potentialBall.centerInImage).abs() - potentialBall.radius);
  }
  error /= numOfPoints;
  if(error > parameters.roundnessThreshold)
  {
    return false;
  }

  return true;
}

bool BallPerceptorRLE2::getRelativePosition(PotentialBall& potentialBall) const
{
  const Vector2<> corrected = theImageCoordinateSystem.toCorrected(potentialBall.centerInImage);
  const float actualCameraZ = theCameraMatrix.translation.z - (float)theFieldDimensions.ballRadius;
  // vector from the center of the camera to the found ball in the image plane
  Vector3<> camera2Ball(theCameraInfo.focalLength, theCameraInfo.opticalCenter.x - corrected.x, theCameraInfo.opticalCenter.y - corrected.y);
  Vector3<> camera2BallRotated(theCameraMatrix.rotation * camera2Ball);
  // check if center is above horizon
  if(camera2BallRotated.z > 0)
  {
    return false;
  }
  // compute intersection of the camera-ball-ray with the ball plane (parallel to the ground)
  camera2BallRotated *= actualCameraZ / -camera2BallRotated.z;
  const Vector3<> bearingBasedPositionOnField(theCameraMatrix.translation + camera2BallRotated);
  potentialBall.relPosOnField = Vector2<>(bearingBasedPositionOnField.x, bearingBasedPositionOnField.y);

  return true;
}

bool BallPerceptorRLE2::checkBackProjection(PotentialBall& potentialBall) const
{
  // in the back projection the motion distortion is not taken into account
  // TODO take (at least linearized) motion distortion into account
  const Vector3<> ballInWorld(potentialBall.relPosOnField.x, potentialBall.relPosOnField.y, (float)theFieldDimensions.ballRadius);
  Vector3<> ballInCamera = ballInWorld - theCameraMatrix.translation;
  ballInCamera = theCameraMatrix.rotation.invert() * ballInCamera;

  // compute ellipse in image using the concept of quadrics and conics
  Vector3<> tmp(-ballInCamera.y, -ballInCamera.z, ballInCamera.x);
  Matrix<3, 1> C(Vector<3>(tmp.x * theCameraInfo.focalLengthInv, tmp.y * theCameraInfo.focalLengthInv, tmp.z)); // correct for different coordinate systems in world and in camera
  Matrix<3, 3> Q = P2 - C * (1.0f / (tmp.squareAbs() - sqr((float)theFieldDimensions.ballRadius))) * C.transpose(); // actual projection (for the derivation see diploma thesis)
  // Q is now the matrix representation of the projected conic


  const float det33 = Q[0][0] * Q[1][1] - sqr(Q[0][1]);
  const float detQ = Q.det();

  float radiusInImage;

  if(det33 > 0 && Q[1][1] * detQ < 0) // check if it is an ellipse
  {
    const float temp1 = Q[0][0] + Q[1][1];
    const float temp2 = sqrt(sqr(Q[0][0] - Q[1][1]) + 4.0f * sqr(Q[0][1]));
    const float lambda1 = (temp1 + temp2) * 0.5f;
    const float lambda2 = (temp1 - temp2) * 0.5f;
    const float axis1 = sqrt(-detQ / (det33 * lambda1));
    const float axis2 = sqrt(-detQ / (det33 * lambda2));
    radiusInImage = (axis1 + axis2) * 0.5f; // radius is assumed to be the average of the axes of the ellipse
  }
  else // not an ellipse, something's wrong
  {
    return false;
  }

  const float radiusBasedDistanceFromCamera = theCameraInfo.focalLength / potentialBall.radius * (float)theFieldDimensions.ballRadius;
  if(radiusBasedDistanceFromCamera > cameraHeight)
    potentialBall.radiusBasedDistance = sqrt(sqr(radiusBasedDistanceFromCamera) - sqr(cameraHeight));
  else
    potentialBall.radiusBasedDistance = 0;

  if(abs(radiusInImage - (potentialBall.radius + parameters.radiusOffset)) > parameters.thresholdRadius)
  {
    return false;
  }
  return true;
}

void BallPerceptorRLE2::draw() const
{
  DECLARE_DEBUG_DRAWING("module:BallPerceptorRLE2:boundingBoxes", "drawingOnImage");
  COMPLEX_DRAWING("module:BallPerceptorRLE2:boundingBoxes",
    for(list<PotentialBall>::const_iterator i = potentialBalls.begin(); i != potentialBalls.end(); ++i)
    {
      RECTANGLE("module:BallPerceptorRLE2:boundingBoxes", i->xMin, i->yMin, i->xMax + 1, i->yMax + 1, 1, Drawings::ps_solid, ColorRGBA(ColorClasses::yellow));
    }
  );

  DECLARE_DEBUG_DRAWING("module:BallPerceptorRLE2:convexHulls", "drawingOnImage");
  COMPLEX_DRAWING("module:BallPerceptorRLE2:convexHulls",
    for(list<PotentialBall>::const_iterator i = potentialBalls.begin(); i != potentialBalls.end(); ++i)
    {
      vector< Vector2<int> > points;
      for(vector<const RunLengthImage::Run*>::const_iterator j = i->runs.begin(); j != i->runs.end(); ++j)
      {
        points.push_back(Vector2<int>((*j)->xStart, (*j)->y));
        points.push_back(Vector2<int>((*j)->xEnd, (*j)->y));
      }
      ConvexHull::jarvisMarch(points);
      for(vector< Vector2<int> >::iterator j = points.begin() + 1; j != points.end(); ++j)
      {
        LINE("module:BallPerceptorRLE2:convexHulls", j->x, j->y, (j - 1)->x, (j - 1)->y, 1, Drawings::ps_solid, ColorRGBA(ColorClasses::yellow));
      }
      LINE("module:BallPerceptorRLE2:convexHulls", points.back().x, points.back().y, points.front().x, points.front().y, 1, Drawings::ps_solid, ColorRGBA(ColorClasses::yellow));
    }
  );

  DECLARE_DEBUG_DRAWING("module:BallPerceptorRLE2:balls", "drawingOnImage");
  COMPLEX_DRAWING("module:BallPerceptorRLE2:balls",
    ColorRGBA backgroundColor(ColorClasses::yellow);
    backgroundColor.a = 100;
    for(list<PotentialBall>::const_iterator i = potentialBalls.begin(); i != potentialBalls.end(); ++i)
    {
      CIRCLE("module:BallPerceptorRLE2:balls", i->centerInImage.x, i->centerInImage.y, i->radius, 1, Drawings::ps_solid, ColorClasses::yellow, Drawings::bs_solid, backgroundColor);
    }
  );

  DECLARE_DEBUG_DRAWING("module:BallPerceptorRLE2:radius", "drawingOnImage");
  COMPLEX_DRAWING("module:BallPerceptorRLE2:radius",
    for(list<PotentialBall>::const_iterator i = potentialBalls.begin(); i != potentialBalls.end(); ++i)
    {
      DRAWTEXT("module:BallPerceptorRLE2:radius", i->centerInImage.x, i->centerInImage.y, 6, ColorClasses::yellow, i->radius);
    }

  );
}

void BallPerceptorRLE2::drawRatingBallRuns(const ExtendedRun* const runs, const RunLengthImage::Run* startRun, const int numberOfRuns) const
{
  COMPLEX_DEBUG_IMAGE(ratingBallRuns,
    
    INIT_DEBUG_IMAGE_BLACK(ratingBallRuns);
    const ExtendedRun* const pEnd = runs + numberOfRuns;

    for(const ExtendedRun* p = runs; p != pEnd; ++p)
    {
      const int grayValue = max(0, 255 - p->colorDistance);
      const RunLengthImage::Run* const run = startRun + p->runIndex;
      const int yStart = run->y;
      const int yEnd = yStart + theImageGrid.lineInformationLUT[yStart].rowInterval;
      for(int y = yStart; y < yEnd; ++y)
      {
        for(int x = run->xStart; x <= run->xEnd; ++x)
        {
          DEBUG_IMAGE_SET_PIXEL_YUV(ratingBallRuns, x, y, grayValue, 127, 127);
        }
      }
    }
    SEND_DEBUG_IMAGE(ratingBallRuns);
  );
}

void BallPerceptorRLE2::drawnormalizedImages() const
{
  COMPLEX_DEBUG_IMAGE(normalizedYColors,
    for(int y = 0; y < cameraResolutionHeight; ++y)
    {
      for(int x = 0; x < cameraResolutionWidth; ++x)
      {
        const Image::Pixel pixel = theImage.image[y][x];
        DEBUG_IMAGE_SET_PIXEL_YUV(normalizedYColors, x, y, 127, pixel.cb, pixel.cr);
      }
    }
    SEND_DEBUG_IMAGE(normalizedYColors);
  );

  COMPLEX_DEBUG_IMAGE(normalizedColors,
    for(int y = 0; y < cameraResolutionHeight; ++y)
    {
      for(int x = 0; x < cameraResolutionWidth; ++x)
      {
        const Image::Pixel pixel = theImage.image[y][x];
        const ColorLutEntry& entry = colorLUT[pixel.cr * 256 + pixel.cb];
        DEBUG_IMAGE_SET_PIXEL_YUV(normalizedColors, x, y, 127, entry.cb, entry.cr);
      }
    }
    SEND_DEBUG_IMAGE(normalizedColors);
  );

  unsigned char sat = 127;
  MODIFY("module:BallPerceptorRLE2:drawingSaturation", sat);
  COMPLEX_DEBUG_IMAGE(normalizedHue,
    for(int y = 0; y < cameraResolutionHeight; ++y)
    {
      for(int x = 0; x < cameraResolutionWidth; ++x)
      {
        Image::Pixel pixel = theImage.image[y][x];
        Image::Pixel hsiPixel;
        ColorModelConversions::fromYCbCrToHSI(pixel.y, pixel.cb, pixel.cr, hsiPixel.h, hsiPixel.s, hsiPixel.i);
        ColorModelConversions::fromHSIToYCbCr(hsiPixel.h, sat, 127, pixel.y, pixel.cb, pixel.cr);
        DEBUG_IMAGE_SET_PIXEL_YUV(normalizedHue, x, y, pixel.y, pixel.cb, pixel.cr);
      }
    }
    SEND_DEBUG_IMAGE(normalizedHue);
  );

  COMPLEX_DEBUG_IMAGE(saturation,
    const float factor = 2.0f / sqrt(2.0f);
    for(int y = 0; y < cameraResolutionHeight; ++y)
    {
      for(int x = 0; x < cameraResolutionWidth; ++x)
      {
        const Image::Pixel pixel = theImage.image[y][x];
        const ColorLutEntry& entry = colorLUT[pixel.cr * 256 + pixel.cb];
        DEBUG_IMAGE_SET_PIXEL_YUV(saturation, x, y, (unsigned char)(entry.saturation * factor), 127, 127);
      }
    }
    SEND_DEBUG_IMAGE(saturation);
  );

  COMPLEX_DEBUG_IMAGE(brightness,
    for(int y = 0; y < cameraResolutionHeight; ++y)
    {
      for(int x = 0; x < cameraResolutionWidth; ++x)
      {
        const Image::Pixel pixel = theImage.image[y][x];
        DEBUG_IMAGE_SET_PIXEL_YUV(brightness, x, y, pixel.y, 127, 127);
      }
    }
    SEND_DEBUG_IMAGE(brightness);
  );

  COMPLEX_DEBUG_IMAGE(ballRegions,
    INIT_DEBUG_IMAGE_BLACK(ballRegions);
    for(list<PotentialBall>::const_iterator i = potentialBalls.begin(); i != potentialBalls.end(); ++i)
    {
      for(vector<const RunLengthImage::Run*>::const_iterator j = i->runs.begin(); j != i->runs.end(); ++j)
      {
        const int yStart = (**j).y;
        const int yEnd = yStart + theImageGrid.lineInformationLUT[yStart].rowInterval;
        for(int y = yStart; y < yEnd; ++y)
        {
          for(int x = (**j).xStart; x <= (**j).xEnd; ++x)
          {
            DEBUG_IMAGE_SET_PIXEL_YUV(ballRegions, x, y, i->avgColor.y, i->avgColor.cb, i->avgColor.cr);
          }
        }
      }
    }
    SEND_DEBUG_IMAGE(ballRegions);
  );
}