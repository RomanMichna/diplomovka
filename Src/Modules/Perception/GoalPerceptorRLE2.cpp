/**
* @file GoalPerceptorRLE2.cpp
* This file implements a module that provides a goal percept only using the RunLengthImage.
* @author Alexander Härtl
*/

#include "GoalPerceptorRLE2.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Geometry.h"
#include "Tools/ImageProcessing/ColorModelConversions.h"
#include "Modules/Perception/RunLengthImageProvider.h"
#include <algorithm>

#define SCALE_B 8
#define SCALE_F 16

MAKE_MODULE(GoalPerceptorRLE2, Perception)

void GoalPerceptorRLE2::init()
{
  RunLengthImageProvider::createLutUsingC(divisionLUT, cameraResolutionHeight);
}

void GoalPerceptorRLE2::preExecution()
{
  MODIFY("module:GoalPerceptorRLE2:parameters", parameters);

  Image::Pixel threshold;
  threshold.y = parameters.thresholdY;
  threshold.cb = parameters.thresholdCb;
  threshold.cr = parameters.thresholdCr;
  threshold.yCbCrPadding = 255; // padding value is ignored, so threshold is set to maximum value

  // cast packed bytes to packed shorts
  mThreshold = _mm_unpacklo_pi8(_mm_cvtsi32_si64(threshold.color), _mm_setzero_si64());
  MM_EMPTY();
  DECLARE_DEBUG_DRAWING("module:GoalPerceptorRLE2:averagedHoughSpace", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:GoalPerceptorRLE2:goalRegions", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:GoalPerceptorRLE2:houghMaxima", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:GoalPerceptorRLE2:houghSpace", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:GoalPerceptorRLE2:vanishingPoint", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:GoalPerceptorRLE2:buckets", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:GoalPerceptorRLE2:houghSpace2", "drawingOnImage");
}

void GoalPerceptorRLE2::update(GoalPercept& goalPercept)
{
  preExecution();

  // clear goal percept
  goalPercept.goalPosts.clear();

  // only process the image if the camera matrix is valid
  if(!theCameraMatrix.isValid)
    return;

  Vector2<> vanishingPoint = computeVanishingPoint(2);

  // "uncorrect" vanishing point with linearized motion distortion model
  vanishingPoint = theImageCoordinateSystem.fromCorrectedLinearized(vanishingPoint);

  // if the cameras z-axis is almost aligned with the worlds z-axis, the vanishing point
  // is very far outside the image. To ensure the integer arithmetics below don't get
  // out of range, it is clipped to a reasonable value, which has no large influence on
  // the single rays through the point.
  const float vanishingPointProductAbs = abs(vanishingPoint.x * vanishingPoint.y);
  const float vanishingPointProductLimit = (float)(1 << (30 - SCALE_B));
  if(vanishingPointProductAbs > vanishingPointProductLimit)
  {
    const float scale = sqrt(vanishingPointProductLimit / vanishingPointProductAbs);
    vanishingPoint *= scale;
  }

  const int averagingWidth = parameters.houghSpaceConvolutionKernel.size();
  // reserve some additional space before and after the actual hough space to simplify and speed up some computations
  unsigned int rawHoughWidth = parameters.houghWidth + 2 * averagingWidth;

  // the rare case that the vanishing point is at the top of the image must be considered
  const bool houghSpaceStartAtLowerBorder = vanishingPoint.y > cameraResolutionHeight / 2;

  // yStart and yEnd mark the actual area of the image that can be processed into the hough space
  const int yStart = houghSpaceStartAtLowerBorder ? 0 : max((int)vanishingPoint.y + parameters.houghSpaceBeginAboveVanishingPoint, 0);
  const int yEnd = houghSpaceStartAtLowerBorder ? min((int)vanishingPoint.y - parameters.houghSpaceBeginAboveVanishingPoint, cameraResolutionHeight - 1) : cameraResolutionHeight - 1;

  float xStart = 0.0f, xEnd = float(cameraResolutionWidth);

  // consider that if the x-coordinate of the vanishing point is outside the image borders, 
  // the hough space has to be streched so that the image area opposite to the vanishing point
  // is covered by the hough space
  if(vanishingPoint.x < 0.0f)
  {
    if(houghSpaceStartAtLowerBorder)
    {
      xStart = vanishingPoint.x * (float(yEnd) - vanishingPoint.y) / (float(yStart) - vanishingPoint.y);
    }
    else
    {
      xStart = vanishingPoint.x * (float(yStart) - vanishingPoint.y) / (float(yEnd) - vanishingPoint.y);
    }
  }
  else if(vanishingPoint.x >= float(cameraResolutionWidth))
  {
    if(houghSpaceStartAtLowerBorder)
    {
      xEnd = vanishingPoint.x - (vanishingPoint.x - float(cameraResolutionWidth)) * (float(yEnd) - vanishingPoint.y) / (float(yStart) - vanishingPoint.y);
    }
    else
    {
      xEnd = vanishingPoint.x - (vanishingPoint.x - float(cameraResolutionWidth)) * (float(yStart) - vanishingPoint.y) / (float(yEnd) - vanishingPoint.y);
    }
  }

  // the width of one hough accumulator in image coordinates (at the upper or lower border of the image lying opposite to the vanishing point)
  const float widthPerHoughPixel = (xEnd - xStart) / (float)parameters.houghWidth;
  const float houghPixelPerPixel = 1.0f / widthPerHoughPixel;

  // the reference y-coordinate to compute the corresponding hough accumulator
  const int houghRefY = houghSpaceStartAtLowerBorder ? yEnd : yStart;

  // compute values necessary for getHoughLine()
  houghLineSlopeSlope = widthPerHoughPixel / (houghRefY - vanishingPoint.y);
  houghLineSlopeIntercept = (0.5f * widthPerHoughPixel + xStart - vanishingPoint.x) / (houghRefY - vanishingPoint.y);
  houghLineXStartSlope = widthPerHoughPixel * (1.0f - houghRefY / (houghRefY - vanishingPoint.y));
  houghLineXStartIntercept = 0.5f * widthPerHoughPixel + xStart - (0.5f * widthPerHoughPixel + xStart - vanishingPoint.x) * houghRefY / (houghRefY - vanishingPoint.y);

  // precompute values for filling hough space
  const float temp1 = (houghRefY - vanishingPoint.y) * houghPixelPerPixel;
  const float tempScaleB = (float)(1 << SCALE_B);
  const float tempScaleF = (float)(1 << SCALE_F);
  const float temp2 = (temp1 * vanishingPoint.x) * tempScaleB;
  const float temp3 = ((vanishingPoint.x - xStart) * houghPixelPerPixel) * tempScaleB;
  const float temp4 = (temp1 * -0.5f) * tempScaleF;

  // if the values are out of range, abort
  if(abs(temp2) > 2147483647.0f || abs(temp3) > 2147483647.0f || abs(temp4) > 2147483647.0f)
  {
    ASSERT(false);
    return;
  }

  // cast precomputed values to int
  const int precomp1 = (int)temp2;
  const int precomp2 = (int)temp3;
  const int precomp3 = (int)temp4;
  const int precomp4 = (int)(vanishingPoint.y + 0.5f);

  const RunLengthImage::Run* pRun = theRunLengthImage.image;

  // move pointer to start coordinate
  while(pRun->y < yStart)
  {
    pRun = pRun->firstOverlapBelow;
  }
  ASSERT(pRun->xStart == 0);

  const RunLengthImage::Run* pEnd = theRunLengthImage.beginOfLastLine;
  // move end pointer to end coordinate
  // somewhat more complicated than start pointer
  while(pEnd->y > yEnd)
  {
    pEnd = pEnd->firstOverlapAbove;
  }
  pEnd = pEnd->firstOverlapBelow;
  if(pEnd == &theRunLengthImage.voidRun)
  {
    pEnd = theRunLengthImage.image + theRunLengthImage.numberOfRuns;
  }

  // allocate hough space
  unsigned int* rawHoughSpace = new unsigned int[rawHoughWidth];
  // houghSpace is used as the actual array for the hough space with a "safety buffer" at its borders
  unsigned int* houghSpace = rawHoughSpace + averagingWidth;

  STOP_TIME_ON_REQUEST_WITH_PLOT("module:GoalPerceptorRLE2:fillHoughSpace",
    // initialize the hough space
    fill(rawHoughSpace, rawHoughSpace + rawHoughWidth, 0);

    MM_EMPTY();
    const Image::Pixel yellow = theColorConfiguration.colors[ColorClasses::yellow].color.convertToPixel();
    const __m64 mYellow = _mm_cvtsi32_si64(yellow.color);
    const __m64 mHoughBase = _mm_cvtsi32_si64(parameters.houghSpaceBaseValue - parameters.houghSpaceMinValue);
    
    const RunLengthImage::Run* p = pRun;

    while(p < pEnd)
    {
      // process one line after the other
      int currentY = p->y;

      // compute values only depending on the y-coordinate
      const int y2 = precomp4 - currentY;
      const int bScaled = precomp1 / y2 + precomp2;
      const int fScaled = precomp3 / y2;

      const unsigned short rowInterval = theImageGrid.lineInformationLUT[currentY].rowInterval;
      while(p->y == currentY)
      {
        // compute hough index from precomputed values
        const int centerOfRun2 = p->xStart + p->xEnd;
        const int houghIndex = (bScaled + ((centerOfRun2 * fScaled) >> (SCALE_F - SCALE_B))) >> SCALE_B;

        // compute color difference using MMX-intrinsics, scale it and subtract it from the hough base value
        // the used color difference is the color distance to yellow
        const __m64 mColor = _mm_cvtsi32_si64(p->color.color);
        const __m64 mColorDifference = _mm_sad_pu8(mColor, mYellow);
        const int scaledColorDifference = _mm_cvtsi64_si32(_mm_subs_pu16(mHoughBase, mColorDifference));

        ASSERT(houghIndex >= -averagingWidth && houghIndex < parameters.houghWidth + averagingWidth);
        // increase corresponding hough accumulator
        houghSpace[houghIndex] += (scaledColorDifference + parameters.houghSpaceMinValue) * rowInterval;
        ++p;
      }
    }
    MM_EMPTY();
  );

  bool useAveragedHoughSpace = false;
  bool copyHoughSpaceForDrawing = false;
  DEBUG_RESPONSE("debug images:houghSpace", copyHoughSpaceForDrawing = true;);
  DEBUG_RESPONSE("debug drawing:module:GoalPerceptorRLE2:houghSpace2", copyHoughSpaceForDrawing = true;);
  MODIFY("module:GoalPerceptorRLE2:useAveragedHoughSpaceForDrawings", useAveragedHoughSpace);
  vector<int> houghSpaceForDrawing;
  if(copyHoughSpaceForDrawing && !useAveragedHoughSpace)
  {
    houghSpaceForDrawing.insert(houghSpaceForDrawing.begin(), houghSpace, houghSpace + parameters.houghWidth);
  }

  // the averaged hough space has a "safety border" as well
  unsigned int* rawAveragedHoughSpace = new unsigned int[parameters.houghWidth + 2 * averagingWidth];
  unsigned int* averagedHoughSpace = rawAveragedHoughSpace + averagingWidth;
  STOP_TIME_ON_REQUEST_WITH_PLOT("module:GoalPerceptorRLE2:averaging",
    averageHoughSpaceConvolution(houghSpace, averagedHoughSpace, parameters.houghWidth, parameters.houghSpaceConvolutionKernel);
  );

  if(copyHoughSpaceForDrawing && useAveragedHoughSpace)
  {
    houghSpaceForDrawing.insert(houghSpaceForDrawing.begin(), averagedHoughSpace, averagedHoughSpace + parameters.houghWidth);
  }

  // this debug drawing must be drawn before the local maxima are cleared
  COMPLEX_DRAWING("module:GoalPerceptorRLE2:averagedHoughSpace",
    const int maxValue = *max_element(averagedHoughSpace, averagedHoughSpace + parameters.houghWidth);
    const float factor = (float)(cameraResolutionHeight - 1) / (float)maxValue;
    int lastXCoord = 0;
    for(int i = 1; i < parameters.houghWidth; ++i)
    {
      int xCoord = i * cameraResolutionWidth / parameters.houghWidth;
      LINE("module:GoalPerceptorRLE2:averagedHoughSpace", lastXCoord, cameraResolutionHeight - 1 - float(averagedHoughSpace[i - 1]) * factor, xCoord, cameraResolutionHeight - 1 - float(averagedHoughSpace[i]) * factor, 1, Drawings::ps_solid, ColorClasses::red);
      lastXCoord = xCoord;
    }
  );

  // find maxima in averaged hough space
  int* maxima = new int[parameters.houghSpaceMaxima];
  STOP_TIME_ON_REQUEST_WITH_PLOT("module:GoalPerceptorRLE2:findMaxima",
    findMaxima(averagedHoughSpace, maxima, parameters.houghSpaceMaxima, parameters.houghWidth, averagingWidth);
  );

  vector< vector<const RunLengthImage::Run*> > adjacentRuns;
  STOP_TIME_ON_REQUEST_WITH_PLOT("module:GoalPerceptorRLE2:getAdjacentRuns",
    adjacentRuns = vector< vector<const RunLengthImage::Run*> >(parameters.houghSpaceMaxima);
    for(int i = 0; i < parameters.houghSpaceMaxima; ++i)
    {
      HoughLine houghLine = getHoughLine(maxima[i]);
      getAdjacentRuns(houghLine, adjacentRuns[i]);
    }
  );

  vector< vector<Region> > regions;
  regions.resize(parameters.houghSpaceMaxima);

  bool skipping = true;
  MODIFY("module:GoalPerceptorRLE2:buildRegionsWithSkipping", skipping);
  STOP_TIME_ON_REQUEST_WITH_PLOT("module:GoalPerceptorRLE2:buildRegions",
    MM_EMPTY();
    if(skipping)
    {
      for(int i = 0; i < parameters.houghSpaceMaxima; ++i)
      {
        regions[i] = buildRegionsWithSkipping(adjacentRuns[i]);
      }
    }
    else
    {
      for(int i = 0; i < parameters.houghSpaceMaxima; ++i)
      {
        regions[i] = buildRegions(adjacentRuns[i]);
      }
    }
    MM_EMPTY();
  );

  vector<PotentialGoalPost> goalPosts;
  goalPosts.reserve(parameters.houghSpaceMaxima);

  //STOP_TIME_ON_REQUEST_WITH_PLOT("module:GoalPerceptorRLE2:processRegions",
    goalPosts.clear();

    // set up MMX registers with color prototypes and thresholds
    Image::Pixel thresholdYellow = theColorConfiguration.colors[ColorClasses::yellow].threshold.convertToPixel();
    Image::Pixel thresholdGreen = theColorConfiguration.colors[ColorClasses::green].threshold.convertToPixel();
    thresholdYellow.yCbCrPadding = thresholdGreen.yCbCrPadding = 255;
    MM_EMPTY();

    const __m64 mThresholdYellow = _mm_unpacklo_pi8(_mm_cvtsi32_si64(thresholdYellow.color), _mm_setzero_si64());
    const __m64 mThresholdGreen = _mm_unpacklo_pi8(_mm_cvtsi32_si64(thresholdGreen.color), _mm_setzero_si64());
    const __m64 mYellow = _mm_unpacklo_pi8(_mm_cvtsi32_si64(theColorConfiguration.colors[ColorClasses::yellow].color.convertToPixel().color), _mm_setzero_si64());
    const __m64 mGreen = _mm_unpacklo_pi8(_mm_cvtsi32_si64(theColorConfiguration.colors[ColorClasses::green].color.convertToPixel().color), _mm_setzero_si64());

    // process regions on the hough lines
    for(int i = 0; i < parameters.houghSpaceMaxima; ++i)
    {
      const vector<Region>& currentRegions = regions[i];

      // check if this hough line lies on anohter goal post already found
      bool liesOnAnotherGoalPost = false;
      const int houghIndex = maxima[i];
      for(vector<PotentialGoalPost>::const_iterator j = goalPosts.begin(); j != goalPosts.end(); ++j)
      {
        if(j->exclusionRange.isInside(houghIndex))
        {
          liesOnAnotherGoalPost = true;
        }
      }
      if(liesOnAnotherGoalPost)
      {
        continue;
      }

      // process one region after the other, from bottom to top
      for(vector<Region>::const_reverse_iterator j = currentRegions.rbegin(); j != currentRegions.rend(); ++j)
      {
        // first check if the region is large enough
        if(j->height < parameters.minHeight)
        {
          continue;
        }

        // check if the region is yellow enough
        const __m64 mColor = _mm_unpacklo_pi8(_mm_cvtsi32_si64(j->color.color), _mm_setzero_si64());
        const __m64 absDifferenceYellow = _mm_or_si64(_mm_subs_pu16(mColor, mYellow), _mm_subs_pu16(mYellow, mColor));
        if(_mm_movemask_pi8(_mm_cmpgt_pi16(absDifferenceYellow, mThresholdYellow)))
        {
          continue;
        }

        // check if there is some green below this region
        bool greenBelow = false;
        int count = 0;
        vector<Region>::const_iterator k = j.base();
        while(!greenBelow && count < parameters.searchDepthGreenBelow && k != currentRegions.end())
        {
          count += k->count;
          const __m64 mColor = _mm_unpacklo_pi8(_mm_cvtsi32_si64(k->color.color), _mm_setzero_si64());
          const __m64 absDifferenceGreen = _mm_or_si64(_mm_subs_pu16(mColor, mGreen), _mm_subs_pu16(mGreen, mColor));
          if(!_mm_movemask_pi8(_mm_cmpgt_pi16(absDifferenceGreen, mThresholdGreen)))
          {
            greenBelow = true;
            break;
          }
          ++k;
        }

        // TODO what if looking from the side, where there is a field line below?
        if(!greenBelow)
        {
          continue;
        }

        MM_EMPTY();
        // compute average width of the region
        int widthAcc = 0;
        for(const RunLengthImage::Run* const * p = j->pStart + 5; p != j->pEnd; ++p)
        {
          widthAcc += (*p)->runLength;
        }

        // the foot point y-coordinate is determined from the bottom-most run of the region
        Vector2<> footPoint;
        int y = (*(j->pEnd - 1))->y;
        footPoint.y = (float)y + theImageGrid.lineInformationLUT[y].rowInterval * 0.5f;

        // the foot point x-coordinate is the intersection of the bottom-most run and the hough line the region was found on
        const HoughLine houghLine = getHoughLine(maxima[i]);
        footPoint.x = houghLine.xStart + footPoint.y * houghLine.slope;

        PotentialGoalPost goalPost;
        goalPost.maximumIndex = i;
        goalPost.footPointInImage = footPoint;

        goalPost.averageWidth = (float)widthAcc / (float)(j->count - 5);

        const int yStart = (*(j->pStart))->y;
        const float fYStart = (float)yStart + theImageGrid.lineInformationLUT[yStart].rowInterval * 0.5f;
        goalPost.height = (int)(footPoint.y - fYStart);

        const Vector2<> footPointCorrected = theImageCoordinateSystem.toCorrected(footPoint);
        Geometry::calculatePointOnField(footPointCorrected, theCameraMatrix, theCameraInfo, goalPost.relPos);

        // consider that the foot point is on the surface, whereas relPos denotes the center point of the goal post
        goalPost.relPos.normalize(goalPost.relPos.abs() + (float)theFieldDimensions.goalPostRadius);

        // compute the width of the goal post in the image at the top and bottom to approximate the average expected width
        Range<> rangeYStart = getWidthOfGoalPostInImage(goalPost.relPos, fYStart);
        Range<> rangeYEnd = getWidthOfGoalPostInImage(goalPost.relPos, footPoint.y);
        const float expectedAverageWidth = ((rangeYStart.max - rangeYStart.min) + (rangeYEnd.max - rangeYEnd.min)) * 0.5f;

        // check if average width matches the expected width
        if(abs(expectedAverageWidth - goalPost.averageWidth) > expectedAverageWidth * parameters.widthToleranceFactor)
        {
          MM_EMPTY();
          continue;
        }

        // try to determine the laterality of the goal post by finding the goal bar
        goalPost.side = findGoalBar(*j, getHoughLine(maxima[i]), (int)getWidthOfGoalPostInImage(goalPost.relPos, j->pStart[parameters.searchHeightGoalPost / 2]->y).getSize());

        // exclude the (expected) area of the goal post in the image from further processings of hough lines
        const Range<> tempRange = getWidthOfGoalPostInImage(goalPost.relPos, (float)houghRefY);
        const float widthAtRefY = tempRange.max - tempRange.min;
        // TODO maybe parametrize?
        const int halfWidthInHoughSpace = (int)floor(widthAtRefY * houghPixelPerPixel);
        goalPost.exclusionRange = Range<int>(maxima[i] - halfWidthInHoughSpace, maxima[i] + halfWidthInHoughSpace);

        goalPosts.push_back(goalPost);

        // if we are here, we found a goal post on the current hough line, so we quit
        MM_EMPTY();
        break;
      }
    }
    MM_EMPTY();
  //);

  // do some high level sanity checks
  sanityChecks(goalPosts, goalPercept);

  COMPLEX_DRAWING("module:GoalPerceptorRLE2:goalRegions",
    for(unsigned int i = 0; i < goalPosts.size(); ++i)
    {
      const int maximumIndex = goalPosts[i].maximumIndex;
      const vector<Region>& region = regions[maximumIndex];
      const HoughLine houghLine = getHoughLine(maxima[maximumIndex]);
      for(vector<Region>::const_iterator j = region.begin(); j != region.end(); ++j)
      {
        const int yStart = (**(j->pStart)).y;
        const int yEnd = yStart + j->height;
        const int xStart = (int)(houghLine.xStart + (float)yStart * houghLine.slope + 0.5f);
        const int xEnd = (int)(houghLine.xStart + (float)yEnd * houghLine.slope + 0.5f);

        Image::Pixel rgbColor;
        ColorModelConversions::fromYCbCrToRGB(j->color.y, j->color.cb, j->color.cr, rgbColor.r, rgbColor.g, rgbColor.b);

        LINE("module:GoalPerceptorRLE2:goalRegions", xStart, yStart, xEnd, yEnd, 15, Drawings::ps_solid, ColorRGBA(rgbColor.r, rgbColor.g, rgbColor.b));
        //LINE("module:GoalPerceptorRLE2:goalRegions", xEnd - 20, yEnd, xEnd + 20, yEnd, 1, Drawings::ps_solid, ColorClasses::red);
      }
    }
  );

  COMPLEX_DEBUG_IMAGE(goalSegments,
    INIT_DEBUG_IMAGE_BLACK(goalSegments);
    for(unsigned int i = 0; i < goalPosts.size(); ++i)
    {
      const vector<const RunLengthImage::Run*>& runs = adjacentRuns[i];
      for(vector<const RunLengthImage::Run*>::const_iterator j = runs.begin(); j != runs.end(); ++j)
      {
        const RunLengthImage::Run* run = *j;
        const int rowInterval = theImageGrid.lineInformationLUT[run->y].rowInterval;

        for(int yy = run->y; yy < run->y + rowInterval; ++yy)
        {
          for(int xx = run->xStart; xx <= run->xEnd; ++xx)
          {
            DEBUG_IMAGE_SET_PIXEL_YUV(goalSegments, xx, yy, run->color.y, run->color.cb, run->color.cr);
          }
        }
      }
    }
    SEND_DEBUG_IMAGE(goalSegments);
  );

  COMPLEX_DRAWING("module:GoalPerceptorRLE2:houghMaxima",
    for(int i = 0; i < parameters.houghSpaceMaxima; ++i)
    {
      ColorRGBA color(255 - i * 255 / parameters.houghSpaceMaxima, 0, 0);
      HoughLine houghLine = getHoughLine(maxima[i]);
      const float xEnd = houghLine.xStart + houghLine.slope * float(cameraResolutionHeight);
      LINE("module:GoalPerceptorRLE2:houghMaxima", houghLine.xStart, 0, xEnd, cameraResolutionHeight, 1, Drawings::ps_solid, color);
    }
  );

  COMPLEX_DRAWING("module:GoalPerceptorRLE2:houghSpace",
    const int maxValue = *max_element(houghSpace, houghSpace + parameters.houghWidth);
    const float factor = (float)(cameraResolutionHeight - 1) / (float)maxValue;
    int lastXCoord = 0;
    for(int i = 1; i < parameters.houghWidth; ++i)
    {
      int xCoord = i * cameraResolutionWidth / parameters.houghWidth;
      LINE("module:GoalPerceptorRLE2:houghSpace", lastXCoord, cameraResolutionHeight - 1 - float(houghSpace[i - 1]) * factor, xCoord, cameraResolutionHeight - 1 - float(houghSpace[i]) * factor, 1, Drawings::ps_solid, ColorClasses::black);
      lastXCoord = xCoord;
    }
  );

  int vanishingPointLines = 40;
  MODIFY("module:GoalPerceptorRLE2:vanishingPointLines", vanishingPointLines);
  COMPLEX_DRAWING("module:GoalPerceptorRLE2:vanishingPoint",
    const float scale = (float)cameraResolutionWidth / (float)vanishingPointLines;
    for(int i = 0; i < vanishingPointLines; ++i)
    {
      const float x = (float)i * scale;
      LINE("module:GoalPerceptorRLE2:vanishingPoint", x, 0, vanishingPoint.x, vanishingPoint.y, 1, Drawings::ps_solid, ColorClasses::black);
    }
  );

  COMPLEX_DRAWING("module:GoalPerceptorRLE2:buckets",
    for(int i = 0; i <= parameters.houghWidth; ++i)
    {
      const float x1 = xStart + i * widthPerHoughPixel;
      const float y1 = (float)(houghSpaceStartAtLowerBorder ? yEnd : yStart);

      const float y2 = (float)(houghSpaceStartAtLowerBorder ? yStart : yEnd);
      const float x2 = (x1 - vanishingPoint.x) * (y2 - vanishingPoint.y) / (y1 - vanishingPoint.y) + vanishingPoint.x;
      LINE("module:GoalPerceptorRLE2:buckets", x1, y1, x2, y2, 1, Drawings::ps_solid, ColorClasses::black);
    }
  );


  COMPLEX_DEBUG_IMAGE(goalPostSegmentWeighting,
    const RunLengthImage::Run* p = theRunLengthImage.image;
    const RunLengthImage::Run* const pEnd = p + theRunLengthImage.numberOfRuns;

    const Image::Pixel yellow = theColorConfiguration.colors[ColorClasses::yellow].color.convertToPixel();
    const int minValue = 255 * parameters.houghSpaceMinValue / parameters.houghSpaceBaseValue;
    const float scale = 255.0f / (float)(parameters.houghSpaceBaseValue - parameters.houghSpaceMinValue);

    while(p != pEnd)
    {
      int currentY = p->y;
      int rowInterval = theImageGrid.lineInformationLUT[currentY].rowInterval;
      do
      {
        const int diff = abs(p->color.y - yellow.y) + abs(p->color.cb - yellow.cb) + abs(p->color.cr - yellow.cr);
        const int color = max(255 - (int)((float)diff * scale), minValue);

        for(int yy = currentY; yy < currentY + rowInterval; ++yy)
        {
          for(int xx = p->xStart; xx <= p->xEnd; ++xx)
          {
            DEBUG_IMAGE_SET_PIXEL_YUV(goalPostSegmentWeighting, xx, yy, color, 127, 127);
          }
        }
        ++p;
      }
      while(p->y == currentY);
    }
    SEND_DEBUG_IMAGE(goalPostSegmentWeighting);
  );

  COMPLEX_DEBUG_IMAGE(houghSpace,
    const int maxValue = *max_element(houghSpaceForDrawing.begin(), houghSpaceForDrawing.end());
    for(int yy = yStart; yy < yEnd; ++yy)
    {
      const int y2 = precomp4 - yy;
      const int bScaled = precomp1 / y2 + precomp2;
      int fScaled = precomp3 / y2;
      fScaled <<= 1;
      for(int xx = 0; xx < cameraResolutionWidth; ++xx)
      {
        const int houghIndex = (bScaled + ((xx * fScaled) >> (SCALE_F - SCALE_B))) >> SCALE_B;
        const int color = houghSpaceForDrawing[houghIndex] * 255 / maxValue;
        DEBUG_IMAGE_SET_PIXEL_YUV(houghSpace, xx, yy, color, 127, 127);
      }
    }
    SEND_DEBUG_IMAGE(houghSpace);
  );

  COMPLEX_DRAWING("module:GoalPerceptorRLE2:houghSpace2",
    const int maxValue = *max_element(houghSpaceForDrawing.begin(), houghSpaceForDrawing.end());
    for(int i = 0; i < parameters.houghWidth; ++i)
    {
      Vector2<> points[2][2];
      for(int j = 0; j < 2; ++j)
      {
        points[j][0].x = xStart + (float)(i + j) * widthPerHoughPixel;
        points[j][0].y = (float)(houghSpaceStartAtLowerBorder ? yEnd : yStart);
        points[j][1].y = (float)(houghSpaceStartAtLowerBorder ? yStart : yEnd);
        points[j][1].x = (points[j][0].x - vanishingPoint.x) * (points[j][1].y - vanishingPoint.y) / (points[j][0].y - vanishingPoint.y) + vanishingPoint.x;
      }
      Vector2<> pointList[4];
      pointList[0] = points[0][0];
      pointList[1] = points[0][1];
      pointList[2] = points[1][1];
      pointList[3] = points[1][0];

      const int color = houghSpaceForDrawing[i] * 255 / maxValue;

      POLYGON("module:GoalPerceptorRLE2:houghSpace2", 4, pointList, 1, Drawings::ps_solid, ColorRGBA(color, color, color), Drawings::bs_solid, ColorRGBA(color, color, color));
    }
  );


  delete[] rawHoughSpace;
  delete[] maxima;
  delete[] rawAveragedHoughSpace;

  draw();
}

void GoalPerceptorRLE2::averageHoughSpaceConvolution(const unsigned int* houghSpace, unsigned int* averagedHoughSpace, int houghSpaceWidth, vector<int>& coefficients) const
{
  const int averagingWidth = coefficients.size();
  const unsigned int* shiftedHoughSpace = houghSpace - averagingWidth / 2;

  for(int i = 0; i < houghSpaceWidth; ++i)
  {
    int sum = 0;
    for(int j = 0; j < averagingWidth; ++j)
    {
      sum += coefficients[j] * shiftedHoughSpace[i + j];
    }
    averagedHoughSpace[i] = sum;
  }
}

void GoalPerceptorRLE2::findMaxima(unsigned int* averagedHoughSpace, int* maxima, int numOfMaxima, int averagedHoughSpaceWidth, int clearingWidth) const
{
  for(int i = 0; i < numOfMaxima; ++i)
  {
    unsigned int* maxElem = max_element(averagedHoughSpace, averagedHoughSpace + averagedHoughSpaceWidth);
    int index = maxElem - averagedHoughSpace; // index of maximum

    // check if there are other maxima, which would indicate a plateau
    // if so, advance to the center of the plateau
    int temp = index;
    while(++temp < parameters.houghSpaceMaxima && averagedHoughSpace[index] == *maxElem);
    index = (index + temp) / 2;
    maxElem = averagedHoughSpace + index;
    maxima[i] = index;

    // clear hough space around maximum
    unsigned int* p = maxElem - clearingWidth / 2;
    unsigned int* pEnd = maxElem + (clearingWidth + 1) / 2;
    while(p < pEnd)
    {
      *p = 0;
      ++p;
    }
  }
}

void GoalPerceptorRLE2::getAdjacentRuns(HoughLine& houghLine, vector<const RunLengthImage::Run*>& runs) const
{
  runs.clear();
  runs.reserve(theImageGrid.numberOfLines + 1); // reserve a sufficiently large amount of memory to avoid resizing
  const RunLengthImage::Run* pStart = theRunLengthImage.image;

  // first compute start and end row by clipping the hough line with the image border
  int yStart = 0;
  if(houghLine.xStart < 0.0f)
  {
    float y = houghLine.xStart / houghLine.slope;
    yStart = (int)ceil(-y);
  }
  else if(houghLine.xStart > float(cameraResolutionWidth - 1))
  {
    float y = (houghLine.xStart - float(cameraResolutionWidth - 1)) / houghLine.slope;
    yStart = (int)ceil(-y);
  }

  if(yStart != 0)
  {
    if(yStart >= theImageGrid.lastYCoord)
    {
      return;
    }
    while(pStart->y < yStart)
    {
      pStart = pStart->firstOverlapBelow;
    }
  }

  int yEnd = cameraResolutionHeight;
  const float xEnd = houghLine.xStart + houghLine.slope * (float)cameraResolutionHeight;
  if(xEnd < 0.0f)
  {
    yEnd -= (int)(xEnd / houghLine.slope);
  }
  else if(xEnd >= float(cameraResolutionWidth - 1))
  {
    yEnd -= (int)((xEnd - float(cameraResolutionWidth - 1)) / houghLine.slope);
  }

  // precompute integer values to avoid floating point computations
  // for computing the x-coordinate on the hough line
  const int iSlope = (int)(houghLine.slope * (float)(1 << 20) + 0.5f);
  const int iXStart = (int)(houghLine.xStart * (float)(1 << 20) + 0.5f);
  const RunLengthImage::Run* p = pStart;
  while(p != &theRunLengthImage.voidRun && p->y < yEnd)
  {
    const int x = (iXStart + p->y * iSlope) >> 20;
    ASSERT(x >= 0 && x < cameraResolutionWidth);

    // advance pointer to run that crosses the hough line
    while(p->xEnd < x)
    {
      ++p;
    }
    runs.push_back(p);
    p = p->firstOverlapBelow;
  }
}

vector<GoalPerceptorRLE2::Region> GoalPerceptorRLE2::buildRegions(const vector<const RunLengthImage::Run*>& runs) const
{
  vector<Region> regions;
  if(runs.empty())
  {
    return regions;
  }
  regions.reserve(10);

  const int size = runs.size();

  const RunLengthImage::Run* const * pStart = &runs.front();
  const RunLengthImage::Run* const * pEnd = pStart + size;

  const RunLengthImage::Run* const * p = pStart;


  __m64 currentColor = _mm_unpacklo_pi8(_mm_cvtsi32_si64((*p)->color.color), _mm_setzero_si64());
  while(p != pEnd)
  {
    Region region;
    region.pStart = p;
    const int yStart = (**p).y;
    // due to some MMX restrictions, at most 127 runs can be accumulated
    const RunLengthImage::Run* const * pEnd2 = p + min(127, int(pEnd - p));
    __m64 accColor = currentColor;
    ++p;
    int count = 1; // the accumulators initially hold one run
    int extraRounds = 0; // counts how often the accumulators had to be halved due to the "127-restriction"
extraRound:
    while(p != pEnd2)
    {
      currentColor = _mm_unpacklo_pi8(_mm_cvtsi32_si64((*p)->color.color), _mm_setzero_si64());
      __m64 mCount = _mm_shuffle_pi16(_mm_cvtsi32_si64(count), 0);
      __m64 scaledColor = _mm_mullo_pi16(currentColor, mCount);
      __m64 scaledThreshold = _mm_mullo_pi16(mThreshold, mCount);
      __m64 absDifference = _mm_or_si64(_mm_subs_pu16(accColor, scaledColor), _mm_subs_pu16(scaledColor, accColor));
      const int mask = _mm_movemask_pi8(_mm_cmpgt_pi16(absDifference, scaledThreshold));

      if(mask)
      { // end of region
        break;
      }
      else
      {
        accColor = _mm_add_pi16(accColor, currentColor);
        ++count;
      }
      ++p;
    }
    if(p == pEnd2 && p != pEnd)
    {
      // if the limit of 127 runs is reached, half the accumulator and the counter, and increase the limit by that half
      ++extraRounds;
      count /= 2;
      accColor = _mm_srli_pi16(accColor, 1);
      pEnd2 += std::min(64, int(pEnd - pEnd2));
      goto extraRound;
    }

    // use LUT with scaled reciprocals for fast division using multiplication
    const __m64 divisor = divisionLUT[count - 1];
    region.color.color = _mm_cvtsi64_si32(_mm_packs_pu16(_mm_mulhi_pu16(accColor, divisor), accColor));
    region.count = count + extraRounds * 64;
    region.pEnd = p;
    region.height = (p == pEnd ? cameraResolutionHeight : (**p).y) - yStart;

    regions.push_back(region);
  }

  MM_EMPTY();
  return regions;
}

vector<GoalPerceptorRLE2::Region> GoalPerceptorRLE2::buildRegionsWithSkipping(const vector<const RunLengthImage::Run*>& runs) const
{
  vector<Region> regions;
  if(runs.empty())
  {
    return regions;
  }
  regions.reserve(10);

  const int size = runs.size();

  const RunLengthImage::Run* const * pStart = &runs.front();
  const RunLengthImage::Run* const * pEnd = pStart + size;

  const RunLengthImage::Run* const * p = pStart;

  __m64 currentColor = _mm_unpacklo_pi8(_mm_cvtsi32_si64((*p)->color.color), _mm_setzero_si64());
  while(p != pEnd)
  {
    Region region;
    region.pStart = p;
    const int yStart = (**p).y;
    // due to some MMX restrictions, at most 127 runs can be accumulated
    const RunLengthImage::Run* const * pEnd2 = p + min(127, int(pEnd - p));
    __m64 accColor = currentColor;
    ++p;
    int count = 1; // the accumulators initially hold one run
    int skippedRuns = 0;
    int extraRounds = 0; // counts how often the accumulators had to be halved due to the "127-restriction"
extraRound:
    while(p != pEnd2 && skippedRuns <= parameters.maxSkipRuns)
    {
      currentColor = _mm_unpacklo_pi8(_mm_cvtsi32_si64((*p)->color.color), _mm_setzero_si64());
      __m64 mCount = _mm_shuffle_pi16(_mm_cvtsi32_si64(count), 0);
      __m64 scaledColor = _mm_mullo_pi16(currentColor, mCount);
      __m64 scaledThreshold = _mm_mullo_pi16(mThreshold, mCount);
      __m64 absDifference = _mm_or_si64(_mm_subs_pu16(accColor, scaledColor), _mm_subs_pu16(scaledColor, accColor));
      const int mask = _mm_movemask_pi8(_mm_cmpgt_pi16(absDifference, scaledThreshold));

      if(mask)
      { // non-matching run (not too bad as long as there are not too much)
        ++skippedRuns;
      }
      else
      {
        accColor = _mm_add_pi16(accColor, currentColor);
        ++count;
      }
      ++p;
    }
    if(p == pEnd2 && p != pEnd)
    {
      // if the limit of 127 runs is reached, half the accumulator and the counter, and increase the limit by that half
      ++extraRounds;
      count /= 2;
      accColor = _mm_srli_pi16(accColor, 1);
      pEnd2 += min(64, int(pEnd - pEnd2));
      goto extraRound;
    }

    p -= skippedRuns;

    // use LUT with scaled reciprocals for fast division using multiplication
    const __m64 divisor = divisionLUT[count - 1];
    region.color.color = _mm_cvtsi64_si32(_mm_packs_pu16(_mm_mulhi_pu16(accColor, divisor), accColor));
    region.count = count + extraRounds * 64;
    region.pEnd = p;
    region.height = (p == pEnd ? cameraResolutionHeight : (**p).y) - yStart;

    regions.push_back(region);
  }

  MM_EMPTY();
  return regions;
}

GoalPost::Position GoalPerceptorRLE2::findGoalBar(const Region& region, const HoughLine& houghLine, int expectedWidth) const
{
  const int yCenter = (**(region.pStart + parameters.searchHeightGoalPost / 2)).y;
  const int xCenter = (int)(houghLine.xStart + houghLine.slope * (float)yCenter + 0.5f);
  const int xStart = xCenter - expectedWidth / 2;
  const int xEnd = xCenter + expectedWidth / 2;

  // first check if one of the uppermost runs belongs to a goal post,
  // i.e., it is substantially longer than the expected width of the goal post
  for(int i = 0; i < parameters.searchHeightGoalPost; ++i)
  {
    const RunLengthImage::Run* currentRun = region.pStart[i];
    if(currentRun->runLength > 2 * expectedWidth)
    {
      if(abs(currentRun->xStart - xStart) < 5)
      {
        return GoalPost::IS_LEFT;
      }
      else if(abs(currentRun->xEnd - xEnd) < 5)
      {
        return GoalPost::IS_RIGHT;
      }
    }
  }

  // then check the runs next to the uppermost runs
  MM_EMPTY();
  const __m64 mZero = _mm_setzero_si64();
  const __m64 mGoalpostColor = _mm_unpacklo_pi8(_mm_cvtsi32_si64(region.color.color), mZero);
  const __m64 threshold = mThreshold;
  for(int i = 0; i < parameters.searchHeightGoalPost; ++i)
  {
    const RunLengthImage::Run* p = region.pStart[i];
    const int y = p->y;

    // first search left
    const RunLengthImage::Run* p2 = p - 1;
    int width = 0;
    while(p2->y == y && width <= expectedWidth) // search only in current line and not more than a ceratin witdth
    {
      // compute color difference of that run to the average color of the goal post
      const __m64 mCurrentColor = _mm_unpacklo_pi8(_mm_cvtsi32_si64(p2->color.color), mZero);
      const __m64 mAbsDifference = _mm_or_si64(_mm_subs_pu16(mGoalpostColor, mCurrentColor), _mm_subs_pu16(mCurrentColor, mGoalpostColor));
      const bool colorMatches = _mm_movemask_pi8(_mm_cmpgt_pi16(mAbsDifference, threshold)) == 0;
      if(colorMatches && p2->runLength >= expectedWidth) // if the run is long enough
      {
        MM_EMPTY();
        return GoalPost::IS_RIGHT;
      }
      width += p2->runLength;
      --p2;
    }

    // then search right
    p2 = p + 1;
    width = 0;
    while(p2->y == y && width <= expectedWidth) // search only in current line and not more than a ceratin witdth
    {
      // compute color difference of that run to the average color of the goal post
      const __m64 mCurrentColor = _mm_unpacklo_pi8(_mm_cvtsi32_si64(p2->color.color), mZero);
      const __m64 mAbsDifference = _mm_or_si64(_mm_subs_pu16(mGoalpostColor, mCurrentColor), _mm_subs_pu16(mCurrentColor, mGoalpostColor));
      const bool colorMatches = _mm_movemask_pi8(_mm_cmpgt_pi16(mAbsDifference, threshold)) == 0;
      if(colorMatches && p2->runLength >= expectedWidth) // if the run is long enough
      {
        MM_EMPTY();
        return GoalPost::IS_LEFT;
      }
      width += p2->runLength;
      ++p2;
    }
  }

  MM_EMPTY();
  return GoalPost::IS_UNKNOWN;
}

void GoalPerceptorRLE2::sanityChecks(vector<PotentialGoalPost>& goalPosts, GoalPercept& goalPercept) const
{
  if(goalPosts.empty())
  {
    return;
  }

  // sort goal posts by color (primarily) and by angle to goal post (secondarily)
  sort(goalPosts.begin(), goalPosts.end());

  // more than two posts of a color or differently colored posts at once are not sane
  if(!(goalPosts.size() > 2))
  {
    if(goalPosts.size() == 2 && (goalPosts[0].side == GoalPost::IS_RIGHT || goalPosts[1].side == GoalPost::IS_LEFT))
    {
      return;
    }
    
    goalPercept.timeWhenGoalPostLastSeen = theFrameInfo.time;
    if(goalPosts.size() == 2)
    {
      // if two posts of the same color are seen, their laterality can be surely determined
      goalPosts[0].side = GoalPost::IS_LEFT;
      goalPosts[1].side = GoalPost::IS_RIGHT;
      goalPercept.timeWhenCompleteGoalLastSeen = theFrameInfo.time;
    }
    for(vector<PotentialGoalPost>::const_iterator i = goalPosts.begin(); i != goalPosts.end(); ++i)
    {
      goalPercept.goalPosts.push_back(convertGoalPost(*i));
    }
  }
}

void GoalPerceptorRLE2::draw() const
{
}

Vector2<> GoalPerceptorRLE2::computeVanishingPoint(int axis) const
{
  // compute vanishing point for a worlds axis by intersecting
  // that axis with the image plane
  Vector3<> vAxis = theCameraMatrix.rotation.transpose()[axis];
  float factor = theCameraInfo.focalLength / vAxis.x;
  if(abs(factor) > 1e6f)
  {
    factor = 1e6f;
  }
  vAxis *= factor;
  Vector2<> vanishingPoint(vAxis.y, vAxis.z);
  vanishingPoint = theCameraInfo.opticalCenter - vanishingPoint;
  return vanishingPoint;
}

GoalPerceptorRLE2::HoughLine GoalPerceptorRLE2::getHoughLine(int index)
{
  // compute hough line corresponding to a hough index using precomputed values
  const float temp = (float)index;
  return HoughLine(temp * houghLineXStartSlope + houghLineXStartIntercept, temp * houghLineSlopeSlope + houghLineSlopeIntercept);
}

GoalPost GoalPerceptorRLE2::convertGoalPost(const PotentialGoalPost& goalPost) const
{
  GoalPost result;
  result.positionInImage = Vector2<int>(int(goalPost.footPointInImage.x + 0.5f), int(goalPost.footPointInImage.y + 0.5f));
  result.positionOnField = Vector2<int>(int(goalPost.relPos.x + 0.5f), int(goalPost.relPos.y + 0.5f));
  result.distanceType = GoalPost::BEARING_BASED;
  result.position = goalPost.side;
  return result;
}

Range<> GoalPerceptorRLE2::getWidthOfGoalPostInImage(const Vector2<>& fieldCoords, float y) const
{
  // the width in the image is acquired by intersecting the vertical border lines of the goal post
  // with the plane in the world corresponding to a single row in the image
  const Vector2<> cameraMatrixOffset(theCameraMatrix.translation.x, theCameraMatrix.translation.y);
  Vector2<> goalPostOffset(fieldCoords - cameraMatrixOffset); // offset of the goal post relative to the camera projected onto the ground
  goalPostOffset.normalize((float)theFieldDimensions.goalPostRadius);
  goalPostOffset.rotateLeft(); // the border lines start orthonormal (left and right) of the goal post center point
  Vector2<> footPoints[2] = {fieldCoords + goalPostOffset, fieldCoords - goalPostOffset};
  // consider that the viewing rays actually hit the goal post tangentially
  footPoints[0] = fieldCoords + footPoints[0].normalize((float)theFieldDimensions.goalPostRadius).rotateLeft();
  footPoints[1] = fieldCoords + footPoints[1].normalize((float)theFieldDimensions.goalPostRadius).rotateRight();
  float xCoords[2];

  const float yOffset = theCameraInfo.opticalCenter.y - y;
  Pose3D rotatedCamMatrix(theCameraMatrix);
  rotatedCamMatrix.rotateY(-atan2(yOffset, theCameraInfo.focalLength));
  rotatedCamMatrix = rotatedCamMatrix.invert();
  // the xy-plane of that matrix now corresponds the y-line in the image

  for(int i = 0; i < 2; ++i)
  {
    // actual intersection
    const Vector2<>& footPoint(footPoints[i]);
    Vector3<> footPoint3(footPoint.x, footPoint.y, 0);
    footPoint3 = rotatedCamMatrix * footPoint3;
    footPoint3 -= rotatedCamMatrix.rotation.c2 * (footPoint3.z / rotatedCamMatrix.rotation.c2.z);
    footPoint3 *= theCameraInfo.focalLength / footPoint3.x;
    xCoords[i] = theCameraInfo.opticalCenter.x - footPoint3.y;
  }

  return Range<>(xCoords[0], xCoords[1]);
}
