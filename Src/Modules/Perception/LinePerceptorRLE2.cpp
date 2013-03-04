/**
* @file LinePerceptorRLE2.cpp
* This file implements a module that provides the line percept only using the RunLengthImage.
* @author Alexander Härtl
*/

#include "LinePerceptorRLE2.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Matrix.h"
#include "Tools/Math/Geometry.h"
#include "Modules/Perception/RunLengthImageProvider.h"
#include <algorithm>

MAKE_MODULE(LinePerceptorRLE2, Perception)

const ColorClasses::Color LinePerceptorRLE2::type2color[] =  {ColorClasses::red, ColorClasses::orange, ColorClasses::yellow, ColorClasses::green, ColorClasses::black, ColorClasses::robotBlue};

void LinePerceptorRLE2::init()
{
  RunLengthImageProvider::createLutUsingC(divisionLUT, cameraResolutionHeight);

  DECLARE_PLOT("module:LinePerceptorRLE2:numberOfRuns");
  DECLARE_PLOT("module:LinePerceptorRLE2:numberOfRegions");
  DECLARE_PLOT("module:LinePerceptorRLE2:numberOfShapedRegions");
  DECLARE_PLOT("module:LinePerceptorRLE2:circleError");
}

void LinePerceptorRLE2::preExecution()
{
  MODIFY("module:LinePerceptorRLE2:parameters", parameters);

  DECLARE_DEBUG_DRAWING("module:LinePerceptorRLE2:adjacency", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:LinePerceptorRLE2:adjacencyRegions", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:LinePerceptorRLE2:shapedRegions", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:LinePerceptorRLE2:shapedRegionsAfterMerging", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:LinePerceptorRLE2:circleCenters", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:LinePerceptorRLE2:parameterSpace", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:LinePerceptorRLE2:projectedRegions", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:LinePerceptorRLE2:circleCenter", "drawingOnField");

  Image::Pixel threshold;
  threshold.y = parameters.thresholdY;
  threshold.cb = parameters.thresholdCb;
  threshold.cr = parameters.thresholdCr;
  threshold.yCbCrPadding = 255;

  mThreshold = _mm_unpacklo_pi8(_mm_cvtsi32_si64(threshold.color), _mm_setzero_si64());
  MM_EMPTY();
}

void LinePerceptorRLE2::update(LinePercept& linePercept)
{
  preExecution();

  linePercept.clear();

  // only proceed if the camera matrix is valid
  if(theCameraMatrix.isValid)
  {
    const Vector3<>& xAxis3D = theCameraMatrix.rotation.c0;
    const Vector2<> xAxis(Vector2<>(xAxis3D.x, xAxis3D.y).abs(), xAxis3D.z);

    // the y start coordinate (first row to be processd) is determined by back projecting a point in the distance of the field diagonal
    int yStartCoord;
    {
      const float fieldDiagonal = 2.0f * Vector2<>((float)theFieldDimensions.xPosOpponentGroundline, (float)theFieldDimensions.yPosLeftSideline).abs();
      const Vector2<> transformedXAxis(xAxis.x, -xAxis.y);
      Vector2<> temp(Vector2<>(transformedXAxis).rotateRight() * theCameraMatrix.translation.z + transformedXAxis * fieldDiagonal);
      temp *= theCameraInfo.focalLength / temp.x;
      yStartCoord = (int)ceil(theCameraInfo.opticalCenter.y - temp.y);
    }
    if(yStartCoord < 0)
    {
      yStartCoord = 0;
    }

    pStart = theRunLengthImage.image;

    // move pointer to start coordinate
    if(yStartCoord > theImageGrid.lastYCoord)
    {
      return;
    }
    else
    {
      while(pStart->y < yStartCoord)
      {
        pStart = pStart->firstOverlapBelow;
      }
    }

    numberOfRuns = theRunLengthImage.beginOfLastLine - pStart;
    PLOT("module:LinePerceptorRLE2:numberOfRuns", numberOfRuns);

    STOP_TIME_ON_REQUEST("module:LinePerceptorRLE2:computeAdjacency", computeAdjacency(pStart););

    vector<Region> regions;
    regions.reserve(numberOfRuns); // avoid reallocaction

    STOP_TIME_ON_REQUEST("module:LinePerceptorRLE2:buildRegions",
      if(parameters.buildRegionsWithColorSplitting)
      {
        buildRegionsWithColorSplitting(regions);
      }
      else
      {
        buildRegions(regions);
      }
    );

    PLOT("module:LinePerceptorRLE2:numberOfRegions", regions.size());

    COMPLEX_DRAWING("module:LinePerceptorRLE2:adjacencyRegions",
      for(vector<Region>::iterator i = regions.begin(); i != regions.end(); ++i)
      {
        const RunLengthImage::Run* run1 = i->run;
        for(int j = 1; j < i->numberOfRuns; ++j)
        {
          const RunLengthImage::Run* run2 = adjacency[run1 - pStart].run;
          const ColorClasses::Color color = run1->color.y > 127 ? ColorClasses::black : ColorClasses::white;
          LINE("module:LinePerceptorRLE2:adjacencyRegions", (run1->xStart + run1->xEnd) / 2, run1->y, (run2->xStart + run2->xEnd) / 2, run2->y, 1, Drawings::ps_solid, color);
          run1 = run2;
        }
      }
    );

    STOP_TIME_ON_REQUEST("module:LinePerceptorRLE2:checkShape", checkShape(regions); );

    COMPLEX_DRAWING("module:LinePerceptorRLE2:shapedRegions",
      for(vector<Region>::iterator i = regions.begin(); i != regions.end(); ++i)
      {
        LINE("module:LinePerceptorRLE2:shapedRegions", i->startInImage.x, i->startInImage.y, i->endInImage.x, i->endInImage.y, 1, Drawings::ps_solid, type2color[i->type]);
      }
    );

    STOP_TIME_ON_REQUEST("module:LinePerceptorRLE2:checkGreen", checkGreen(regions); );

    PLOT("module:LinePerceptorRLE2:numberOfShapedRegions", regions.size());

    STOP_TIME_ON_REQUEST("module:LinePerceptorRLE2:projectOnField", projectOntoField(regions); );

    COMPLEX_DRAWING("module:LinePerceptorRLE2:projectedRegions",
      for(vector<Region>::const_iterator i = regions.begin(); i != regions.end(); ++i)
      {
        LINE("module:LinePerceptorRLE2:projectedRegions", i->startOnField.x, i->startOnField.y, i->endOnField.x, i->endOnField.y, 10, Drawings::ps_solid, type2color[i->type]);
      }
    );

    COMPLEX_DRAWING("module:LinePerceptorRLE2:circleCenters",
      for(vector<Region>::iterator i = regions.begin(); i != regions.end(); ++i)
      {
        if(!i->shortSegment)
        {
          continue;
        }
        for(int j = 0; j < 2; ++j)
        {
          CIRCLE("module:LinePerceptorRLE2:circleCenters", i->circleCenters[j].x, i->circleCenters[j].y, 8, 1, Drawings::ps_solid, ColorClasses::red, Drawings::bs_solid, ColorClasses::red);
        }
        LINE("module:LinePerceptorRLE2:circleCenters", i->circleCenters[0].x, i->circleCenters[0].y, i->circleCenters[1].x, i->circleCenters[1].y, 2, Drawings::ps_dot, ColorClasses::red);
      }
    );

    COMPLEX_DRAWING("module:LinePerceptorRLE2:parameterSpace",
      // draw coordinate axes
      LINE("module:LinePerceptorRLE2:parameterSpace", theFieldDimensions.yPosRightFieldBorder, 0, theFieldDimensions.yPosLeftFieldBorder, 0, 4, Drawings::ps_solid, ColorClasses::black);
      LINE("module:LinePerceptorRLE2:parameterSpace", 0, theFieldDimensions.yPosRightFieldBorder, 0, theFieldDimensions.yPosLeftFieldBorder, 4, Drawings::ps_solid, ColorClasses::black);
      LINE("module:LinePerceptorRLE2:parameterSpace", theFieldDimensions.yPosLeftFieldBorder, 0, theFieldDimensions.yPosLeftFieldBorder - 100, 100, 4, Drawings::ps_solid, ColorClasses::black);
      LINE("module:LinePerceptorRLE2:parameterSpace", theFieldDimensions.yPosLeftFieldBorder, 0, theFieldDimensions.yPosLeftFieldBorder - 100, -100, 4, Drawings::ps_solid, ColorClasses::black);
      LINE("module:LinePerceptorRLE2:parameterSpace", 0, theFieldDimensions.yPosLeftFieldBorder, 100, theFieldDimensions.yPosLeftFieldBorder - 100, 4, Drawings::ps_solid, ColorClasses::black);
      LINE("module:LinePerceptorRLE2:parameterSpace", 0, theFieldDimensions.yPosLeftFieldBorder, -100, theFieldDimensions.yPosLeftFieldBorder - 100, 4, Drawings::ps_solid, ColorClasses::black);
      DRAWTEXT("module:LinePerceptorRLE2:parameterSpace", theFieldDimensions.xPosOpponentFieldBorder / 2, 50, 200, ColorClasses::black, "d");
      DRAWTEXT("module:LinePerceptorRLE2:parameterSpace", 50, theFieldDimensions.yPosLeftFieldBorder / 2, 200, ColorClasses::black, "alpha");

      const float scaleD = (float)(theFieldDimensions.xPosOpponentFieldBorder - theFieldDimensions.xPosOwnFieldBorder) / sqrt((float)sqr(theFieldDimensions.xPosOpponentFieldBorder - theFieldDimensions.xPosOwnFieldBorder) + (float)sqr(theFieldDimensions.yPosLeftFieldBorder - theFieldDimensions.yPosRightFieldBorder));
      const float scaleAlpha = (float)(theFieldDimensions.yPosLeftFieldBorder - theFieldDimensions.yPosRightFieldBorder) / pi2;
      for(vector<Region>::iterator i = regions.begin(); i != regions.end(); ++i)
      {
        if(i->type >= Region::Circle)
        {
          CIRCLE("module:LinePerceptorRLE2:parameterSpace", i->d * scaleD, i->alpha * scaleAlpha, 10, 1, Drawings::ps_solid, ColorClasses::black, Drawings::bs_solid, ColorClasses::black);
        }
      }
    );

    STOP_TIME_ON_REQUEST("module:LinePerceptorRLE2:findCircle", findCircle(regions, linePercept); );

    STOP_TIME_ON_REQUEST("module:LinePerceptorRLE2:mergeRegions", mergeRegions(regions); );

    COMPLEX_DRAWING("module:LinePerceptorRLE2:shapedRegionsAfterMerging",
      for(vector<Region>::iterator i = regions.begin(); i != regions.end(); ++i)
      {
        if(i->valid)
        {
          LINE("module:LinePerceptorRLE2:shapedRegionsAfterMerging", i->startInImage.x, i->startInImage.y, i->endInImage.x, i->endInImage.y, 1, Drawings::ps_solid, type2color[i->type]);
        }
      }
    );

    createLines(regions, linePercept);

    STOP_TIME_ON_REQUEST("module:LinePerceptorRLE2:findIntersections", findIntersections(regions, linePercept); );

  }

  // draw line percept
  linePercept.drawOnField(theFieldDimensions, 0);
  linePercept.drawOnImage(theCameraMatrix, theCameraInfo, theFieldDimensions, 0, theImageCoordinateSystem);
  linePercept.drawIn3D(theFieldDimensions, 0);

  draw();
}

void LinePerceptorRLE2::draw() const
{
  COMPLEX_DRAWING("module:LinePerceptorRLE2:adjacency",
    const int numberOfRuns = theRunLengthImage.beginOfLastLine - pStart;
    for(int i = 0; i < numberOfRuns; ++i)
    {
      const RunLengthImage::Run* const run1 = adjacency[i].run;
      if(run1)
      {
        const RunLengthImage::Run* const run2 = pStart + i;
        LINE("module:LinePerceptorRLE2:adjacency", (run1->xStart + run1->xEnd) / 2, run1->y, (run2->xStart + run2->xEnd) / 2, run2->y, 1, Drawings::ps_solid, ColorClasses::black);
      }
    }
  );
}

LinePerceptorRLE2::LinePerceptorRLE2()
{
  // use malloc because new initializes the elements as well
  adjacency = (Run*)malloc(sizeof(Run) * cameraResolutionHeight * cameraResolutionWidth);
}

LinePerceptorRLE2::~LinePerceptorRLE2()
{
  free(adjacency);
}

void LinePerceptorRLE2::computeAdjacency(const RunLengthImage::Run* const pStart)
{
  const RunLengthImage::Run* p = pStart;
  const RunLengthImage::Run* p2 = p->firstOverlapBelow;
  const RunLengthImage::Run* pEnd = theRunLengthImage.beginOfLastLine;

  const int maxWidthRatioInt = (int)(parameters.maxWidthRatioDifference * (float)(1 << 16));

  const __m64 mZero = _mm_setzero_si64();
  __m64 color1 = _mm_unpacklo_pi8(_mm_cvtsi32_si64(p->color.color), mZero);
  __m64 color2 = _mm_unpacklo_pi8(_mm_cvtsi32_si64(p2->color.color), mZero);

  while(p != pEnd)
  {
    const int currentY = p->y;

    while(p->y == currentY)
    {
      ASSERT(p < pEnd);
      bool valid = false;
      if(abs((1 << 16) - (p->runLength << 16) / p2->runLength) < maxWidthRatioInt)
      {
        __m64 absDifference = _mm_or_si64(_mm_subs_pu16(color1, color2), _mm_subs_pu16(color2, color1));
        if(!_mm_movemask_pi8(_mm_cmpgt_pi16(absDifference, mThreshold)))
        {
          valid = true;
        }
      }

      adjacency[p - pStart] = valid ? p2 : NULL;

      if(valid)
      {
        ++p;
        color1 = _mm_unpacklo_pi8(_mm_cvtsi32_si64(p->color.color), mZero);
        if(p->y == p2->y) // p has been advanced into the next row
        {
          p2 = p->firstOverlapBelow;
          color2 = _mm_unpacklo_pi8(_mm_cvtsi32_si64(p2->color.color), mZero);
        }
        else
        {
          bool changed = false;
          while(p2->xEnd < p->xStart)
          {
            changed = true;
            ++p2;
          }
          if(changed)
          {
            color2 = _mm_unpacklo_pi8(_mm_cvtsi32_si64(p2->color.color), mZero);
          }
        }
      }
      else
      {
        if(p->xEnd < p2->xEnd)
        {
          ++p;
          color1 = _mm_unpacklo_pi8(_mm_cvtsi32_si64(p->color.color), mZero);
        }
        else if(p->xEnd > p2->xEnd)
        {
          ++p2;
          color2 = _mm_unpacklo_pi8(_mm_cvtsi32_si64(p2->color.color), mZero);
        }
        else
        {
          ++p;
          ++p2;
          color1 = _mm_unpacklo_pi8(_mm_cvtsi32_si64(p->color.color), mZero);
          color2 = _mm_unpacklo_pi8(_mm_cvtsi32_si64(p2->color.color), mZero);
        }
      }
    }
  }

  MM_EMPTY();

  // ensure that adjacency contains null-pointers for last line
  for(int i = numberOfRuns; i < theRunLengthImage.image + theRunLengthImage.numberOfRuns - pStart; ++i)
  {
    adjacency[i] = NULL;
  }
}

void LinePerceptorRLE2::buildRegions(vector<Region>& regions)
{
  const ColorConfiguration::ColorDefinition& white = theColorConfiguration.colors[ColorClasses::white];
  const __m64 mZero = _mm_setzero_si64();
  const __m64 mWhite = _mm_unpacklo_pi8(_mm_cvtsi32_si64(white.color.convertToPixel().color), mZero);
  Image::Pixel thresholdPrototype = white.threshold.convertToPixel();
  thresholdPrototype.yCbCrPadding = 255;
  const __m64 mThresholdPrototype = _mm_unpacklo_pi8(_mm_cvtsi32_si64(thresholdPrototype.color), mZero);

  for(int i = 0; i < numberOfRuns; ++i)
  {
    if(!adjacency[i].visited)
    {
      int count = 0;
      const RunLengthImage::Run* p = pStart + i;
      __m64 mAcc = mZero;

      do
      {
        ++count;
        mAcc = _mm_add_pi16(mAcc, _mm_unpacklo_pi8(_mm_cvtsi32_si64(p->color.color), mZero));
        Run& run = adjacency[p - pStart];
        run.visited = true;
        p = run.run;
      }
      while(p);

      if(count >= parameters.minRunsInRegion)
      {
        // compare with color prototype
        const __m64 avgColor = _mm_mulhi_pu16(mAcc, divisionLUT[count - 1]);
        if(!_mm_movemask_pi8((_mm_cmpgt_pi16(_mm_or_si64(_mm_subs_pu16(avgColor, mWhite), _mm_subs_pu16(mWhite, avgColor)), mThresholdPrototype))))
        {
          Region region;
          region.run = pStart + i;
          region.numberOfRuns = count;
          regions.push_back(region);
        }
      }
    }
  }
  MM_EMPTY();
}

void LinePerceptorRLE2::buildRegionsWithColorSplitting(vector<Region>& regions)
{
  const ColorConfiguration::ColorDefinition& white = theColorConfiguration.colors[ColorClasses::white];
  const __m64 mZero = _mm_setzero_si64();
  const __m64 mWhite = _mm_unpacklo_pi8(_mm_cvtsi32_si64(white.color.convertToPixel().color), mZero);
  Image::Pixel thresholdPrototype = white.threshold.convertToPixel();
  thresholdPrototype.yCbCrPadding = 255;
  const __m64 mThresholdPrototype = _mm_unpacklo_pi8(_mm_cvtsi32_si64(thresholdPrototype.color), mZero);

  for(int i = 0; i < numberOfRuns; ++i)
  {
    if(!adjacency[i].visited)
    {
      const RunLengthImage::Run* p = pStart + i;
      __m64 mCurrentColor = _mm_unpacklo_pi8(_mm_cvtsi32_si64(p->color.color), mZero);
      while(p)
      {
        const RunLengthImage::Run* const pStartOfRegion = p;
        int count = 1;
        __m64 mAcc = mCurrentColor;
        Run& run = adjacency[p - pStart];
        run.visited = true;
        p = run.run;

        while(p)
        {
          const __m64 mCount = _mm_shuffle_pi16(_mm_cvtsi32_si64(count), 0x0);
          mCurrentColor = _mm_unpacklo_pi8(_mm_cvtsi32_si64(p->color.color), mZero);
          const __m64 mScaledColor = _mm_mullo_pi16(mCurrentColor, mCount);
          const __m64 mAbsDiff = _mm_or_si64(_mm_subs_pu16(mAcc, mScaledColor), _mm_subs_pu16(mScaledColor, mAcc));
          const __m64 mScaledThreshold = _mm_mullo_pi16(mThreshold, mCount);
          const int mask = _mm_movemask_pi8(_mm_cmpgt_pi16(mAbsDiff, mScaledThreshold));

          if(mask) // end of region, color differs too much
          {
            break;
          }
          else
          {
            ++count;
            mAcc = _mm_add_pi16(mAcc, mCurrentColor);
        
            Run& run = adjacency[p - pStart];
            run.visited = true;
            p = run.run;
          }
        }

        if(count >= parameters.minRunsInRegion)
        {
          // compare with color prototype
          const __m64 avgColor = _mm_mulhi_pu16(mAcc, divisionLUT[count - 1]);
          if(!_mm_movemask_pi8((_mm_cmpgt_pi16(_mm_or_si64(_mm_subs_pu16(avgColor, mWhite), _mm_subs_pu16(mWhite, avgColor)), mThresholdPrototype))))
          {
            Region region;
            region.run = pStartOfRegion;
            region.numberOfRuns = count;
            regions.push_back(region);
          }
        }
      }
    }
  }
  MM_EMPTY();
}

void LinePerceptorRLE2::checkShape(vector<Region>& regions)
{
  // precompute some values to project image lines on field
  const Vector3<>& xAxis3D = theCameraMatrix.rotation.c0;
  const Vector2<> xAxis(Vector2<>(xAxis3D.x, xAxis3D.y).abs(), xAxis3D.z);

  const float distortionA = xAxis.x / theCameraMatrix.translation.z;
  const float distortionB = -(theCameraInfo.opticalCenter.y * xAxis.x + theCameraInfo.focalLength * xAxis.y) / theCameraMatrix.translation.z;

  const float precompA = -theCameraInfo.opticalCenter.x * xAxis.x * theCameraInfo.focalLengthInv;
  const float precompBX = xAxis.x * theCameraInfo.focalLengthInv * 0.5f;
  const float precompBSlope = (xAxis.x * theCameraInfo.opticalCenter.y + xAxis.y * theCameraInfo.focalLength) * theCameraInfo.focalLengthInv * 0.5f;

  const float expectedAWidth = distortionB * theFieldDimensions.fieldLinesWidth;
  const float expectedBWidth = distortionA * theFieldDimensions.fieldLinesWidth;

  Vector2<> vanishingPoint = computeVanishingPoint(2);

  // integrate linearized motion distortion into vanishing point
  vanishingPoint = theImageCoordinateSystem.fromCorrectedLinearized(vanishingPoint);

  for(unsigned int i = 0; i < regions.size(); ++i)
  {
    Region& currentRegion = regions[i];
    const RunLengthImage::Run* p = currentRegion.run;
    currentRegion.startInImage.y = (float)p->y;
    SuccessiveLinearRegressor linReg(*p);
    int counter;
    for(counter = 1; counter < parameters.minRunsInRegion; ++counter)
    {
      p = adjacency[p - pStart].run;
      linReg.addMeasurement(*p);
    }
    int lastY = p->y;
    p = adjacency[p - pStart].run;
    while(counter < currentRegion.numberOfRuns)
    {
      ASSERT(p);
      const int columnInterval = theImageGrid.lineInformationLUT[p->y].columnInterval;
      const int tolerance = 2 + 2 * columnInterval + (int)(abs(linReg.bCenter));
      int predictedCenter;
      int predictedWidth;
      linReg.predictValues(p->y, predictedWidth, predictedCenter);
      if(!(abs(predictedWidth - p->runLength) < tolerance && abs(predictedCenter - (p->xStart + p->xEnd)) < tolerance))
      {
        break;
      }
      linReg.addMeasurement(*p);
      lastY = p->y;
      p = adjacency[p - pStart].run;
      ++counter;
    }
    currentRegion.startInImage.x = (linReg.aCenter + currentRegion.startInImage.y * linReg.bCenter) * 0.5f;

    currentRegion.endInImage.y = (float)lastY;
    currentRegion.endInImage.x = (linReg.aCenter + currentRegion.endInImage.y * linReg.bCenter) * 0.5f;

    const Vector2<> centerInImage = (currentRegion.startInImage + currentRegion.endInImage) * 0.5;
    const float slopeVerticalLine = (centerInImage.x - vanishingPoint.x) / (centerInImage.y - vanishingPoint.y);
    const bool possibleRobot = abs(slopeVerticalLine - linReg.bCenter * 0.5) < parameters.thresholdVerticalLine;

    const float slopeOnField = precompA + precompBX * linReg.aCenter + precompBSlope * linReg.bCenter;
    const float expectedWidthDivisor = 1.0f / sqrt(1.0f + sqr(slopeOnField));
    const float widthAtAvgX = (linReg.aWidth + centerInImage.y * linReg.bWidth) * expectedWidthDivisor;
    const float expectedWidthAtAvgX = expectedAWidth + centerInImage.y * expectedBWidth;
    float widthTolerance = parameters.toleranceWidth + abs(linReg.bCenter) * parameters.toleranceWidthFactor;
    float widthSlopeTolerance = parameters.toleranceWidthSlope + abs(linReg.bCenter) * parameters.toleranceWidthSlopeFactor;
    if(possibleRobot)
    {
      widthTolerance *= parameters.straightPenaltyFactor;
      widthSlopeTolerance *= parameters.straightPenaltyFactor;
      currentRegion.type = Region::PossibleRobot;
    }

    const bool matchingWidth = abs(widthAtAvgX - expectedWidthAtAvgX) < widthTolerance;
    if(matchingWidth)
    {
      currentRegion.type = Region::MatchingWidth;
      const float widthSlopeDifference = abs(linReg.bWidth * expectedWidthDivisor - expectedBWidth);
      if(widthSlopeDifference < widthSlopeTolerance)
      {
        currentRegion.type = possibleRobot ? Region::StraightPossibleRobot : Region::Straight;
      }
      else if(widthSlopeDifference < widthSlopeTolerance * 2.0f)
      {
        currentRegion.type = Region::Circle;
      }
    }

    currentRegion.valid = currentRegion.type >= Region::MatchingWidth;

    int remainingRuns = currentRegion.numberOfRuns - counter;
    currentRegion.numberOfRuns = counter;
    if(remainingRuns >= parameters.minRunsInRegion)
    {
      Region newRegion;
      newRegion.numberOfRuns = remainingRuns;
      newRegion.run = p;
      regions.push_back(newRegion);
    }
  }
}

void LinePerceptorRLE2::checkGreen(vector<Region>& regions)
{
  const ColorConfiguration::ColorDefinition& defGreen = theColorConfiguration.colors[ColorClasses::green];
  const __m64 mZero = _mm_setzero_si64();
  const __m64 mGreen = _mm_unpacklo_pi8(_mm_cvtsi32_si64(defGreen.color.convertToPixel().color), mZero);
  Image::Pixel threshold = defGreen.threshold.convertToPixel();
  threshold.yCbCrPadding = 255;
  const __m64 mThreshold = _mm_unpacklo_pi8(_mm_cvtsi32_si64(threshold.color), mZero);

  for(vector<Region>::iterator i = regions.begin(); i != regions.end(); ++i)
  {
    if(!i->valid)
      continue;

    const RunLengthImage::Run* p = i->run;
    const int step = i->numberOfRuns / 3;
    int matchesLeft = 0, matchesRight = 0;
    ASSERT(step > 0);
    for(int j = 0; j < 2; ++j)
    {
      // advance to one third resp. two thirds of the region
      for(int k = 0; k < step; ++k)
      {
        p = adjacency[p - pStart].run;
      }

      // first search left, then right
      const RunLengthImage::Run* p2 = p;
      int counter = 0;
      while(p2->xStart != 0 && counter < parameters.searchDepthLeftRight)
      {
        --p2;
        counter += p2->runLength;
      }
      if(counter < parameters.searchDepthLeftRight) // there is no neighboring run
      {
        ++matchesLeft;
      }
      else // check if neighboring run is green (enough)
      {
        const __m64 mColor = _mm_unpacklo_pi8(_mm_cvtsi32_si64(p2->color.color), mZero);
        const __m64 absDifferenceGreen = _mm_or_si64(_mm_subs_pu16(mColor, mGreen), _mm_subs_pu16(mGreen, mColor));
        if(!_mm_movemask_pi8(_mm_cmpgt_pi16(absDifferenceGreen, mThreshold)))
        {
          ++matchesLeft;
        }
      }

      p2 = p;
      counter = 0;
      while(p2->xStart != cameraResolutionWidth - 1 && counter < parameters.searchDepthLeftRight)
      {
        ++p2;
        counter += p2->runLength;
      }
      if(counter < parameters.searchDepthLeftRight) // there is no neighboring run
      {
        ++matchesRight;
      }
      else // check if neighboring run is green (enough)
      {
        const __m64 mColor = _mm_unpacklo_pi8(_mm_cvtsi32_si64(p2->color.color), mZero);
        const __m64 absDifferenceGreen = _mm_or_si64(_mm_subs_pu16(mColor, mGreen), _mm_subs_pu16(mGreen, mColor));
        if(!_mm_movemask_pi8(_mm_cmpgt_pi16(absDifferenceGreen, mThreshold)))
        {
          ++matchesRight;
        }
      }
    }
    if(matchesLeft < 1 || matchesRight < 1)
    {
      i->valid = false;
    }
  }

  MM_EMPTY();
}

void LinePerceptorRLE2::findHorizontalSegments(vector<Region>& regions)
{
  const RunLengthImage::Run* p = theRunLengthImage.image;
  const RunLengthImage::Run* const pEnd = p + theRunLengthImage.numberOfRuns;
  while(p != pEnd)
  {
    if(p->runLength > parameters.horizontalSegmentLength)
    {
    }
    ++p;
  }
}

void LinePerceptorRLE2::projectOntoField(vector<Region>& regions) const
{
  const float sqrLengthThreshold = sqr(parameters.thresholdLengthCircleSegment);
  for(vector<Region>::iterator i = regions.begin(); i != regions.end(); ++i)
  {
    if(i->type < Region::Circle)
    {
      continue;
    }
    const Vector2<> startInImageCorrected(theImageCoordinateSystem.toCorrected(i->startInImage));
    const Vector2<> endInImageCorrected(theImageCoordinateSystem.toCorrected(i->endInImage));
    Geometry::calculatePointOnField(startInImageCorrected, theCameraMatrix, theCameraInfo, i->startOnField);
    Geometry::calculatePointOnField(endInImageCorrected, theCameraMatrix, theCameraInfo, i->endOnField);

    i->shortSegment = (i->startOnField - i->endOnField).squareAbs() < sqrLengthThreshold;

    const Vector2<> center = (i->startOnField + i->endOnField) * 0.5f;
    Vector2<> offset = i->endOnField - i->startOnField;
    i->alpha = offset.angle();

    i->dir0 = offset.normalize();

    offset.rotateLeft(); // offset is now the normal vector of the line segment
    i->d = offset * i->startOnField; // see Geometry::getDistanceToLine

    offset *= (float)theFieldDimensions.centerCircleRadius;

    i->circleCenters[0] = center + offset;
    i->circleCenters[1] = center - offset;
  }
}

void LinePerceptorRLE2::mergeRegions(vector<Region>& regions) const
{
  vector<MergingRegion> mergingRegions(regions.size());
  int numOfMergingRegions = 0;
  for(vector<Region>::iterator it = regions.begin(); it != regions.end(); ++it)
  {
    if(!it->valid || it->type < Region::Circle)
      continue;
    MergingRegion& mergingRegion = mergingRegions[numOfMergingRegions++];
    mergingRegion.parent = &mergingRegion;
    mergingRegion.region = &*it;
  }

  MergingRegion* const pBegin = mergingRegions.data();
  MergingRegion* const pEnd = pBegin + numOfMergingRegions;

  const float cosAlphaThreshold = cos(parameters.thresholdMergingAlpha);

  for(MergingRegion* i = pBegin; i != pEnd; ++i)
  {
    for(MergingRegion* j = i + 1; j != pEnd; ++j)
    {
      if((i->region->type > Region::Circle || j->region->type > Region::Circle)
        && abs(i->region->d - j->region->d) < parameters.thresholdMergingD
        && abs(i->region->dir0 * j->region->dir0) > cosAlphaThreshold)
      {
        unite(i, j);
      }
    }
  }

  int regionCtr = 0;
  vector< vector<Region*> > mergedRegions(numOfMergingRegions);
  for(MergingRegion* i = pBegin; i != pEnd; ++i)
  {
    int region;
    if(i->parent == i) // root, new region
    {
      region = regionCtr++;
      mergedRegions[region].reserve(10);
    }
    else // existing region
    {
      region = i->parent->regionNumber;
    }
    mergedRegions[region].push_back(i->region);
    i->regionNumber = region;
  }

  const float sqrLengthThreshold = sqr(parameters.thresholdLengthCircleSegment);

  for(vector< vector<Region*> >::iterator i = mergedRegions.begin(); i != mergedRegions.begin() + regionCtr; ++i)
  {
    if(i->size() <= 1)
    {
      continue;
    }
    Region *minY = i->front(), *maxY = i->front();
    float coveredY = 0;
    for(vector<Region*>::iterator j = i->begin(); j != i->end(); ++j)
    {
      if((*j)->startInImage.y < minY->startInImage.y)
        minY = *j;
      if((*j)->endInImage.y > maxY->endInImage.y)
        maxY = *j;
      coveredY += (*j)->endInImage.y - (*j)->startInImage.y;
    }

    if(coveredY / (maxY->endInImage.y - minY->startInImage.y) > parameters.minMergingCoverage)
    {
      Region& mergedRegion = *i->front();
      mergedRegion.startInImage = minY->startInImage;
      mergedRegion.endInImage = maxY->endInImage;
      mergedRegion.startOnField = minY->startOnField;
      mergedRegion.endOnField = maxY->endOnField;
      mergedRegion.type = Region::Straight;

      Vector2<> offset = mergedRegion.endOnField - mergedRegion.startOnField;
      mergedRegion.shortSegment = offset.squareAbs() < sqrLengthThreshold;
      mergedRegion.alpha = offset.angle();
      mergedRegion.dir0 = offset.normalize();
      offset.rotateLeft();
      mergedRegion.d = offset * mergedRegion.startOnField;

      for(vector<Region*>::iterator j = i->begin() + 1; j != i->end(); ++j)
      {
        (*j)->valid = false;
      }
    }
  }

  // invalidate short segments
  const float thresholdLengthInImage = sqr(parameters.thresholdLengthInImage);
  for(vector<Region>::iterator i = regions.begin(); i != regions.end(); ++i)
  {
    if(!i->valid)
      continue;
    if(i->type <= Region::Circle || (i->shortSegment && (i->startInImage - i->endInImage).squareAbs() < thresholdLengthInImage))
      i->valid = false;
  }
}

void LinePerceptorRLE2::findCircle(vector<Region>& regions, LinePercept& linePercept) const
{
  const float sqrDistThreshold = sqr(parameters.maxDistCircleCenters);
  vector<CircleRegion> circleRegions(regions.size() * 2);
  int numberOfCircleRegions = 0;
  for(unsigned int i = 0; i < regions.size(); ++i)
  {
    Region& r(regions[i]);
    if(!r.valid || !r.shortSegment || r.type < Region::Circle)
    {
      continue;
    }
    CircleRegion &r1(circleRegions[numberOfCircleRegions]), &r2(circleRegions[numberOfCircleRegions + 1]);
    numberOfCircleRegions += 2;
    r1.parent = &r1;
    r1.region = &r;
    r1.circleCenter = &r.circleCenters[0];
    r2.parent = &r2;
    r2.region = &r;
    r2.circleCenter = &r.circleCenters[1];
  }

  if(numberOfCircleRegions < 2 * parameters.minCircleSegments)
    return;
  
  CircleRegion* const pEnd = circleRegions.data() + numberOfCircleRegions;
  for(CircleRegion* i = circleRegions.data(); i != pEnd; ++i)
  {
    for(CircleRegion* j = i + 1; j != pEnd; ++j)
    {
      if((*(i->circleCenter) - *(j->circleCenter)).squareAbs() < sqrDistThreshold)
      {
        unite(i, j);
      }
    }
  }

  vector<int> counters(circleRegions.size());

  int regionCtr = 0;
  for(CircleRegion* i = circleRegions.data(); i != pEnd; ++i)
  {
    int region;
    if(i->parent == i)
      region = regionCtr++;
    else
      region = i->parent->regionNumber;
    counters[region]++;
    i->regionNumber = region;
  }

  const int* bestRegionPtr = max_element(counters.data(), counters.data() + regionCtr);
  const int bestRegionIndex = bestRegionPtr - counters.data();
  const int bestRegionCount = *bestRegionPtr;

  if(bestRegionCount < parameters.minCircleSegments)
    return;

  Vector2<> center;
  for(CircleRegion* i = circleRegions.data(); i != pEnd; ++i)
  {
    if(i->regionNumber== bestRegionIndex)
    {
      center += *i->circleCenter;
    }
  }
  center /= (float)bestRegionCount;

  // center is now an approximation of the center circle's center
  // based on this center look again for matching segments

  if(parameters.circleRefinement)
  {
    float Mx = 0, My = 0, Mxx = 0, Myy = 0, Mxy = 0, Mz = 0, Mxz = 0, Myz = 0;
    int numOfPoints = 0;

    vector< Vector2<> > circlePoints;
    circlePoints.reserve(bestRegionCount * 2);

    CircleRegion* const pBegin = circleRegions.data();
    for(CircleRegion* i = pBegin; i != pEnd; ++i)
    {
      if((*(i->circleCenter) - center).squareAbs() < sqrDistThreshold)
      {
        i->match = true;
        const RunLengthImage::Run* tempRegions[2]; // two runs from beginning and middle of the region
        const RunLengthImage::Run* p = i->region->run;
        tempRegions[0] = p;
        for(int j = 0; j < i->region->numberOfRuns / 2; ++j)
        {
          p = adjacency[p - pStart].run;
        }
        tempRegions[1] = p;

        for(int j = 0; j < 2; ++j)
        {
          const int x = (i - pBegin) % 2 != 0 ? tempRegions[j]->xStart : tempRegions[j]->xEnd;
          const Vector2<> correctedImage = theImageCoordinateSystem.toCorrected(Vector2<int>(x, tempRegions[j]->y));
          Vector2<> pointOnField;
          Geometry::calculatePointOnField(correctedImage, theCameraMatrix, theCameraInfo, pointOnField);
          circlePoints.push_back(pointOnField);

          const float xx = sqr(pointOnField.x);
          const float yy = sqr(pointOnField.y);
          const float z = xx + yy;
          Mx += pointOnField.x;
          My += pointOnField.y;
          Mxx += xx;
          Myy += yy;
          Mxy += pointOnField.x * pointOnField.y;
          Mz += z;
          Mxz += pointOnField.x * z;
          Myz += pointOnField.y * z;
          ++numOfPoints;
        }
      }
    }

    if(numOfPoints < parameters.minCircleSegments)
      return;

    // Construct and solve matrix (might fail and throw exception).
    // Result will be center and radius of the center circle in robot relative coordinates.
    Matrix<3, 3> M(
      Vector<3>(Mxx, Mxy, Mx),
      Vector<3>(Mxy, Myy, My),
      Vector<3>(Mx, My,  static_cast<float>(numOfPoints)));

    Vector<3> v(-Mxz, -Myz, -Mz);

    Vector<3> BCD;

    if(!M.solve(v, BCD))
      return;

    center = Vector2<>(BCD[0], BCD[1]) * -0.5f;
    const float radicand = center.squareAbs() - BCD[2];
    if(radicand <= 0.0f)
      return;
    const float radius = sqrt(radicand);
    if(abs(radius - (float)(theFieldDimensions.centerCircleRadius - theFieldDimensions.fieldLinesWidth / 2)) > parameters.circleRadiusThreshold)
      return;

    DECLARE_DEBUG_DRAWING("module:LinePerceptorRLE2:circleRefinement", "drawingOnField");
    COMPLEX_DRAWING("module:LinePerceptorRLE2:circleRefinement",
      for(vector< Vector2<> >::const_iterator i = circlePoints.begin(); i != circlePoints.end(); ++i)
      {
        CROSS("module:LinePerceptorRLE2:circleRefinement", i->x, i->y, 30, 2, Drawings::ps_solid, ColorClasses::red);
      }
      CIRCLE("module:LinePerceptorRLE2:circleRefinement", center.x, center.y, radius, 2, Drawings::ps_solid, ColorClasses::black, Drawings::bs_null, ColorClasses::none);
      CROSS("module:LinePerceptorRLE2:circleRefinement", center.x, center.y, 100, 10, Drawings::ps_solid, ColorRGBA(0, 0, 255));
    );

    float error = 0;
    for(vector< Vector2<> >::const_iterator i = circlePoints.begin(); i != circlePoints.end(); ++i)
    {
      error += sqr(radius - (center - *i).abs());
    }
    const float variance = error /= ((float)circlePoints.size() - 1);
    PLOT("module:LinePerceptorRLE2:circleError", variance);
    if(variance > parameters.maxCircleVariance)
      return;

    for(CircleRegion* i = pBegin; i != pEnd; ++i)
    {
      if(i->match)
        i->region->valid = false;
    }
  }
  else
  {
    // invalidate circle segments
    for(CircleRegion* i = circleRegions.data(); i != pEnd; ++i)
    {
      if(i->regionNumber== bestRegionIndex)
      {
        i->region->valid = false;
      }
    }
  }

  linePercept.circle.found = true;
  linePercept.circle.lastSeen = theFrameInfo.time;
  linePercept.circle.pos = Vector2<int>((int)center.x, (int)center.y);
}

void LinePerceptorRLE2::findIntersections(vector<Region>& regions, LinePercept& linePercept) const
{
  const float angleDiffThreshold = sin(parameters.thresholdIntersectionAngle);
  for(vector<Region>::iterator i = regions.begin(); i != regions.end(); ++i)
  {
    if(!i->valid)
      continue;

    const Vector2<>& sI = i->startInImage;
    Vector2<> dI = i->endInImage - sI;
    const float lI = dI.abs();
    dI /= lI;

    for(vector<Region>::iterator j = i + 1; j != regions.end(); ++j)
    {
      if(!j->valid)
        continue;

      if(abs(i->dir0 * j->dir0) < angleDiffThreshold)
      {
        const Vector2<>& sJ = j->startInImage;
        Vector2<> dJ = j->endInImage - sJ;
        const float lJ = dJ.abs();
        dJ /= lJ;

        // solve the linear equation system sI + fI * dI = sJ + fJ * dJ
        const float denominator = -1.0f / (dI.x * dJ.y - dJ.x * dI.y);
        const Vector2<> temp(sJ.y - sI.y, sI.x - sJ.x);
        const float fI = temp * dJ * denominator;
        const float fJ = temp * dI * denominator;

        const Vector2<> intersectionInImage(sI + dI * fI);

        const bool overlapStartI = fI > parameters.minOverlapIntersection;
        const bool overlapEndI = fI < lI - parameters.minOverlapIntersection;
        const bool meetsStartI = fI > -parameters.maxDistanceIntersection;
        const bool meetsEndI = fI < lI + parameters.maxDistanceIntersection;

        const bool overlapStartJ = fJ > parameters.minOverlapIntersection;
        const bool overlapEndJ = fJ < lJ - parameters.minOverlapIntersection;
        const bool meetsStartJ = fJ > -parameters.maxDistanceIntersection;
        const bool meetsEndJ = fJ < lJ + parameters.maxDistanceIntersection;

        enum IntersectionType
        {
          none, contains, starts, ends
        } typeI, typeJ;

        if(overlapStartI && overlapEndI)
          typeI = contains;
        else if(!overlapStartI && meetsStartI)
          typeI = starts;
        else if(!overlapEndI && meetsEndI)
          typeI = ends;
        else
          typeI = none;

        if(overlapStartJ && overlapEndJ)
          typeJ = contains;
        else if(!overlapStartJ && meetsStartJ)
          typeJ = starts;
        else if(!overlapEndJ && meetsEndJ)
          typeJ = ends;
        else
          typeJ = none;

        if(typeI == none || typeJ == none)
        {
          continue;
        }

        LinePercept::Intersection intersection;
        intersection.dir1 = typeI == starts ? i->dir0 : -i->dir0;
        intersection.dir2 = typeJ == starts ? j->dir0 : -j->dir0;

        if(typeI == starts || typeI == ends)
        {
          if(typeJ == starts || typeJ == ends)
            intersection.type = LinePercept::Intersection::L;
          else
            intersection.type = LinePercept::Intersection::T;

        }
        else
        {
          if(typeJ == starts || typeJ == ends)
          {
            intersection.type = LinePercept::Intersection::T;
            swap(intersection.dir1, intersection.dir2);
          }
          else
            intersection.type = LinePercept::Intersection::X;
        }
        Geometry::getIntersectionOfLines(Geometry::Line(i->startOnField, i->dir0), Geometry::Line(j->startOnField, j->dir0), intersection.pos);

        linePercept.intersections.push_back(intersection);
      }
    }
  }

  // check for intersections with center circle
  if(linePercept.circle.found)
  {
    const float radiusFloat = (float)theFieldDimensions.centerCircleRadius;
    const float maxDistCenterCircle = sin(parameters.thresholdIntersectionAngle) * radiusFloat;
    const float sqrRadius = sqr(radiusFloat);
    const Vector2<> circleCenter((float)linePercept.circle.pos.x, (float)linePercept.circle.pos.y);
    for(vector<Region>::iterator i = regions.begin(); i != regions.end(); ++i)
    {
      if(!i->valid)
        continue;
    
      // first check if line is close enough to circle center
      Vector2<> normal(i->dir0);
      normal.rotateRight();
      const float dist = normal * circleCenter - normal * i->startOnField;
      if(abs(dist) < maxDistCenterCircle)
      {
        const Vector2<> &s(i->startOnField), &d(i->dir0);
        const Vector2<> temp = (s - circleCenter);
        const float p_2 = temp * d;
        const float q = temp.squareAbs() - sqrRadius;
        const float radicand = sqr(p_2) - q;
        if(radicand > 0)
        {
          const float radix = sqrt(radicand);
          const float factors[2] = {-p_2 + radix, -p_2 - radix};
          LinePercept::Intersection intersection;
          intersection.dir1 = i->dir0;
          intersection.dir2 = normal;
          intersection.type = LinePercept::Intersection::X;
          const float length = (s - i->endOnField).abs();
          for(int j = 0; j < 2; ++j)
          {
            if(factors[j] > -parameters.minOverlapIntersection && factors[j] < length - parameters.minOverlapIntersection)
            {
              const Vector2<> intersectionFloat = s + d * factors[j];
              intersection.pos = Vector2<int>((int)intersectionFloat.x, (int)intersectionFloat.y);
              linePercept.intersections.push_back(intersection);
            }
          }
        }
      }
    }
  }
}

void LinePerceptorRLE2::createLines(vector<Region>& regions, LinePercept& linePercept) const
{
  for(vector<Region>::const_iterator i = regions.begin(); i != regions.end(); ++i)
  {
    if(!i->valid)
      continue;

    LinePercept::Line line;
    line.alpha = i->alpha;
    line.d = i->d;
    line.dead = false;
    line.first = Vector2<int>((int)(i->startOnField.x + 0.5f), (int)(i->startOnField.y + 0.5f));
    line.last = Vector2<int>((int)(i->endOnField.x + 0.5f), (int)(i->endOnField.y + 0.5f));
    line.midLine = false;
    line.startInImage = i->startInImage;
    line.endInImage = i->endInImage;
    linePercept.lines.push_back(line);
  }
}

Vector2<> LinePerceptorRLE2::computeVanishingPoint(int axis) const
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

LinePerceptorRLE2::SuccessiveLinearRegressor::SuccessiveLinearRegressor(const RunLengthImage::Run& run) : count(1), nextParameterUpdate(2)
{
  const float fY = (float)run.y;
  const float width = (float)run.runLength;
  const float center = (float)(run.xStart + run.xEnd);
  sumX = fY;
  sumXX = fY * fY;
  sumYWidth = width;
  sumXYWidth = fY * width;
  sumYCenter = center;
  sumXYCenter = fY * center;
  bWidth = bCenter = 0;
  aWidth = width;
  aCenter = center;
}

inline void LinePerceptorRLE2::SuccessiveLinearRegressor::addMeasurement(const RunLengthImage::Run& run)
{
  const float center = (float)(run.xStart + run.xEnd);
  const float width = (float)run.runLength;
  const float y = (float)run.y;
  sumX += y;
  sumXX += y * y;
  sumYCenter += center;
  sumYWidth += width;
  sumXYCenter += center * y;
  sumXYWidth += width * y;
  ++count;
  if(count >= nextParameterUpdate)
  {
    computeModelParameters();
    nextParameterUpdate += nextParameterUpdate / 2;
  }
}

inline void LinePerceptorRLE2::SuccessiveLinearRegressor::computeModelParameters()
{
  ASSERT(count > 1);

  const float fCount = (float)count;
  const float denominator = sumXX * fCount - sumX * sumX;
  const float sumX_sumYWidth = sumX * sumYWidth;
  const float sumX_sumYCenter = sumX * sumYCenter;

  bWidth = (sumXYWidth * fCount - sumX_sumYWidth) / denominator;
  bCenter = (sumXYCenter * fCount - sumX_sumYCenter) / denominator;
  const float temp = 1.0f / (fCount);
  aWidth = (sumYWidth - bWidth * sumX) * temp;
  aCenter = (sumYCenter - bCenter * sumX) * temp;
}

inline void LinePerceptorRLE2::SuccessiveLinearRegressor::predictValues(const int x, int& width, int& center) const
{
  const float fX = (float)x;
  width = (int)(aWidth + bWidth * fX + 0.5f);
  center = (int)(aCenter + bCenter * fX + 0.5f);
}
