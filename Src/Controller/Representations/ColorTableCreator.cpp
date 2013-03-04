/*
 * ColorTableCreator.cpp
 *
 *  Created on: 06.02.2011
 *      Author: moe
 */

#include "ColorTableCreator.h"
#include "Platform/BHAssert.h"
#include "Color3DTree.h"
#include "Tools/Configuration/ConfigMap.h"
#include <iostream>
#include <algorithm>
#include <list>


ColorClasses::Color ColorTableCreator::eraseColor = ColorClasses::none;
bool ColorTableCreator::debugblocks = false;

ColorTableCreator::ColorTableCreator()
  : treeRange(20),
    treeNeighbors(3),
    imageCounter(0),
    maxDiff(10),
    avgY(0),
    avgCnt(0),
    historyIndex(0),
    historySize(10),
    averageColor(0, 0, 0),
    yHistory(new float[historySize])
{

}

ColorTableCreator::~ColorTableCreator()
{
  delete [] yHistory;
}

void ColorTableCreator::createColorTable(const Image& image, ColorTable64& colorTable)
{

  
  std::vector<ColorTableCreator::Segment> segments;
  Vector3<unsigned int> avgColor(0, 0, 0);
  Vector3<unsigned int> halfColor(0, 0, 0);
  unsigned int histogram[64][64][64];
  for(int y = 0; y < 64; ++y)
    for(int cb = 0; cb < 64; ++cb)
      for(int cr = 0; cr < 64; ++cr)
        histogram[y][cb][cr] = 0;
  getAverageColor(image, avgColor, halfColor, histogram);

  std::list<HistListNode> histList;
  for(int y = 0; y < 64; ++y)
    for(int cb = 0; cb < 64; ++cb)
      for(int cr = 0; cr < 64; ++cr)
        if(!histogram[y][cb][cr])
          histList.push_back(HistListNode(&(histogram[y][cb][cr])));

  histList.sort();
  int topEnd = (int)(histList.size() * 0.95f);
  std::list<HistListNode>::iterator itEnd = histList.begin();
  for(int i = 0; i < topEnd; ++i)
    itEnd++;

  int histThreshold = *(itEnd->value);

  stage1[white].yMax = 255;
  stage1[white].yMin = (unsigned int)min(avgColor.x * 2, (avgColor.x + stage1[white].yMax * 2) / 3);
  stage1[white].cbMin = stage1[white].crMin = 0;
  stage1[white].cbMax = stage1[white].crMax = stage1[white].yMax;

  stage1[green].yMin = (unsigned int)(halfColor.x * 0.8f);
  stage1[green].yMax = (unsigned int)(halfColor.x * 1.2f);
  stage1[green].cbMin = (unsigned int)(halfColor.y * 0.9f);
  stage1[green].cbMax = (unsigned int)(halfColor.y * 1.1f);
  stage1[green].crMin = (unsigned int)(halfColor.z * 0.9f);
  stage1[green].crMax = (unsigned int)(halfColor.z * 1.1f);

  stage2[green] = stage1[green];
  if(stage2[green].yMax >= stage1[white].yMin * 0.8)
    stage2[green].yMax = (unsigned int)(stage1[white].yMin * 0.8);
  scanImage(image, segments);

  ASSERT(stage2[green].yMin < 256);
  ASSERT(stage2[green].yMax < 256);
  ASSERT(stage2[green].cbMin < 256);
  ASSERT(stage2[green].cbMax < 256);
  ASSERT(stage2[green].crMin < 256);
  ASSERT(stage2[green].crMax < 256);

  stage2[blue].yMin = 0;
  stage2[blue].yMax = stage1[white].yMin;
  stage2[blue].cbMin = (unsigned int)(stage2[green].cbMax * 1.05f);
  stage2[blue].cbMax = 255;
  stage2[blue].crMin = 0;
  stage2[blue].crMax = (stage2[green].crMin + stage2[green].crMax * 2) / 3;

  stage2[yellow].yMin = 0;
  stage2[yellow].yMax = (unsigned int)(stage1[white].yMin * 0.9);
  stage2[yellow].cbMin = 0;
  stage2[yellow].cbMax = stage2[green].cbMin;
  stage2[yellow].crMin = stage2[green].crMax;
  stage2[yellow].crMax = (unsigned int)(stage1[white].yMax * 0.95f);

  stage2[orange].yMin = 0;
  stage2[orange].yMax = (unsigned int)(stage1[white].yMin * 0.8);
  stage2[orange].cbMin = 0;
  stage2[orange].cbMax = (stage2[green].cbMin + 2 * stage2[green].cbMax) / 3;
  stage2[orange].crMin = (unsigned int)(stage2[green].crMax * 1.1);
  stage2[orange].crMax = stage1[white].yMax;

  stage2[red].yMin = stage2[green].yMax / 2;
  stage2[red].yMax = (unsigned int)(stage1[white].yMax * 0.9f);
  stage2[red].cbMin = stage2[green].cbMin;
  stage2[red].cbMax = stage2[green].cbMax;
  stage2[red].crMin = stage2[orange].crMin;
  stage2[red].crMax = 255;
  ASSERT(stage2[red].yMin < 256);
  ASSERT(stage2[red].yMax < 256);
  ASSERT(stage2[red].cbMin < 256);
  ASSERT(stage2[red].cbMax < 256);
  ASSERT(stage2[red].crMin < 256);
  ASSERT(stage2[red].crMax < 256);


  if(!debugblocks)
  {
    std::vector<ClassifiedColor> trainingVector;
    buildStage3Colors(trainingVector, image, histogram, histThreshold, 80);
    iterations.push_back(trainingVector);
    buildColorTable(colorTable);
  }
  else
  {
    insertInColorTable(colorTable, stage2[red], ColorClasses::red);
    insertInColorTable(colorTable, stage2[orange], ColorClasses::orange);
    insertInColorTable(colorTable, stage2[yellow], ColorClasses::yellow);
    insertInColorTable(colorTable, stage2[blue], ColorClasses::blue);
    insertInColorTable(colorTable, stage2[green], ColorClasses::green);
    insertInColorTable(colorTable, stage1[white], ColorClasses::white);
  }
}

void ColorTableCreator::insertInColorTable(ColorTable64& colorTable, ColorTableCreator::ColorCube colorCube, ColorClasses::Color color)
{
  for(unsigned int y = colorCube.yMin; y <= colorCube.yMax; y += 4)
    for(unsigned int cb = colorCube.cbMin; cb <= colorCube.cbMax; cb += 4)
      for(unsigned int cr = colorCube.crMin; cr <= colorCube.crMax; cr += 4)
        colorTable.colorClasses[y / 4][cb / 4][cr / 4] = color;
}
void ColorTableCreator::getAverageColor(const Image& image, Vector3<unsigned int>& avgColor, Vector3<unsigned int>& halfColor, unsigned int histogram[64][64][64])
{
  Vector3<unsigned int> last(0, 0, 0);
  unsigned int avgCnt = 0;
  unsigned int halfCnt = 0;
  for(int y = 0; y < image.resolutionHeight; y += 5)
  {
    for(int x = 0; x < image.resolutionWidth; x += 5)
    {

      last = avgColor;
      Image::Pixel pixel = image.image[y][x];
      avgColor.x += pixel.y;
      avgColor.y += pixel.cb;
      avgColor.z += pixel.cr;
      if(y > image.resolutionHeight / 2)
      {
        halfColor.x += pixel.y;
        halfColor.y += pixel.cb;
        halfColor.z += pixel.cr;
        ++halfCnt;
      }
      histogram[pixel.y / 4][pixel.cb / 4][pixel.cr / 4] += 1;
      ASSERT(avgColor.x >= last.x);
      ASSERT(avgColor.y >= last.y);
      ASSERT(avgColor.z >= last.z);
      ++avgCnt;
    }
  }
  avgColor /= avgCnt;
  halfColor /= halfCnt;
}

void ColorTableCreator::scanImage(const Image& image, std::vector<ColorTableCreator::Segment>& segments)
{
  for(int x = 0; x < image.resolutionWidth; x += 5)
  {
    for(int y = 0; y < image.resolutionHeight; y = scanDown(image, segments, x, y));
  }
}

int ColorTableCreator::scanDown(const Image& image, std::vector<ColorTableCreator::Segment>& segments, int x, int y)
{
  ColorTableCreator::Segment segment;
  Image::Pixel lastPixel = image.image[y++][x];
  segment.push_back(lastPixel);
  Vector3<unsigned int> avgColor(0, 0, 0);

  for(; y < image.resolutionHeight; ++y)
  {
    Image::Pixel viewedPixel = image.image[y][x];
    if(checkDiff(lastPixel, viewedPixel))
      break;

    segment.push_back(viewedPixel);
    avgColor.x += lastPixel.y;
    avgColor.y += lastPixel.cb;
    avgColor.z += lastPixel.cr;
  }

  if(segment.size() < 30)
    return y;

  avgColor /= segment.size();
  if(stage1[green].checkMembership(avgColor.x, avgColor.y, avgColor.z))
    for(Segment::iterator segIt = segment.begin();
        segIt < segment.end();
        ++segIt)
      stage2[green].add(segIt->y, segIt->cb, segIt->cr);

  segments.push_back(segment);
  return y;
}

bool ColorTableCreator::checkDiff(Image::Pixel p1, Image::Pixel p2)
{
  return checkDiff(p1, p2, maxDiff);
}

bool ColorTableCreator::checkDiff(Image::Pixel p1, Image::Pixel p2, int diff)
{
  int yDiff = diff;
  int cbDiff = (int)(diff * (1 - (p1.cr / 255.0f)));
  int crDiff = diff;
  if(abs(p1.y - p2.y) > yDiff)
    return false;
  else if(abs(p1.cb - p2.cb) > cbDiff)
    return false;
  else if(abs(p1.cr - p2.cr) > crDiff)
    return false;
  return true;
}

void ColorTableCreator::buildStage3Colors(std::vector<ClassifiedColor>& trainingVector,
    const Image& image,
    unsigned int histogram[64][64][64],
    unsigned int histThreshold,
    int horizon)
{
  for(int x = 0; x < image.resolutionWidth; x += 5)
  {
    ColoredSegment lastSegment(Vector3<unsigned int>(0, 0, 0), 0, 0, 0, ColorClasses::none);
    ColoredSegment viewedSegment(Vector3<unsigned int>(0, 0, 0), 0, 0, 0, ColorClasses::none);
    for(int y = 0; y < image.resolutionHeight; y++)
    {
      bool add = false;
      lastSegment = viewedSegment;
      y = buildStage3ColorSegment(y, x, image, viewedSegment);
      switch(viewedSegment.colorClass)
      {
      case ColorClasses::green:
        if(viewedSegment.yStart > horizon &&
           viewedSegment.length() > 30 &&
           histogram[viewedSegment.avgColor.x / 4][viewedSegment.avgColor.y / 4][viewedSegment.avgColor.z / 4] >= histThreshold)
        {
          add = true;
        }
        break;
      case ColorClasses::yellow:
      case ColorClasses::blue:
        if(viewedSegment.length() > 20 &&
           viewedSegment.yStart < horizon &&
           viewedSegment.yEnd > horizon)
        {
          add = true;
        }
        break;
      case ColorClasses::white:
        if(viewedSegment.length() < 10 &&
           viewedSegment.yStart > horizon)
        {
          add = true;
        }
        break;
      case ColorClasses::orange:
        if(viewedSegment.length() < 30 &&
           viewedSegment.yStart > horizon)
        {
          add = true;
        }
      default:
        break;
      }
      if(add)
        trainingVector.push_back(ClassifiedColor(viewedSegment.colorClass, viewedSegment.avgColor.x, viewedSegment.avgColor.y, viewedSegment.avgColor.z));
    }
  }
}

int ColorTableCreator::buildStage3ColorSegment(int y, int x,  const Image& image, ColoredSegment& viewedSegment)
{
  Image::Pixel lastPixel = image.image[y][x];
  int yStart = y++;
  int yEnd = image.resolutionHeight;
  int skip = 0;
  int maxSkip = 5;
  int cnt = 0;
  Vector3<unsigned int> avgColor(0, 0, 0);
  for(; y < yEnd && skip < maxSkip; ++y)
  {
    Image::Pixel viewedPixel = image.image[y][x];
    if(checkDiff(lastPixel, viewedPixel, 40))
    {
      avgColor.x += viewedPixel.y;
      avgColor.y += viewedPixel.cb;
      avgColor.z += viewedPixel.cr;
      ++cnt;
      lastPixel = viewedPixel;
      if(skip)
        skip = 0;
    }
    else
      ++skip;
  }
  if(skip)
    y -= skip;

  avgColor /= cnt;
  viewedSegment.avgColor = avgColor;
  viewedSegment.x = x;
  viewedSegment.yStart = yStart;
  viewedSegment.yEnd = y;
  if(stage1[white].checkMembership(avgColor.x, avgColor.y, avgColor.z))
  {
    viewedSegment.colorClass = ColorClasses::white;
  }
  else if(stage2[green].checkMembership(avgColor.x, avgColor.y, avgColor.z))
    viewedSegment.colorClass = ColorClasses::green;
  else if(stage2[blue].checkMembership(avgColor.x, avgColor.y, avgColor.z))
    viewedSegment.colorClass = ColorClasses::blue;
  else if(stage2[yellow].checkMembership(avgColor.x, avgColor.y, avgColor.z))
    viewedSegment.colorClass = ColorClasses::yellow;
  else if(stage2[orange].checkMembership(avgColor.x, avgColor.y, avgColor.z))
    viewedSegment.colorClass = ColorClasses::orange;
  else
    viewedSegment.colorClass = ColorClasses::none;
  return y;

}

void ColorTableCreator::buildColorTable(ColorTable64& colorTable)
{
  colorTable.clear();
  for(unsigned int k = 0; k < iterations.size(); ++k)
  {
    std::vector<ClassifiedColor> trainingVector = iterations[k];
    for(unsigned int i = 0; i < trainingVector.size(); ++i)
    {
      ClassifiedColor sample = trainingVector[i];
      colorTable.colorClasses[sample.getY() / 4][sample.getU() / 4][sample.getV() / 4] = sample.getColorClass();
    }
  }
  for(unsigned int i = 0; i < handMadeClassification.size(); ++i)
  {
    ClassifiedColor sample = handMadeClassification[i];
    colorTable.colorClasses[sample.getY() / 4][sample.getU() / 4][sample.getV() / 4] = sample.getColorClass();
  }
}

void ColorTableCreator::buildFinalColorTable(ColorTable64& colorTable)
{
  std::vector<ClassifiedColor> trainingVector;
  for(unsigned int i = 0; i < iterations.size(); ++i)
  {
    std::vector<ClassifiedColor> iteration = iterations[i];
    for(unsigned int k = 0; k < iteration.size(); ++k)
    {
      trainingVector.push_back(iteration[k]);
    }
  }

  for(unsigned int k = 0; k < handMadeClassification.size(); ++k)
  {
    trainingVector.push_back(handMadeClassification[k]);
  }

  colorTable.clear();
  Color3DTree tree(treeRange, treeNeighbors);
  tree.build(trainingVector);
  tree.writeToColorTable(colorTable);
}

void ColorTableCreator::addColorByHand(const Image::Pixel& pixel, const ColorClasses::Color& colorClass)
{
  handMadeClassification.push_back(ClassifiedColor(colorClass, pixel.y, pixel.cb, pixel.cr));
}

bool ColorTableCreator::saveTrainingData(string fileName) const
{
  ConfigMap cm;
  cm["handMadeClassification"] << handMadeClassification;
  cm["iterations"] << iterations;
  ofstream of;
  of.open(fileName.c_str());
  if(!of.is_open())
    return false;
  cm.write(of);
  of.close();
  return true;
}

bool ColorTableCreator::loadTrainingData(string fileName)
{
  handMadeClassification.clear();
  iterations.clear();
  ConfigMap cm;
  if(cm.read(fileName) < 0)
    return false;

  int failCount = 0;
  try
  {
    cm["handMadeClassification"] >> handMadeClassification;
  }
  catch(std::invalid_argument e)
  {
    failCount++;
  } //In this case the Vector was empty

  try
  {
    cm["iterations"] >> iterations;
  }
  catch(std::invalid_argument e)
  {
    failCount++;
  } //In this case the Vector was empty
  return failCount < 2;
}

void ColorTableCreator::removeColor(std::vector<ClassifiedColor>& colors, ColorClasses::Color eColor)
{
  eraseColor = eColor;
  std::vector<ClassifiedColor>::iterator end = colors.end();
  std::vector<ClassifiedColor>::iterator newEnd =
    std::remove_if(colors.begin(), end, ColorTableCreator::checkColor);
  colors.erase(newEnd, end);
}

bool ColorTableCreator::checkColor(ClassifiedColor color)
{
  return color.getColorClass() == eraseColor;
}

void ColorTableCreator::removeHandColor(ColorClasses::Color color)
{
  removeColor(handMadeClassification, color);
}
void ColorTableCreator::removeAutoColor(ColorClasses::Color color)
{
  for(std::vector<std::vector<ClassifiedColor> >::iterator it = iterations.begin();
      it < iterations.end();
      ++it)
    removeColor(*it, color);
}

void ColorTableCreator::replaceColor(ColorClasses::Color search, ColorClasses::Color replaceBy)
{
  for(std::vector<std::vector<ClassifiedColor> >::iterator it = iterations.begin();
      it < iterations.end();
      ++it)
  {
    for(std::vector<ClassifiedColor>::iterator innerIt = it->begin();
        innerIt > it->end();
        ++innerIt)
    {
      if(innerIt->getColorClass() == search)
      {
        (*innerIt) = ClassifiedColor(replaceBy, innerIt->getY(), innerIt->getU(), innerIt->getV());
      }
    }
  }

  for(std::vector<ClassifiedColor>::iterator handIt = handMadeClassification.begin();
      handIt < handMadeClassification.end();
      ++handIt)
  {
    if(handIt->getColorClass() == search)
    {
      (*handIt) = ClassifiedColor(replaceBy, handIt->getY(), handIt->getU(), handIt->getV());
    }
  }
}
