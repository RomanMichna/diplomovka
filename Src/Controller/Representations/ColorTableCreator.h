/*
 * ColorTableCreator.h
 *
 *  Created on: 06.02.2011
 *      Author: moe
 */

#pragma once

#include "Representations/Configuration/ColorTable64.h"
#include "Tools/Math/Vector3.h"
#include "Tools/MessageQueue/InMessage.h"
#include "Controller/Representations/ClassifiedColor.h"

class ColorTableCreator
{
public:
  class HistListNode
  {
  public:
    unsigned int* value;
    HistListNode(unsigned int* value): value(value) {}
    inline bool operator<(const HistListNode& other)
    {
      return *(this->value) < *(other.value);
    }
  };
  class ColorCube
  {
  public:
    ColorCube()
    {
      yMin = yMax = cbMin = cbMax = crMin = crMax = 0;
    }
    unsigned int yMin,
                 yMax,
                 cbMin,
                 cbMax,
                 crMin,
                 crMax;
    bool checkMembership(unsigned int y, unsigned int cb, unsigned int cr)
    {
      if(y <= yMax && y >= yMin &&
         cb <= cbMax && cb >= cbMin &&
         cr <= crMax && cr >= crMin)
        return true;
      return false;
    }

    void add(unsigned int y, unsigned int cb, unsigned int cr)
    {
      float maxFactor, minFactor;
      maxFactor = 0.9f;
      minFactor = 1.1f;

      if(y * minFactor < yMin)
        yMin = (unsigned int)(y * minFactor);
      else if(y * maxFactor > yMax)
        yMax = (unsigned int)(y * maxFactor);

      if(cb * minFactor  < cbMin)
        cbMin = (unsigned int)(cb * minFactor);
      else if(cb * maxFactor > cbMax)
        cbMax = (unsigned int)(cb * maxFactor);

      if(cr * minFactor  < crMin)
        crMin = (unsigned int)(cr * minFactor);
      else if(cr * maxFactor > crMax)
        crMax = (unsigned int)(cr * maxFactor);
    }

    ColorCube& operator=(ColorCube cc)
    {
      yMin = cc.yMin;
      yMax = cc.yMax;
      cbMin = cc.cbMin;
      cbMax = cc.cbMax;
      crMin = cc.crMin;
      crMax = cc.crMax;
      return *this;
    }
  };

  class ColoredSegment
  {
  public:
    Vector3<unsigned int> avgColor;
    int yStart, yEnd, x;
    ColorClasses::Color colorClass;
    ColoredSegment(Vector3<unsigned int> avgColor,
                   int yStart,
                   int yEnd,
                   int x,
                   ColorClasses::Color colorClass):
      avgColor(avgColor),
      yStart(yStart),
      yEnd(yEnd),
      x(x),
      colorClass(colorClass)
    {}

    ColoredSegment& operator=(const ColoredSegment& other)
    {
      avgColor = other.avgColor;
      yStart = other.yStart;
      yEnd = other.yEnd;
      x = other.x;
      colorClass = other.colorClass;
      return *this;
    }

    int length() { return yEnd - yStart; }
  };

  typedef std::vector<Image::Pixel > Segment;

  ColorTableCreator();
  virtual ~ColorTableCreator();

  void createColorTable(const Image& image, ColorTable64& colorTable);
  void calculateStage1Colors(const Image& image);
  void scanImage(const Image& image, std::vector<ColorTableCreator::Segment>& segments);
  int scanDown(const Image& image, std::vector<ColorTableCreator::Segment>& segments, int x, int y);
  bool checkDiff(Image::Pixel p1, Image::Pixel p2);
  bool checkDiff(Image::Pixel p1, Image::Pixel p2, int diff);
  void getAverageColor(const Image& image, Vector3<unsigned int>& avgColor, Vector3<unsigned int>& halfColor, unsigned int histogram[64][64][64]);
  void insertInColorTable(ColorTable64& colorTable, ColorTableCreator::ColorCube colorCube, ColorClasses::Color color);

  void buildStage3Colors(std::vector<ClassifiedColor>& trainingVector,
                         const Image& image,
                         unsigned int histogram[64][64][64],
                         unsigned int histThreshold,
                         int horizon);
  int buildStage3ColorSegment(int y, int x,  const Image& image, ColoredSegment& viewedSegment);
  void buildColorTable(ColorTable64& colorTable);
  void buildFinalColorTable(ColorTable64& colorTable);
  void addColorByHand(const Image::Pixel& pixel, const ColorClasses::Color& colorClass);
  void undoLastCalculation(ColorTable64& colorTable)
  {
    if(!iterations.empty())
      iterations.pop_back();
    buildColorTable(colorTable);
  };
  void undoLastHandData(ColorTable64& colorTable)
  {
    if(!handMadeClassification.empty())
      handMadeClassification.pop_back();
    buildColorTable(colorTable);
  }
  void resetHandData()
  {
    handMadeClassification.clear();
  }
  void resetAutoData()
  {
    iterations.clear();
  }
  bool saveTrainingData(string fileName) const;
  bool loadTrainingData(string fileName);
  bool empty() const { return handMadeClassification.empty() && iterations.empty(); }
  void removeHandColor(ColorClasses::Color color);
  void removeAutoColor(ColorClasses::Color color);
  void removeColor(std::vector<ClassifiedColor>& colors, ColorClasses::Color);
  void replaceColor(ColorClasses::Color search, ColorClasses::Color replaceBy);
  static bool checkColor(ClassifiedColor color);

  static bool debugblocks;
  int treeRange,
      treeNeighbors;
private:
  static ColorClasses::Color eraseColor;
  int imageCounter,
      maxDiff,
      avgY,
      avgCnt,
      historyIndex,
      historySize;

  Vector3<int> averageColor;
  float* yHistory;
  ColorCube stage1[2];
  ColorCube stage2[6];
  std::vector<std::vector<ClassifiedColor> > iterations;
  std::vector<ClassifiedColor> handMadeClassification;

  ENUM(Color,
    white,
    green,
    blue,
    yellow,
    orange,
    red
  );
};
