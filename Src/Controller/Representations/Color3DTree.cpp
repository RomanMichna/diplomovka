/*
 * Color3DTree.cpp
 * Implementation of 3d-tree for color classification.
 */

#include "Color3DTree.h"
#include "Platform/BHAssert.h"

#include <algorithm>
#include <vector>
#include <cstring>
#include <cstdio> // Breakpoint

void Color3DTree::Node::insertSorted(std::deque<Neighbour>& nearestNeighbours, const Neighbour& neighbour, unsigned int maxneighbours) const
{
  std::deque<Neighbour>::iterator it = nearestNeighbours.begin();
  const std::deque<Neighbour>::iterator end = it + maxneighbours;
  ASSERT(end == nearestNeighbours.end());
  int distance = neighbour.getDistance();
  for(; it < end; it++)
    if(it->getDistance() > distance)
    {
      nearestNeighbours.insert(it, neighbour);
      nearestNeighbours.pop_back();
      break;
    }
}

void Color3DTree::Node::mNearestNeighbours(const Vector3<int>& color, unsigned int level,
    unsigned int maxneighbours, std::deque<Neighbour>& nearestNeighbours) const
{
  const int distance = this->color.distance(color);
  const unsigned int component = level % 3;
  const int d = color[component] - this->color.get(component);
  insertSorted(nearestNeighbours, Neighbour(this->color, distance), maxneighbours);
  const int lastDistance = nearestNeighbours.back().getDistance();
  if(d < 0)
  {
    if(below)
    {
      below->mNearestNeighbours(color, level + 1, maxneighbours, nearestNeighbours);
    }
    if(above && (-d) < lastDistance)
    {
      above->mNearestNeighbours(color, level + 1, maxneighbours, nearestNeighbours);
    }
  }
  else
  {
    if(above)
    {
      above->mNearestNeighbours(color, level + 1, maxneighbours, nearestNeighbours);
    }
    if(below && d < lastDistance)
    {
      below->mNearestNeighbours(color, level + 1, maxneighbours, nearestNeighbours);
    }
  }
}

ColorClasses::Color Color3DTree::Node::classify(int y, int u, int v, unsigned int maxdistance, unsigned int maxneighbours)
{
  return classify(Vector3<int>(y, u, v), maxdistance, maxneighbours);
}

ColorClasses::Color Color3DTree::Node::classify(const Vector3<int>& color, unsigned int maxdistance, unsigned int maxneighbours)
{
  std::deque<Neighbour> nearestNeighbours;
  for(unsigned i = 0; i < maxneighbours; i++)
    nearestNeighbours.push_back(Neighbour(ClassifiedColor(ColorClasses::none, 0, 0, 0), maxdistance + 1));

  mNearestNeighbours(color, 0, maxneighbours, nearestNeighbours);

  unsigned int histogram[ColorClasses::numOfColors];
  //I have absolute no idea why, but memset dies not always set the histogram to 0
  //memset(histogram, 0, ColorClasses::numOfColors);
  for(int i = 0; i < ColorClasses::numOfColors; ++i)
    histogram[i] = 0;
  std::deque<Neighbour>::const_iterator it = nearestNeighbours.begin();
  std::deque<Neighbour>::const_iterator end = nearestNeighbours.end();
  for(; it < end; it++)
  {
    const ClassifiedColor& cc = it->getColor();
    unsigned idx = cc.getColorClass();
    histogram[idx] += 1;
    if(histogram[idx] > maxneighbours / 2)
      return cc.getColorClass();
  }
  // Color could not be classified. Classify into none.
  return ColorClasses::none;
}

void Color3DTree::build(std::vector<ClassifiedColor>& data)
{
  root = build3DTree(data, 0, 0, data.size() - 1);
}

ColorClasses::Color Color3DTree::classify(int y, int u, int v)
{
  if(root != NULL)
    return root->classify(y, u, v, maxdistance, maxneighbours);
  return ColorClasses::none;
}

Color3DTree::Node* Color3DTree::build3DTree(std::vector<ClassifiedColor>& data, unsigned char level,
    int low, int high)
{
  if(low > high)
    return NULL;
  ClassifiedColorComparator comparator(level);
  std::vector<ClassifiedColor>::iterator base = data.begin();
  sort(base + low, base + high + 1, comparator);
  ASSERT(base + high + 1 <= data.end());
  unsigned int median = (low + high) / 2;
  const ClassifiedColor color = data[median];
  Node* below = build3DTree(data, level + 1, low, median - 1);
  Node* above = build3DTree(data, level + 1, median + 1, high);

  return new Node(color, above, below);
}

Color3DTree::Node& Color3DTree::Node::operator=(const Color3DTree::Node& other)
{
  color = other.color;
  above = other.above;
  below = other.below;
  return *this;
}

void Color3DTree::writeToColorTable(ColorTable64& colorTable)
{
  for(int y = 0; y < 64; y++)
    for(int u = 0; u < 64; u++)
      for(int v = 0; v < 64; v++)
        colorTable.colorClasses[y][u][v] = classify(y * 4, u * 4, v * 4);
}

Color3DTree::Node::~Node()
{
  if(above)
  {
    delete above;
    above = NULL;
  }
  if(below)
  {
    delete below;
    below = NULL;
  }
}
