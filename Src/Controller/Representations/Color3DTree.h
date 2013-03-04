/*
 * Color3DTree.h
 * Declaration of a 3d-tree for color classification.
 * @author: Felix Wenk
 */

#pragma once

#include "ClassifiedColor.h"
#include <deque>
#include "Representations/Configuration/ColorTable64.h"

class Color3DTree
{
public:
  class Neighbour
  {
  public:
    Neighbour(ClassifiedColor color, int distance) : color(color), distance(distance) {}
    Neighbour(const Neighbour& other) : color(other.color), distance(other.distance) {}
    Neighbour& operator=(const Neighbour& other) { color = other.color; distance = other.distance; return *this; }
    ClassifiedColor getColor() const { return color; }
    int getDistance() const { return distance; }
  private:
    ClassifiedColor color;
    int distance;
  };

  class Node
  {
  public:
    Node(ClassifiedColor color, const Node* above, const Node* below)
      : color(color), above(above), below(below) {}
    Node(ClassifiedColor color)
      : color(color), above(NULL), below(NULL) {}
    Node(const Node& other);

    /**
     * Destructor which recursively deletes all nodes of the subtree
     * with this node as its root.
     */
    virtual ~Node();
    Node& operator=(const Node& other);
    bool isLeaf() { return above == NULL && below == NULL; }
    ColorClasses::Color classify(int y, int u, int v, unsigned int maxdistance, unsigned int maxneighbours);
    ColorClasses::Color classify(const Vector3<int>& color, unsigned int maxdistance, unsigned int maxneighbours);
  private:
    void mNearestNeighbours(const Vector3<int>& color, unsigned int level, unsigned int maxneighbours, std::deque<Neighbour>& nearestNeighbours) const;
    void insertSorted(std::deque<Neighbour>& nearestNeighbours, const Neighbour& neighbour, unsigned int maxneighbours) const;

    ClassifiedColor color;
    const Node* above;
    const Node* below;
  };

  Color3DTree() : root(NULL), maxdistance(20), maxneighbours(3) {}
  Color3DTree(unsigned maxdistance, unsigned maxneighbours) : root(NULL), maxdistance(maxdistance), maxneighbours(maxneighbours) {}
  virtual ~Color3DTree() { if(root) delete root; }
  bool isEmpty() { return root == NULL; }
  void build(std::vector<ClassifiedColor>& data);
  ColorClasses::Color classify(int y, int u, int v);
  void writeToColorTable(ColorTable64& colorTable);
private:
  /*
   * Builds a 3d-Tree of the elements contained in data. Level is the current depth of the tree,
   * low and high are the lower and upper bounds in the vector of the data to be used. Note that 'high'
   * is inclusive, i.e. denotes the last element to be used, not the element after the last element
   * to be used!
   */
  Node* build3DTree(std::vector<ClassifiedColor>& data, unsigned char level, int low, int high);

  Node* root;
  unsigned int maxdistance;
  unsigned int maxneighbours;
};
