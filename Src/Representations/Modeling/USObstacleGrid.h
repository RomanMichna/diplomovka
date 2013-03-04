/**
* @file USObstacleGrid.h
*
* Declaration of class USObstacleGrid
*
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
*/

#pragma once

#include "Tools/Math/Pose2D.h"


/**
* @class USObstacleGrid
*
* A class that represents the current state robot's environment as measured by its ultrasound sensors
*/
class USObstacleGrid : public Streamable
{
private:
  /** Streaming function
  * @param in Object for streaming in the one direction
  * @param out Object for streaming in the other direction
  */
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM_REGISTER_FINISH;
  }

public:
  /**
  * @class Cell
  * A cell within the modeled occupancy grid
  */
  class Cell
  {
  public:
    unsigned lastUpdate;                      /** The point of time of the last change of the state*/
    unsigned char state;                              /** The state of the cell, i.e. the number of positive measurements within the last few frames*/
    short cluster;                            /** The cluster a cell belongs to */

    /** Constructor */
    Cell(): lastUpdate(0), state(0), cluster(0)
    {}


  };

  /** Dimensions of the grid */
  enum
  {
    CELL_SIZE   = 60,
    GRID_LENGTH = 46, 
    GRID_SIZE   = GRID_LENGTH * GRID_LENGTH
  };

  Cell cells[GRID_SIZE];          /**< The grid */
  Pose2D gridOffset;              /**< The relative position of the grid's origin. Note that it must be rotated before translation is applied. */
  int cellOccupiedThreshold;      /**< Threshold as configured for grid generation */
  int cellMaxOccupancy;           /**< Threshold as configured for grid generation */

  /** Function for drawing */
  void draw();
  
  Vector2<int> worldToGrid(const Vector2<int>& inWorld) const;
  
  Vector2<int> gridToWorld(const Vector2<int>& inGrid) const;
  
  Vector2<> worldToGrid(const Vector2<>& inWorld) const;
  
  Vector2<> gridToWorld(const Vector2<>& inGrid) const;

  /**
   * Halves all cell states.
   */
  void halveGridState();
  /**
   * Sets the state of all cells back to 0
   */
  void clear();

};
