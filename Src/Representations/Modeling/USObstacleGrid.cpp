/**
* @file USObstacleGrid.cpp
*
* Implementation of class USObstacleGrid
*
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
*/

#include "USObstacleGrid.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"

void USObstacleGrid::draw()
{
  DECLARE_DEBUG_DRAWING("representation:USObstacleGrid", "drawingOnField",
  {
    unsigned char colorOccupiedStep(255 / cellMaxOccupancy);
    ColorRGBA baseColor(200, 200, 255, 128);
    unsigned char cellsForDrawing[GRID_SIZE];
    for(int i = 0; i < GRID_SIZE; ++i)
      cellsForDrawing[i] = colorOccupiedStep * cells[i].state;
    GRID_MONO("representation:USObstacleGrid", CELL_SIZE, GRID_LENGTH, GRID_LENGTH, baseColor, cellsForDrawing);
    const int gridWidth(GRID_LENGTH* CELL_SIZE);
    const int gridHeight(GRID_LENGTH* CELL_SIZE);
    RECTANGLE("representation:USObstacleGrid", -gridWidth / 2, -gridHeight / 2, gridWidth / 2, gridHeight / 2,
              20, Drawings::ps_solid, ColorRGBA(0, 0, 100));
  });

  DECLARE_DEBUG_DRAWING3D("representation:USObstacleGrid", "robot",
  {
    ROTATE3D("representation:USObstacleGrid", 0.f, 0.f, gridOffset.rotation);
    TRANSLATE3D("representation:USObstacleGrid", gridOffset.translation.x, gridOffset.translation.y, -180.f);
    static const int offset = GRID_LENGTH * (CELL_SIZE - 1) / 2;
    for(int i = 0; i < GRID_SIZE; ++i)
      if(cells[i].state)
      {
        float x = float(i % GRID_LENGTH * CELL_SIZE - offset);
        float y = float(i / GRID_LENGTH * CELL_SIZE - offset);
        POINT3D("representation:USObstacleGrid", x, y, 0, 5, ColorRGBA(0, 0, 0, cells[i].state * 255 / cellMaxOccupancy));
      }
  });
}

Vector2<int> USObstacleGrid::worldToGrid(const Vector2<int>& inWorld) const
{
  Vector2<> inGrid = worldToGrid(Vector2<>((float) inWorld.x, (float) inWorld.y));
  return Vector2<int>((int) inGrid.x, (int) inGrid.y);
}

Vector2<int> USObstacleGrid::gridToWorld(const Vector2<int>& inGrid) const
{
  Vector2<> inWorld = gridToWorld(Vector2<>((float) inGrid.x, (float) inGrid.y));
  return Vector2<int>((int) inWorld.x, (int) inWorld.y);
}

Vector2<> USObstacleGrid::worldToGrid(const Vector2<>& inWorld) const
{
  return (Pose2D(-gridOffset.rotation) * inWorld - gridOffset.translation) / CELL_SIZE + Vector2<>(GRID_LENGTH / 2.f, GRID_LENGTH / 2.f);
}

Vector2<> USObstacleGrid::gridToWorld(const Vector2<>& inGrid) const
{
  return (Pose2D(gridOffset.rotation) * (gridOffset.translation + (inGrid - Vector2<>(GRID_LENGTH / 2.f, GRID_LENGTH / 2.f)) * CELL_SIZE));
}

void USObstacleGrid::halveGridState()
{
  for(int i = 0; i < GRID_SIZE; ++i)
  {
    cells[i].state = (int)((cells[i].state + 0.5) / 2.0f); //int division
  }
}

void USObstacleGrid::clear()
{
  for(int i = 0; i < GRID_SIZE; ++i)
  {
    cells[i].state = 0;
  }
}

