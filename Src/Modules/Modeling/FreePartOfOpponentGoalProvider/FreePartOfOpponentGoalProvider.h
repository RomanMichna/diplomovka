/**
 * @author Carsten Koenemann
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Modeling/FreePartOfOpponentGoalModel.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotsModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Configuration/FieldDimensions.h"

MODULE(FreePartOfOpponentGoalProvider)
  // required for calculation of free part
  REQUIRES(RobotsModel)
  REQUIRES(RobotPose)
  REQUIRES(FieldDimensions)
  REQUIRES(BallModel)
  // required only for debugging
  REQUIRES(CameraMatrix)
  REQUIRES(CameraInfo)
  // provided model
  PROVIDES_WITH_MODIFY_AND_DRAW(FreePartOfOpponentGoalModel)
END_MODULE

/** if a robot occupies a cell on the opponent groundline this cells value will get increased by this */
static const float RENEW_VALUE = 1.0f;
/** max value a cell on the opponent groundline can have */
static const float MAX_VALUE = 255.0f;
/** every iteration all cells on on the opponent groundline will get multiplied by this */
static const float AGING_FACTOR = 0.85f;
/** a cell on the opponent groundline is free if its value i lower than this */
static const float IS_FREE_THRESHOLD = 0.2f;
/** robot width in mm */
static const float ROBOT_WIDTH = 200.0f;
/** a free part must at least consist of this many cells */
static const int   MIN_LARGEST_SIZE = 2;
/** if already a free part was found the next one has to be at least this many cells bigger */
static const int   NEW_MAX_FREE_THRESHOLD = 2;

class FreePartOfOpponentGoalProvider : public FreePartOfOpponentGoalProviderBase
{
public:
  FreePartOfOpponentGoalProvider()
  {
    // initially none of the cells on the opponent ground line is occupied
    for(int i = 0; i < NUM_OF_CELLS; i++)
    {
      cellsOnOppGoalline[i] = 0.0f;
    }
    //
    lastFreePartHigh = NUM_OF_CELLS;
    lastFreePartLow = 0;
  }

  void update(FreePartOfOpponentGoalModel& freePartOfOpponentGoalModel);
  void drawingOnField(FreePartOfOpponentGoalModel& freePartOfOpponentGoalModel);

private:
  /** goal line of opponent goal will be divided into this many cell */
  static const int NUM_OF_CELLS = 14;

  /** this array holds values (likeliness a robot is standing there) for each cell on the opponent goal */
  float cellsOnOppGoalline[NUM_OF_CELLS];

  /** highest (left side when facing the opponent goal) cell of the largest free part of opponent goal */
  int largestFreePartHigh;

  /** lowest (right side when facing the opponent goal) cell of the largest free part of opponent goal */
  int largestFreePartLow;

  /** high cell of largest free part found in last iteration */
  int lastFreePartHigh;

  /** low cell of largest free part found in last iteration */
  int lastFreePartLow;
};
