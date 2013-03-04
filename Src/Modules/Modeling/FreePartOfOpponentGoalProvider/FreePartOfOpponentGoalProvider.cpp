/**
 * @author Carsten Koenemann
 */

#include "FreePartOfOpponentGoalProvider.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Geometry.h"

void FreePartOfOpponentGoalProvider::update(FreePartOfOpponentGoalModel& freePartOfOpponentGoalModel)
{
  freePartOfOpponentGoalModel.valid = true;
  /* setup */
  // dimensions of opponent goalline
  Vector2<> leftOppGoalPostAbs  = Vector2<>((float) theFieldDimensions.xPosOpponentGroundline, (float) theFieldDimensions.yPosLeftGoal);
  Vector2<> rightOppGoalPostAbs = Vector2<>((float) theFieldDimensions.xPosOpponentGroundline, (float) theFieldDimensions.yPosRightGoal);
  Range<> oppGoallineRange = Range<>(rightOppGoalPostAbs.y, leftOppGoalPostAbs.y);
  int widthOfOppGoal = theFieldDimensions.yPosLeftGoal - theFieldDimensions.yPosRightGoal;
  float cellOnOppGoallineWidth = (float) widthOfOppGoal / NUM_OF_CELLS;

  /* update cells */
  // renew cells on opponent goal in which a robot is found
  std::vector<RobotsModel::Robot>::const_iterator robotIt;
  for(robotIt = theRobotsModel.robots.begin(); robotIt < theRobotsModel.robots.end(); robotIt++)
  {
    // setup position to the sides of where the robot is standing
    Vector2<> absRobotOnField = Geometry::relative2FieldCoord(theRobotPose, robotIt->relPosOnField);
    Vector2<> leftRobotEnd  = Vector2<>(absRobotOnField.x, absRobotOnField.y + ROBOT_WIDTH / 2);
    Vector2<> rightRobotEnd = Vector2<>(absRobotOnField.x, absRobotOnField.y - ROBOT_WIDTH / 2);
    Vector2<> absBallOnField = Geometry::relative2FieldCoord(theRobotPose, theBallModel.estimate.position);
    // intersection of lines to these ends with opponent groundline
    Vector2<> leftIntersectOppGroundline, rightIntersectOppGroundline;
    if(
      Geometry::getIntersectionOfLines(Geometry::Line(absBallOnField, leftRobotEnd - absBallOnField),  Geometry::Line((float) theFieldDimensions.xPosOpponentGroundline, 0, 0, 1), leftIntersectOppGroundline) &&
      Geometry::getIntersectionOfLines(Geometry::Line(absBallOnField, rightRobotEnd - absBallOnField), Geometry::Line((float) theFieldDimensions.xPosOpponentGroundline, 0, 0, 1), rightIntersectOppGroundline)
    )
    {
      // check if at least one of these intersections is on the inside of the opponent goal
      if(oppGoallineRange.isInside(leftIntersectOppGroundline.y) || oppGoallineRange.isInside(rightIntersectOppGroundline.y))
      {
        // clip both into the inside of the opponent goal, in case one of them is outside of it
        float leftIntersectOppGroundlineY  = oppGoallineRange.limit(leftIntersectOppGroundline.y);
        float rightIntersectOppGroundlineY = oppGoallineRange.limit(rightIntersectOppGroundline.y);
        // check for each cell if it lies within the line between the intersections
        for(int i = 1; i < NUM_OF_CELLS - 1; i++)
        {
          float leftCellEndY  = theFieldDimensions.yPosRightGoal + (i + 1) * cellOnOppGoallineWidth;
          float rightCellEndY = theFieldDimensions.yPosRightGoal + i * cellOnOppGoallineWidth;
          if(
            (Range<>(rightIntersectOppGroundlineY, leftIntersectOppGroundlineY).isInside(leftCellEndY) ||
             Range<>(rightIntersectOppGroundlineY, leftIntersectOppGroundlineY).isInside(rightCellEndY)) &&
            cellsOnOppGoalline[i] < MAX_VALUE
          )
          {
            cellsOnOppGoalline[i] += RENEW_VALUE;
          }
        }
      }
    }
  }
  // left and right ends are always occupied
  cellsOnOppGoalline[0] = MAX_VALUE;
  cellsOnOppGoalline[NUM_OF_CELLS - 1] = MAX_VALUE;
  // aging of unoccupied cells
  for(int i = 1; i < NUM_OF_CELLS - 1; i++) cellsOnOppGoalline[i] *= AGING_FACTOR;

  /* calculate which free part is the largest / best */
  int currentFreePartSize = 0, currentFreePartHigh = 0, currentFreePartLow = 0;
  int largestHithertoSize = -1, largestHithertoHigh = -1, largestHithertoLow = -1;
  int overlapWithLastFreePart = 0;
  int side = (theRobotPose.translation.y < 0) ? 1 : -1;
  for(
    int i = (side > 0) ? 0 : NUM_OF_CELLS - 1;
    (side > 0) ? (i < NUM_OF_CELLS) : (i >= 0);
    i += side
  )
  {
    if(cellsOnOppGoalline[i] < IS_FREE_THRESHOLD)
    {
      currentFreePartSize++;
      if(Range<int>(lastFreePartLow, lastFreePartHigh).isInside(i)) overlapWithLastFreePart++;
      (side > 0) ? (currentFreePartHigh = i) : (currentFreePartLow = i);
    }
    else
    {
      // if current free part is significantly larger than the last found one, define it as the new largest free part so far
      // + add a bonus to the current free part if it significantly overlaps with the last largest free part
      if(((overlapWithLastFreePart >= currentFreePartSize / 2) ? (currentFreePartSize + NEW_MAX_FREE_THRESHOLD) : currentFreePartSize) > (largestHithertoSize + NEW_MAX_FREE_THRESHOLD / 2))
      {
        largestHithertoSize = currentFreePartSize;
        largestHithertoHigh = currentFreePartHigh;
        largestHithertoLow  = currentFreePartLow;
      }
      currentFreePartSize = 0;
      overlapWithLastFreePart = 0;
      (side > 0) ? (currentFreePartLow = i + 1) : (currentFreePartHigh = i - 1);
    }
  }
  // if not even a minimal number of cells is free...
  if(largestHithertoSize >= MIN_LARGEST_SIZE)
  {
    largestFreePartHigh = largestHithertoHigh;
    largestFreePartLow  = largestHithertoLow;
    freePartOfOpponentGoalModel.isFree = true;
  }
  // ...every part of the goal is equally hopeless
  // (free part will be the whole goal, so one can always shoot into the center,
  // but set a flag the goal is completely blocked)
  else
  {
    largestFreePartHigh = NUM_OF_CELLS - 2;
    largestFreePartLow  = 1;
    freePartOfOpponentGoalModel.isFree = false;
  }
  // new found largest free part will be the last found in next update
  lastFreePartHigh = largestFreePartHigh;
  lastFreePartLow  = largestFreePartLow;

  /* caculate relative position to ends of free part */
  Vector2<> leftEndOfFreePartAbs  = Vector2<>(
                                      (float) theFieldDimensions.xPosOpponentGroundline,
                                      theFieldDimensions.yPosRightGoal + (largestFreePartHigh + 1) * cellOnOppGoallineWidth
                                    );
  Vector2<> rightEndOfFreePartAbs = Vector2<>(
                                      (float) theFieldDimensions.xPosOpponentGroundline,
                                      theFieldDimensions.yPosRightGoal + largestFreePartLow * cellOnOppGoallineWidth
                                    );
  freePartOfOpponentGoalModel.leftEnd  = Geometry::fieldCoord2Relative(theRobotPose, leftEndOfFreePartAbs);
  freePartOfOpponentGoalModel.rightEnd = Geometry::fieldCoord2Relative(theRobotPose, rightEndOfFreePartAbs);

  /* draw absolute position of free part into worldstate, see draw()-function below */
  DECLARE_DEBUG_DRAWING("module:FreePartOfOpponentGoalProvider:FreePartOnField", "drawingOnField");
  COMPLEX_DRAWING("module:FreePartOfOpponentGoalProvider:FreePartOnField", drawingOnField(freePartOfOpponentGoalModel));
}


void FreePartOfOpponentGoalProvider::drawingOnField(FreePartOfOpponentGoalModel& freePartOfOpponentGoalModel)
{
  COMPLEX_DRAWING("module:FreePartOfOpponentGoalProvider:FreePartOnField",
  {
    // draw goal line black
    LINE(
      "module:FreePartOfOpponentGoalProvider:FreePartOnField",
      theFieldDimensions.xPosOpponentGroundline,
      theFieldDimensions.yPosRightGoal,
      theFieldDimensions.xPosOpponentGroundline,
      theFieldDimensions.yPosLeftGoal,
      50,
      Drawings::ps_solid,
      ColorClasses::black
    );
    // draw free parts
    int widthOfOppGoal = theFieldDimensions.yPosLeftGoal - theFieldDimensions.yPosRightGoal;
    float cellOnOppGoallineWidth = (float) widthOfOppGoal / NUM_OF_CELLS;
    for(int i = 0; i < NUM_OF_CELLS; i++)
    {
      // draw all free parts blue
      if(cellsOnOppGoalline[i] < IS_FREE_THRESHOLD)
      {
        LINE(
          "module:FreePartOfOpponentGoalProvider:FreePartOnField",
          theFieldDimensions.xPosOpponentGroundline,
          theFieldDimensions.yPosRightGoal + i * cellOnOppGoallineWidth,
          theFieldDimensions.xPosOpponentGroundline,
          theFieldDimensions.yPosRightGoal + (i + 1)*cellOnOppGoallineWidth,
          50,
          Drawings::ps_solid,
          ColorClasses::blue
        );
      }
      // draw largest free part green
      if(Range<int>(largestFreePartLow, largestFreePartHigh).isInside(i))
      {
        LINE(
          "module:FreePartOfOpponentGoalProvider:FreePartOnField",
          theFieldDimensions.xPosOpponentGroundline,
          theFieldDimensions.yPosRightGoal + i * cellOnOppGoallineWidth,
          theFieldDimensions.xPosOpponentGroundline,
          theFieldDimensions.yPosRightGoal + (i + 1)*cellOnOppGoallineWidth,
          30,
          Drawings::ps_solid,
          ColorClasses::green
        );
      }
    }
  });
}


MAKE_MODULE(FreePartOfOpponentGoalProvider, Modeling)
