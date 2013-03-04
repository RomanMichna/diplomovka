/**
 * @file PathFinder.h
 * @author Katharina Gillmann
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Modeling/CombinedWorldModel.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Configuration/FieldDimensions.h"


class PathFinder
{
public:

  /**
  * @class Node
  * a node of a path
  */
  class Node
  {
  public:
    Vector2<> position; //position of the current node
    int indexPreviousNode; // index of the previous node in the path

    Node() : indexPreviousNode(-1) {} // Constructor
    Node(const Vector2<>& position) : position(position), indexPreviousNode(-1) {}
  };

  PathFinder(const RobotPose& robotPose, const CombinedWorldModel& combinedWorldModel, const FieldDimensions& fieldDimensions, 
    const RobotInfo& robotInfo, const GameInfo& gameInfo) :
    theRobotPose(robotPose),
    theCombinedWorldModel(combinedWorldModel),
    theFieldDimensions(fieldDimensions),
    theRobotInfo(robotInfo),
    theGameInfo(gameInfo),
    countNoPathFound(0)
  {
  }

  vector< Vector2<> > path; // current complete path
  float pathLength; // length of the path

  vector<Node> firstTree;
  vector<Node> secondTree;

  void findPath(const Vector2<>& startOfPath, const Vector2<>& endOfPath);
  void loadParameters();


  /**
  * @class Parameters
  * The parameters of the module
  */
  class Parameters: public Streamable
  {
  private:
    /**
    * The method makes the object streamable.
    * @param in The stream from which the object is read
    * @param out The stream to which the object is written
    */
    virtual void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN;
      STREAM(stepSize);
      STREAM(distancePathCombination);
      STREAM(distanceToObstacle);
      STREAM(distanceCloseToObstacle);
      STREAM(targetProbability);
      STREAM(wayPointsProbability);
      STREAM(countOfCycles);
      STREAM(searchSpaceFactor);
      STREAM(counterUseOldPath);
      STREAM(enterPenaltyAreaInReady);
      STREAM_REGISTER_FINISH;
    }

  public:
    float stepSize;                  /**< The length of the new added edge */
    float distancePathCombination;   /**<max distance between two pathes for combining them > */
    float distanceToObstacle;        /**<needed distance between robot and obstacle> */
    float distanceCloseToObstacle;   /**<close distance of positions edge and obstacle> */
    float targetProbability;         /**<probability for using target as random position> */
    float wayPointsProbability;      /**<probability for using old wayPoints as random position> */
    int countOfCycles;               /**<count of cycles for creating one way> */
    float searchSpaceFactor;         /**<factor for the search area in which random positions can be found>*/
    int counterUseOldPath;           /**<counter until old path is used if no new one was found>*/
    bool enterPenaltyAreaInReady;    /**<flag the enables/disables the penalty area obstacle*/

    /** Constructor */
    Parameters(): stepSize(200.0f), distancePathCombination(400.0f), distanceToObstacle(500.0f), distanceCloseToObstacle(200), targetProbability(0.3f), 
      wayPointsProbability(0.7f), countOfCycles(1000), searchSpaceFactor(1.5f), counterUseOldPath(200), enterPenaltyAreaInReady(false)
    {
    }
  };

  Parameters parameters; /**< The parameters of this module */

private:
  const RobotPose& theRobotPose;
  const CombinedWorldModel& theCombinedWorldModel;
  const FieldDimensions& theFieldDimensions;
  const RobotInfo& theRobotInfo;
  const GameInfo& theGameInfo;

  vector<Node> completePath; // the complete path found in the last run
  int countNoPathFound;

  void createRandomPosition(Vector2<>& randomPosition, const vector<Node>& currentNotUsedTree);
  bool checkForObstaclesNearPosition(const vector<Node>& allObstacles, Vector2<>& positionOfObstacle, const Vector2<>& position, float& smallestDistance);
  void calculateNearestNode(const vector<Node>& currentUsedTree, int& indexNearestNode, const Vector2<>& randomPosition);
  bool checkForCollision(const vector<Node>& allObstacles, const Vector2<>& nearestNode, const Vector2<>& randomPosition, bool usePosition, float& distanceToObstacle, Vector2<>& positionObstacle, const Vector2<>& target);
  void checkForFoundPath(const Vector2<>& currentUsedNode, const vector<Node>& currentNotUsedTree, bool& foundPath, bool& foundPathInFirstTree, int& indexOtherTree, const bool useFirstTree);
  void createNewNode(vector<Node>& currentUsedTree, const Vector2<> randomPosition, const int indexNearestNode);
  void createCompletePath(vector<Node>& completePath, const bool foundPathInFirstTree, const vector<Node>& firstTree, const vector<Node>& secondTree, const int indexOtherTree, const bool ObstacleStart, const bool ObstacleEnd, const Vector2<>& oldStart, const Vector2<>& oldEnd);
  void addAvoidingPoints(const vector<Node>& allObstacles, vector<Node>& currentUsedTree, const Vector2<>& position, const Vector2<>& positionOfObstacle, bool nearestObstacle, const Node& target, const bool startIsUsed);

  bool checkForObstaclesNearPositionMMX(const vector<Node>& allObstacles, Vector2<>& positionOfObstacle, const Vector2<>& position, float& smallestDistance);
  void checkForFoundPathMMX(const Vector2<>& currentUsedNode, const vector<Node>& currentNotUsedTree, bool& foundPath, bool& foundPathInFirstTree, int& indexOtherTree, const bool useFirstTree);
  bool checkForCollisionMMX(const vector<Node>& allObstacles, const Vector2<>& nearestNode, const Vector2<>& randomPosition, bool usePosition, float& distanceToObstacle, Vector2<>& positionObstacle, const Vector2<>& target);
  void calculateNearestNodeMMX(const vector<Node>& currentUsedTree, int& indexNearestNode, const Vector2<>& randomPosition);

};
