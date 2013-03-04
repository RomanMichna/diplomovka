/**
* @file USObstacleGridProvider.h
*
* This file declares a module that provides information about occupied space in the robot's environment.
* The module computes an occupancy grid based on ultrasonic measurements.
* It includes parts of the implementation of the PolygonLocalMapper module of the
* autonomous wheelchair Rolland.
*
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
* @author <a href="mailto:cman@tzi.de">Christian Mandel</a>
*/

#pragma once

#include "Representations/Modeling/USObstacleGrid.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/Module/Module.h"
#include <stack>


MODULE(USObstacleGridProvider)
#ifdef TARGET_SIM
  REQUIRES(GroundTruthRobotPose)
#endif
  REQUIRES(OdometryData)
  REQUIRES(FrameInfo)
  REQUIRES(FilteredSensorData)
  REQUIRES(FilteredJointData)
  REQUIRES(JointRequest)
  REQUIRES(RobotPose)
  REQUIRES(GameInfo)
  USES(MotionRequest)
  USES(MotionInfo)
  USES(RobotInfo)
  USES(USObstacleGrid)
  PROVIDES_WITH_MODIFY_AND_DRAW(USObstacleGrid)
END_MODULE


/**
* @class USObstacleGridProvider
*
* A module for computing the occupied space in the robot's environment
*/
class USObstacleGridProvider: public USObstacleGridProviderBase
{
private:
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
      STREAM(cellFreeInterval);
      STREAM(usOuterOpeningAngle);
      STREAM(usInnerOpeningAngle);
      STREAM(usCenterOpeningAngle);
      STREAM(maxValidUSDist);
      STREAM(minValidUSDist);
      STREAM(cellOccupiedThreshold);
      STREAM(cellMaxOccupancy);
      STREAM(usRightPose);
      STREAM(usLeftPose);
      STREAM(usCenterPose);
      STREAM_REGISTER_FINISH;
    }

  public:
    int cellFreeInterval;                /**< Time (in ms) after which cell value becomes decreased */
    float usOuterOpeningAngle;           /**< Opening angle of US sensor towards side of robot */
    float usInnerOpeningAngle;           /**< Opening angle of US sensor towards center of robot */
    float usCenterOpeningAngle;          /**< Opening angle of Center US sensor to each side */
    int maxValidUSDist;                  /**< Maximum measured distance which is considered for the model */
    int minValidUSDist;                  /**< Minimum measured distance which is considered for the model */
    int cellOccupiedThreshold;           /**< If a cell state is higher than or equal to this value, it is considered as occupied */
    int cellMaxOccupancy;                /**< Maximum value for cell counter */
    Pose2D usRightPose;                  /**< Position and orientation of right US sensor*/
    Pose2D usLeftPose;                   /**< Position and orientation of left US sensor*/
    Pose2D usCenterPose;                 /**< Position and orientation of (virtual) center US sensor */

    /** Constructor */
    Parameters(): cellFreeInterval(3000), maxValidUSDist(1199), minValidUSDist(200),
      cellOccupiedThreshold(3), cellMaxOccupancy(4)
    {
      usInnerOpeningAngle = fromDegrees(30);
      usOuterOpeningAngle = fromDegrees(70);
      usCenterOpeningAngle = fromDegrees(30);
      usRightPose.translation = Vector2<>(53.7f, -34.1f);
      usLeftPose.translation  = Vector2<>(53.7f, 34.1f);
      usCenterPose.translation = Vector2<>(53.7f, 0);
      usRightPose.rotation = -0.350497f;
      usLeftPose.rotation  = 0.350497f;
      usCenterPose.rotation = 0.f;
    }
  };

  /**
  * @class Point
  * The class represents a grid point together with clipping information.
  */
  class Point : public Vector2<int>
  {
  public:
    /**
    * The enumeration represents different edges of the map
    * to which scan points can be clipped, and further options.
    */
    enum Flags
    {
      NONE,
      NO_OBSTACLE = 16,
      PEAK = 32
    };
    int flags; /**< The set of edges to which the point was clipped, and whether it results from a castor wheel. */

    /**
    * Default constructor.
    */
    Point() : flags(NONE) {}

    /**
    * Constructor.
    * @param x The x coordinate of the point.
    * @param y The y coordinate of the point.
    * @param flags The set of edges to which the point was clipped, and whether
    *              it results from a castor wheel.
    */
    Point(int x, int y, int flags = 0) : Vector2<int>(x, y), flags(flags) {}

    /**
    * Constructor.
    * @param v The x and y coordinates of the point.
    * @param flags The set of edges to which the point was clipped, and whether
    *              it results from a castor wheel.
    */
    Point(const Vector2<int>& v, int flags = 0) : Vector2<int>(v), flags(flags) {}
  };

  /**
  * @class Line
  * This class represents a single line-segment out of a polyline.
  */
  class Line
  {
  public:
    /**
    * Constructor.
    * @param ai The first point of the line-segment.
    * @param bi The last point of the line-segment.
    */
    Line(const Point& ai, const Point& bi)
      : peak(ai.flags& Point::PEAK || bi.flags& Point::PEAK)
    {
      if(ai.y <= bi.y)
      {
        a = Vector2<>((float)ai.x, (float)ai.y);
        b = Vector2<>((float)bi.x, (float)bi.y);
      }
      else
      {
        b = Vector2<>((float)ai.x, (float)ai.y);
        a = Vector2<>((float)bi.x, (float)bi.y);
      }
      ils = b.y != a.y ? (b.x - a.x) / (b.y - a.y) : 0;
    }

    /**
    * Operator that compares a given line with this line.
    * @param other This line is compared to other.
    * @return Is the y coordinate of this line smaller than the one of the other line?
    */
    bool operator<(const Line& other) const
    {
      return a.y < other.a.y;
    }

    Vector2<> a; /**< First point of this line. */
    Vector2<> b; /**< Last point of this line. */
    float ils;   /**< Inverse slope of this line. */
    bool peak;   /**< Is this line part of an upper peak? */
  };

  /**
  * @class UsMeasurement
  * A buffered us measurement.
  */
  class UsMeasurement
  {
  public:
    float value; /**< The measured distance. */
    unsigned int timeStamp; /**< Time stamp of the measurement. */

    /**
    * Default constructor.
    */
    UsMeasurement() : value(0), timeStamp(0) {}

    /**
    * Constructor.
    * @param value The measured distance.
    * @param timeStamp Time stamp of the measurement.
    */
    UsMeasurement(float value, unsigned int timeStamp) : value(value), timeStamp(timeStamp) {}
  };

public:
  /** Constructor */
  USObstacleGridProvider();

private:


  ENUM(ActuatorMode,
      leftToLeft,
      leftToRight,
      rightToLeft,
      rightToRight
      );

  Pose2D accumulatedOdometry;                 /**< Storing odometry differences for grid update */
  Pose2D lastOdometry;                        /**< Odometry value at last execution */
  unsigned lastTimePenalized;                 /**< Last point of time the robot was penalized */
  Parameters parameters;                      /**< The parameters of this module */
  vector<Point> polyPoints;                   /**< Vector of obstacle points to become processed. Clipped and filtered. */
  vector<int> inter;                          /**< The intersections per line. */
  UsMeasurement bufferedMeasurements[4];      /**< The last valid us measurements of each actuator mode */
  vector<UsMeasurement> postponedLeftToRightMeasurements; /**< leftToRight measurements that were postponed */
  vector<UsMeasurement> postponedRightToLeftMeasurements; /**< rightToLeft measurements that were postponed */
  bool measuredCenterLeft;                    /**< Flag that indicates that an actual obstacle has currently been entered at the left center */
  bool measuredCenterRight;                   /**< Flag that indicates that an actual obstacle has currently been entered at the right center */
  SensorData::UsActuatorMode lastUsActuatorMode; /**< The controlled actuator mode of the last us measurement */
  unsigned lastUsTimeStamp;                   /**< The time stamp of the last used ultrasonic measurement */
  bool initialized;                           /**< flag for initialization */
  USObstacleGrid::Cell* cells;                /**< Pointer to cells in grid (for shorter notation)*/
  int gameInfoGameStateLastFrame;             /**< The game state in the last frame */


  /** Executes this module
  * @param usObstacleGrid The data structure that is filled by this module
  */
  void update(USObstacleGrid& usObstacleGrid);

  /** Enter obstacles to the grid */
  void checkUS();

  /** Inserts us measurements into the grid.
  * @param actuatorChanged whether the actuator mode has changed compared to the previous iteration
  * @param m The measured value
  * @param mappedActuatorMode The actual actuator mode that is some how different to the controlled actuator mode
  * @param timeStamp The time stamp of the measurement
  */
  void addUsMeasurement(bool actuatorChanged, float m, ActuatorMode mappedActuatorMode, unsigned int timeStamp);

  /** Ages all cells, i.e. decreases the obstacle value in each cell */
  void ageCellState();

  /** Called by occupyCells, frees the space between the sensor and the obstacle */
  void fillScanBoundary();

  /** Clips point to the grid
  * @param p1 The position from which the measurement is done
  * @param p2 The measured point
  */
  void clipPointP2(const Vector2<int>& p1, Point& p2) const;

  /** Enters an obstacle line to the grid
  * @param start The start of the line
  * @param end The end of the line
  */
  void line(const Vector2<int>& start, const Vector2<int>& end);

  /** Moves the grid cells according to the odometry differences*/
  void moveGrid();
};
