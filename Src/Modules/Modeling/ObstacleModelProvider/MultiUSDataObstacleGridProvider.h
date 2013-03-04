/**
* @author <a href="mailto:nicolehm@tzi.de">Nico Lehmann</a>
* @author <a href="mailto:arneboe@tzi.de">Arne Böckmann</a>
* @author <a href="mailto:revan@tzi.de">Marcel Steinbeck</a>
* @author <a href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</a>
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
#include <algorithm>
#include <map>

MODULE(MultiUSDataObstacleGridProvider)
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
PROVIDES_WITH_MODIFY_AND_DRAW(USObstacleGrid)
END_MODULE


/**
* @class USObstacleGridProvider
*
* A module for computing the occupied space in the robot's environment
*/
class MultiUSDataObstacleGridProvider: public MultiUSDataObstacleGridProviderBase
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
      STREAM(groupingDistance);
      STREAM(cellOccupiedThreshold);
      STREAM(cellMaxOccupancy);
      STREAM(usRightPose);
      STREAM(usLeftPose);
      STREAM(usCenterPose);
      STREAM(agingFactorOnFreeDrawing);
      STREAM_REGISTER_FINISH;
    }

  public:
    int cellFreeInterval;                /**< Time (in ms) after which cell value becomes decreased */
    int cellOccupiedThreshold;           /**< If a cell state is higher than or equal to this value, it is considered as occupied */
    int cellMaxOccupancy;                /**< Maximum value for cell counter */
    float agingFactorOnFreeDrawing;      /**< the aging factor of a cell on free drawing relative to cellMaxOccupancy */
    float groupingDistance;              /**< Us measurements that are closer together than this value will be grouped together */
    Pose2D usRightPose;                  /**< Position and orientation of right US sensor*/
    Pose2D usLeftPose;                   /**< Position and orientation of left US sensor*/
    Pose2D usCenterPose;                 /**< Position and orientation of (virtual) center US sensor */

    /** Constructor */
    Parameters(): cellFreeInterval(3000), cellOccupiedThreshold(3), cellMaxOccupancy(255), agingFactorOnFreeDrawing(0.25)
    {
      usRightPose.translation = Vector2<>(53.7f, -34.1f);
      usLeftPose.translation  = Vector2<>(53.7f, 34.1f);
      usCenterPose.translation = Vector2<>(53.7f, 0);
      usRightPose.rotation = -0.350497f; //FIXME remove rotation
      usLeftPose.rotation  = 0.350497f;// FIXME remove rotation
      usCenterPose.rotation = 0.f;
    }
  };

  class USCalibration : public Streamable
  {
  public:
    float llLeftOpeningAngle;
    float llRightOpeningAngle;
    float lrLeftOpeningAngle;
    float lrRightOpeningAngle;
    float rrLeftOpeningAngle;
    float rrRightOpeningAngle;
    float rlLeftOpeningAngle;
    float rlRightOpeningAngle;

    /**
    * The method makes the object streamable.
    * @param in The stream from which the object is read
    * @param out The stream to which the object is written
    */
    virtual void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN;
      STREAM(llLeftOpeningAngle);
      STREAM(llRightOpeningAngle);
      STREAM(lrLeftOpeningAngle);
      STREAM(lrRightOpeningAngle);
      STREAM(rrLeftOpeningAngle);
      STREAM(rrRightOpeningAngle);
      STREAM(rlLeftOpeningAngle);
      STREAM(rlRightOpeningAngle);
      STREAM_REGISTER_FINISH;
    }
  };

  ENUM(AngleName,
    llLeft,
    llRight,
    lrLeft,
    lrRight,
    rlLeft,
    rlRight,
    rrLeft,
    rrRight
    );

  /**
   * Represents a simple angle
   */
  class Angle
  {
  public:
    Angle(float angle,AngleName name) : angle(angle), name(name){}

    bool operator< (const Angle& other) const
    {
      return angle < other.angle;
    }

    float angle;
    AngleName name;
  };

  ENUM(USSensorMode,
    leftToLeft,
    leftToRight,
    rightToLeft,
    rightToRight
    );

  class Interval
  {
    public:

    Interval(Angle left, Angle right) : left(left), right(right), hitCount(0)
    {}

    bool operator <(const Interval& other) const
    {
      return left.angle < other.left.angle;
    }

    Angle left; /** < opening angle */
    Angle right;/** < closing angle */
    /**
     * This value is used inside the algorithm to count how many measurements fall into this interval.
     */
    unsigned int hitCount;
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
    * Represents a single ultrasonic measurement.
    */
  class UsMeasurment
  {
    public:

    UsMeasurment(USSensorMode mode, float distance):mode(mode), distance(distance)
    {}

    bool operator <(const UsMeasurment& other) const
    {
      return this->distance < other.distance;
    }

    USSensorMode mode; /** < The mode the sensor was in when this measurement was taken. */
    float distance; /** < The measured distance */
  };

public:
  /** Constructor */
  MultiUSDataObstacleGridProvider();

private:
  Pose2D accumulatedOdometry;                 /**< Storing odometry differences for grid update */
  Pose2D lastOdometry;                        /**< Odometry value at last execution */
  unsigned lastTimePenalized;                 /**< Last point of time the robot was penalized */
  Parameters parameters;                      /**< The parameters of this module */
  USCalibration usCalibration;                /**< The calibration of the Ultrasonic Sensors*/
  unsigned lastUsTimeStamp;                   /**< The time stamp of the last used ultrasonic measurement */
  int gameInfoGameStateLastFrame;             /**< The game state in the last frame */
  USObstacleGrid* grid;

  float agingValueOnFreeDrawing;

  typedef vector<Interval> Intervals;
  Intervals intervals; /** < The measurement intervals of this robot. */

  typedef vector<Angle> Angles;
  Angles angles; /**< The opening and closing angles of this robots sensors. */

  typedef vector<UsMeasurment> Measurements;
  Measurements measurements; /** < Contains the  current sensor measurements*/

  typedef vector< vector<UsMeasurment> > MeasurementGroups;
  MeasurementGroups groups; /** < Contains grouped measurements. */

  vector<Point> polyPoints;                   /**< Vector of obstacle points to become processed. Clipped and filtered. */
  vector<int> inter;                          /**< The intersections per line. */

  /**
   * Each measurement affects the hitCount of several intervals.
   * This map defines which intervals are affected by a measurement made in a certain sensor mode.
   */
  map<USSensorMode, vector<Interval*> > sensorModeToInterval;

  /** Executes this module
  * @param usObstacleGrid The data structure that is filled by this module
  */
  void update(USObstacleGrid& usObstacleGrid);

  /** Enter obstacles to the grid */
  void checkUS();

  /** Moves the grid cells according to the odometry differences*/
  void moveGrid();

  inline void enterLine(float distance, float leftAngle, float rightAngle);

  void ageCellState();

  void incLine(unsigned time, Vector2<float> p1, Vector2<float> p2, Vector2<float> base);

  /**
   * Resets the interval's hitCount back to zero.
   */
  void resetIntervals();

  void clipPointP2(const Vector2<int>& p1, Point& p2) const;

  void fillScanBoundary();

  /**
   * Gets the current measurements from theFilteredSensorData and fills the
   * measurements attribute.
   * @param outMeasurements Will contain new measurements afterwards. Will be cleared before use.
   */
  void loadMeasurements(Measurements& outMeasurements);

  /**
   * separates the measurements into groups of similar distance.
   * @param measurements The measurements to be grouped. Will be empty afterwards.
   * @parma outGroups Contains the grouped measurements. Will be cleared before use.
   */
  void groupMeasurements(Measurements& measurements, MeasurementGroups& outGroups);

  /**
   * increases the hitCount of every interval within intervals.
   */
  void increaseHitCount(vector<Interval*> intervals);

  void resetHitCount(vector<Interval*> intervals);

  /**
   * Helper method to create the initial
   */
  void initializeIntervalMapping(USSensorMode key, AngleName begin, AngleName end);

  /**
   * Frees empty space between the robot and the obstacle
   */
  void freeEmptySpace(Vector2<float> p1, Vector2<float> p2, Vector2<float> base);

};
