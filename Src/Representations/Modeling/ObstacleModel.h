/**
* @file ObstacleModel.h
*
* Declaration of class ObstacleModel
*
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
*/

#pragma once

#include "Tools/Math/Vector2.h"
#include "Tools/Math/Matrix2x2.h"
#include "Tools/Enum.h"
#include "Platform/BHAssert.h"
#include <vector>

class Pose3D;

/**
* @class ObstacleModel
*
* A class that represents the current state of the robot's environment
*/
class ObstacleModel : public Streamable
{
public:
  /** A single obstacle */
  class Obstacle : public Streamable
  {
  public:
    Vector2<> leftCorner;      /**< Leftmost point of the obstacle */
    Vector2<> rightCorner;     /**< Rightmost point of the obstacle */
    Vector2<> center;          /**< Center of mass of obstacle */
    Vector2<> closestPoint;    /**< Point of obstacle that is closest to the robot */
    int size;                  /**< The number of cells of the obstacle */
    Matrix2x2<> covariance;
    ENUM(Type, US, ROBOT, ARM, FOOT);/**< Different obstacle type */
    Type type;                 /**< The type of an obstacle */

    /** Empty default constructor*/
    Obstacle() {}

    /** Constructor */
    Obstacle(const Vector2<>& leftCorner, const Vector2<>& rightCorner,
             const Vector2<>& center, const Vector2<>& closestPoint, int size, const Matrix2x2<>& covariance, Type type = US) :
      leftCorner(leftCorner), rightCorner(rightCorner), center(center), closestPoint(closestPoint), size(size), covariance(covariance), type(type) {}

  private:
    /**
    * Makes the object streamable
    * @param in The stream from which the object is read
    * @param out The stream to which the object is written
    */
    void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN;
      STREAM(leftCorner);
      STREAM(rightCorner);
      STREAM(center);
      STREAM(closestPoint);
      STREAM(size);
      STREAM(covariance);
      STREAM(type);
      STREAM_REGISTER_FINISH;
    }
  };

  /** A list of obstacles */
  std::vector<Obstacle> obstacles;

  /** Function for drawing */
  void draw();

  void draw3D(const Pose3D& torsoMatrix) const;

private:
  /** Streaming function
  * @param in Object for streaming in the one direction
  * @param out Object for streaming in the other direction
  */
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(obstacles);
    STREAM_REGISTER_FINISH;
  }
};

/**
 * A compressed version of the obstacleModel.
 */
class ObstacleModelCompressed : public Streamable
{

public:
  ObstacleModelCompressed() {}

  ObstacleModelCompressed(const ObstacleModel& other)
  {
    obstacles.reserve(other.obstacles.size());
    for(size_t i = 0, count = other.obstacles.size(); i < count; i++)
      obstacles.push_back(Obstacle(other.obstacles[i]));
  }

  ObstacleModel unpack() const
  {
    ObstacleModel o;
    for(std::vector<Obstacle>::const_iterator it = obstacles.begin(); it != obstacles.end(); it++)
    {
      const Obstacle& compressed = *it;

      //x12 and x21 are identical, therefore only x12 is streamed.
      Matrix2x2<> uncompressedCovariance(compressed.x11, compressed.x12,
                                         compressed.x12, compressed.x22);

      ObstacleModel::Obstacle uncompressed(compressed.leftCorner,
                                           compressed.rightCorner,
                                           compressed.center,
                                           compressed.closestPoint,
                                           compressed.size,
                                           uncompressedCovariance,
                                           compressed.type);

      o.obstacles.push_back(uncompressed);
    }
    return o;
  }


private:

  /**
   * private obstacle with compressed streaming.
   */
   class Obstacle : public Streamable
   {
   public:
     Vector2<> leftCorner;      /**< Leftmost point of the obstacle */
     Vector2<> rightCorner;     /**< Rightmost point of the obstacle */
     Vector2<> center;          /**< Center of mass of obstacle */
     Vector2<> closestPoint;    /**< Point of obstacle that is closest to the robot */
     short int size;                  /**< Is downcasted from int.  */

     /**
      * The covariance is a 2x2 matrix. however x12 and x21 are always identical.
      * Therefore we only store 3 floats instead of 4: x11, x12 and x22
      * |x11  x12|
      * |        |
      * |x21  x22|
      */
     float x11, x12,x22;

     ObstacleModel::Obstacle::Type type;                 /**< The type of an obstacle */

     /**
      * Default ctor to be able to use std::copy.
      */
     Obstacle() : type(ObstacleModel::Obstacle::US) {}

     Obstacle(const ObstacleModel::Obstacle& other) :
       leftCorner(other.leftCorner), rightCorner(other.rightCorner),
       center(other.center),closestPoint(other.closestPoint),
       size((short int)other.size), x11(other.covariance.c[0].x),
       x12(other.covariance.c[1].x),x22(other.covariance.c[1].y),
       type(other.type) {}

   private:
     /**
     * Makes the object streamable
     * @param in The stream from which the object is read
     * @param out The stream to which the object is written
     */
     void serialize(In* in, Out* out)
     {
       STREAM_REGISTER_BEGIN;
       STREAM_COMPRESSED_POSITION(leftCorner);
       STREAM_COMPRESSED_POSITION(rightCorner);
       STREAM_COMPRESSED_POSITION(center);
       STREAM_COMPRESSED_POSITION(closestPoint);
       STREAM(size);

       STREAM_AS_UCHAR(type);
       //x11,x12 and x22 cannot be compressed. Their value range is too wide.
       STREAM(x11);
       STREAM(x12);
       STREAM(x22);
       STREAM_REGISTER_FINISH;
     }
   };


   std::vector<Obstacle> obstacles;

  /** Streaming function
  * @param in Object for streaming in the one direction
  * @param out Object for streaming in the other direction
  */
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(obstacles);
    STREAM_REGISTER_FINISH;
  }
};

