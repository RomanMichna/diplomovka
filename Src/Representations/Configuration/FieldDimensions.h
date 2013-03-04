/**
* @file FieldDimensions.h
*
* Description of the dimensions of the field.
*
* @author Matthias J�ngel
* @author Thomas R�fer
*/

#pragma once

#include "Tools/Boundary.h"
#include "Tools/Math/Pose2D.h"
#include "Tools/Enum.h"

class ConfigMap;

/**
* Class containing definitions and functions
* regarding field dimensions.
*
* @author Max Risler
*/
class FieldDimensions : public Boundary<>, public Streamable
{
public:
  int xPosOpponentFieldBorder;
  int xPosOpponentGoal;
  int xPosOpponentGoalpost;
  int xPosOpponentGroundline;
  int xPosOpponentSideCorner;
  int xPosOpponentPenaltyArea;
  int xPosOpponentPenaltyMark;
  int xPosHalfWayLine;
  int xPosOwnPenaltyArea;
  int xPosOwnPenaltyMark;
  int xPosOwnSideCorner;
  int xPosOwnGroundline;
  int xPosOwnGoalpost;
  int xPosOwnGoal;
  int xPosOwnFieldBorder;

  int yPosLeftFieldBorder;
  int yPosLeftSideline;
  int yPosLeftGroundline;
  int yPosLeftPenaltyArea;
  int yPosLeftGoal;
  int yPosCenterGoal;
  int yPosRightGoal;
  int yPosRightPenaltyArea;
  int yPosRightGroundline;
  int yPosRightSideline;
  int yPosRightFieldBorder;

  //other dimensions
  int centerCircleRadius;
  int goalHeight;
  int ballRadius;
  int ballFriction; // in mm/s^2
  int fieldLinesWidth;
  int goalPostRadius;

  //throw-in points
  int xPosThrowInPointOpponentHalf;
  int xPosThrowInPointCenter;
  int xPosThrowInPointOwnHalf;

  //drop-in line
  int yPosDropInLineLeft;
  int yPosDropInLineRight;
  int xPosDropInLineOpponentHalf;
  int xPosDropInLineOwnHalf;

  /**
  *
  */
  class Dimension : public Streamable
  {
  private:
    virtual void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN;
      STREAM(id);
      STREAM(value);
      STREAM_REGISTER_FINISH;
    }

  public:
    std::string id;
    std::string value;
  };

  /**
  * This is a collection of line- or boundary segments with start-Pose2D and length.
  */
  class LinesTable : public Streamable
  {
  private:
    virtual void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN;
      STREAM(lines);
      STREAM_REGISTER_FINISH;
    }

  public:
    class Line : public Streamable
    {
    public:
      Pose2D corner; /**< The field corners. */
      float length; /**< The lengths of the border segments starting at a corresponding corner. */
      bool isPartOfCircle; /**< Whether the line is a part of a circle. */
      virtual void serialize(In* in, Out* out)
      {
        STREAM_REGISTER_BEGIN;
        STREAM(corner);
        STREAM(length);
        STREAM(isPartOfCircle);
        STREAM_REGISTER_FINISH;
      }
    };
    std::vector<Line> lines;

    void push(const Pose2D& p, float l, bool isPartOfCircle = false);

    void push(const Vector2<>& s, const Vector2<>& e, bool isPartOfCircle = false);

    void pushCircle(const Vector2<>& center, float radius, int numOfSegments);

    void pushQuarterCircle(const Vector2<>& center, float radius, int numOfSegments, int angle);

    /**
    * Doubles the line segments creating float sided segments that have edges on both sides.
    * @param width The distance between both edges of each line segment.
    * @param single A lines table with single-sided lines.
    */
    void doubleSided(float width, const LinesTable& single);

    /*
    * Returns whether a given point is inside the polygon described by the line segments.
    * Only valid if the line segment table describes a closed polygon.
    */

    bool isInside(const Vector2<>& v) const;

    /**
    * The function clips a point to the polygon described by the line segments.
    * Only valid if the line segment table describes a closed polygon.
    * @param v The point.
    * @return How far was the point moved?
    */
    float clip(Vector2<>& v) const;

    /**
    * The function returns the point on a line of a certain type closest to given a point.
    * @param point The point on a line.
    * @param p The reference point and the rotation of the line.
    * @param numberOfRotations The number of discretizations of line rotations.
    * @param minLength The minimum length of the line segments that are considered.
    * @return whether there is a matching point in that direction
    */
    bool getClosestPoint(Vector2<>& point, const Pose2D& p, int numberOfRotations, float minLength) const;

    /**
    * The function returns the distance between a point and the closest point on a line of a certain type in a certain direction.
    * @param pose The reference point and direction.
    * @return The distance. It is -1 if no line of that type exists in the certain direction.
    */
    float getDistance(const Pose2D& pose) const;

    /**
    * Draws a debug field drawing that displays the set of lines.
    */
    //void draw(const ColorRGBA& color, bool drawNormals = true) const;
  };

  /**
  * Tables of line segments
  */
  LinesTable fieldLines;

  /**
  * Describes a polygon around the border of the field carpet.
  * All legal robot positions are inside this polygon.
  */
  LinesTable carpetBorder;

  /**
  * Describes a polygon around the border of the playing field.
  * All legal ball positions are inside this polygon.
  */
  LinesTable fieldBorder;

  /**
  * The class represents all corners of a certain type.
  * It only exists, because an array of vectors is not serializable.
  */
  class CornersTable : public std::vector<Vector2<int> >, public Streamable
  {
  private:
    virtual void serialize(In* in, Out* out)
    {
      std::vector<Vector2<int> >& corners = *this;
      STREAM_REGISTER_BEGIN;
      STREAM(corners);
      STREAM_REGISTER_FINISH;
    }

  public:
    /**
    * The method returns the position of the corner closest to a point.
    * The method is only defined if !empty().
    * @param p The point.
    * @return The position of the closest corner.
    */
    const Vector2<int>& getClosest(const Vector2<int>& p) const;

    /**
    * The method returns the position of the corner closest to a point.
    * The method is only defined if !empty().
    * @param p The point.
    * @return The position of the closest corner.
    */
    const Vector2<> getClosest(const Vector2<>& p) const;
  };

  /**
  * All different corner classes.
  */
  ENUM(CornerClass,
    xCorner,
    tCorner0,
    tCorner90,
    tCorner180,
    tCorner270,
    lCorner0,
    lCorner90,
    lCorner180,
    lCorner270
  );
  enum {numOfCornerClasses = numOfCornerClasss}; // extra, because numOfCornerClasss isn't so nice

  CornersTable corners[numOfCornerClasses]; /**< All corners on the field. */

public:
  /**
   * Default constructor.
   */
  FieldDimensions();

  /**
  * Read field dimensions from configuration file.
  */
  void load();

  /**
  * Returns true when p is inside the carpet.
  */
  bool isInsideCarpet(const Vector2<> &p) const
  {
    return carpetBorder.isInside(p);
  }

  /**
  * The function clips a point to the carpet.
  * @param v The point.
  * @return How far was the point moved?
  */
  float clipToCarpet(Vector2<>& v) const
  {
    return carpetBorder.clip(v);
  }

  /**
  * Returns true when p is inside the playing field.
  */
  bool isInsideField(const Vector2<> &p) const
  {
    return fieldBorder.isInside(p);
  }

  /**
  * The function clips a point to the field.
  * @param v The point.
  * @return How far was the point moved?
  */
  float clipToField(Vector2<>& v) const
  {
    return fieldBorder.clip(v);
  }

  /**
  * The function returns a random pose inside the field.
  * @return The random pose.
  */
  Pose2D randomPoseOnField() const;

  /**
  * The function returns a random pose on the carpet.
  * @return The random pose.
  */
  Pose2D randomPoseOnCarpet() const;

  /**
  * The method draws the field lines.
  */
  void draw() const;

  /**
  * The method draws the field polygons.
  * @param ownColor The color of the own team.
  */
  void drawPolygons(unsigned ownColor) const;

private:
  virtual void serialize(In* in, Out* out);

  /**
  * The method draws the field lines.
  */
  void drawLines() const;

  /**
  * The method draws the field lines.
  */
  void drawCorners() const;

  void readDimensions(int* const* valuesInt, const char* const* namesInt, bool* initializedInt, int numOfValuesInt, const ConfigMap& cm);
  void readLines(const int* const* valuesInt, const char* const* namesInt, const bool* initializedInt, int numOfValuesInt, const ConfigMap& cm);
  void readCorners(const int* const* valuesInt, const char* const* namesInt, const bool* initializedInt, int numOfValuesInt, const ConfigMap& cm);

  int readValue(const std::string& buf, const int* const* values, const char* const* names, const bool* initialized, int numOfValues) const;
};
