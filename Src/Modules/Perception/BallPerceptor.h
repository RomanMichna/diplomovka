/**
* @file BallPerceptor.h
* This file declares a module that provides the ball percept.
* @author Colin Graf
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Perception/BallSpots.h"
#include "Representations/Perception/BallPercept.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/Configuration/ConfigMap.h"

MODULE(BallPerceptor)
  REQUIRES(FieldDimensions)
  REQUIRES(Image)
  REQUIRES(CameraMatrix)
  REQUIRES(ImageCoordinateSystem)
  REQUIRES(CameraInfo)
  REQUIRES(BallSpots)
  REQUIRES(OdometryData)
  USES(RobotPose)
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(BallPercept)
END_MODULE

class BallPerceptor : public BallPerceptorBase
{
private:
  /**
  * A collection of parameters for the ball perceptor.
  */
  class Parameters : public Streamable
  {
  public:
    /** Default constructor. */
    Parameters() {}

    float clippingApproxRadiusScale;
    float clippingApproxRadiusPixelBonus;

    unsigned int scanMaxColorDistance;
    unsigned int scanPixelTolerance;

    unsigned int refineMaxPixelCount;
    unsigned int refineMaxColorDistance;

    float checkMaxRadiusDifference;
    float checkMinRadiusDifference;
    float checkMinRadiusPixelBonus;

    float checkOutlineRadiusScale;
    float checkOutlineRadiusPixelBonus;

  private:
    /**
    * The method makes the object streamable.
    * @param in The stream from which the object is read.
    * @param out The stream to which the object is written.
    */
    virtual void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN;

      STREAM(clippingApproxRadiusScale);
      STREAM(clippingApproxRadiusPixelBonus);

      STREAM(scanMaxColorDistance);
      STREAM(scanPixelTolerance);

      STREAM(refineMaxPixelCount);
      STREAM(refineMaxColorDistance);

      STREAM(checkMaxRadiusDifference);
      STREAM(checkMinRadiusDifference);
      STREAM(checkMinRadiusPixelBonus);

      STREAM(checkOutlineRadiusScale);
      STREAM(checkOutlineRadiusPixelBonus);

      STREAM_REGISTER_FINISH;
    }
  };

  class BallPoint
  {
  public:
    Vector2<int> step;
    Vector2<int> start;
    Vector2<int> point;
    Vector2<> pointf;
    bool atBorder;
    bool isValid;

    BallPoint() : atBorder(false), isValid(false) {}
  };

  Parameters p; /**< Parameters for the module. */
  float sqrMaxBallDistance; /**< The square of the maximal allowed ball distance. */
  std::vector<BallPercept> ballPercepts;
  Pose2D lastOdometryData;

  void init();
  void update(BallPercept& ballPercept);

  bool checkBallSpot(const BallSpot& ballSpot);
  float approxRadius1; /**< Bearing based approximation of the radius. */

  bool searchBallPoints(const BallSpot& ballSpot);
  Image::Pixel startPixel; /**< The ball spot pixel. */
  BallPoint ballPoints[8]; /**< Points on the outer edge of the ball. */
  Vector2<int> approxCenter2;
  int totalPixelCount;
  int totalCbSum;
  int totalCrSum;

  bool searchBallPoint(const Vector2<int>& start, Image::Pixel startPixel, const Vector2<int>& step, float maxLength, BallPoint& ballPoint);

  bool checkBallPoints();
  bool getBallFromBallPoints(Vector2<>& center, float& radius) const;
  unsigned int validBallPoints; /**< Count of usable points on the outer edge of the ball. */

  bool calculateBallInImage();
  Vector2<> center; /**< Center of the ball in image. */
  float radius; /**< Radius of the ball in image. */

  bool checkBallInImage();

  bool calculateBallOnField();
  Vector3<> sizeBasedCenterOnField;
  Vector3<> bearingBasedCenterOnField;
  Vector3<> usedCenterOnField;

  bool checkBallOnField();
};
