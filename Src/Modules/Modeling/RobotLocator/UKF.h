/**
* @file UKF.h
* Class for two-dimensional kalman filtering. Assumes that the control and measurement vectors are 3D respectively 2D.
* @author <a href="mailto:afabisch@tzi.de">Alexander Fabisch</a>
*/

#pragma once

#include "Tools/Math/Vector2.h"
#include "Tools/Math/Vector3.h"
#include "Tools/Math/Matrix2x2.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Infrastructure/CameraInfo.h"

class UKF
{
public:
  class Prediction
  {
  public:
    virtual Vector2<> operator()(const Vector3<>& u, const Vector2<>& x) const = 0;
  };
  class Measurement
  {
  public:
    virtual Vector2<> operator()(const Vector2<>& x,
                                 const CameraMatrix& theCameraMatrix,
                                 const CameraInfo& theCameraInfo,
                                 const ImageCoordinateSystem& theImageCoordinateSystem) const = 0;
  };

private:
  bool initialized;
  Prediction const* g;
  Measurement const* h;

public:
  Vector2<> mean;
  Matrix2x2<> covariance;

  UKF() : initialized(false), g(0), h(0) {}

  void init(
    const UKF::Prediction& prediction,
    const UKF::Measurement& measurement,
    const Vector2<>& mean,
    const Matrix2x2<>& covariance);

  void prediction(const Vector3<>& control, const Matrix2x2<>& R);
  void update(
    const Vector2<> measurement,
    const Matrix2x2<>& Q,
    const CameraMatrix& theCameraMatrix,
    const CameraInfo& theCameraInfo,
    const ImageCoordinateSystem& theImageCoordinateSystem);

  float approximateProbabilityAtMeanArea(float halfSquareLength) const;
};
