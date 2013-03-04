/**
* @file UKF.cpp
* Template class for two-dimensional Kalman filtering.
* @author <a href="mailto:afabisch@tzi.de">Alexander Fabisch</a>
*/

#include "UKF.h"
#include "Tools/Math/Common.h"
#include "Tools/Math/Covariance.h"

void UKF::init(
  const UKF::Prediction& prediction,
  const UKF::Measurement& measurement,
  const Vector2<>& mean,
  const Matrix2x2<>& covariance)
{
  g = &prediction;
  h = &measurement;
  this->mean = mean;
  this->covariance = covariance;

  initialized = true;
}

void UKF::prediction(const Vector3<>& control, const Matrix2x2<>& R)
{
  Matrix2x2<> L = Covariance::choleskyDecomposition(covariance);
  std::vector<Vector2<> > sigmaPoints(5);
  sigmaPoints[0] = (*g)(control, mean);
  for(int i = 0; i < 2; i++)
  {
    sigmaPoints[i + 1] = (*g)(control, mean + L[i]);
    sigmaPoints[i + 3] = (*g)(control, mean - L[i]);
  }

  mean = (sigmaPoints[0] + sigmaPoints[1] + sigmaPoints[2] + sigmaPoints[3] + sigmaPoints[4]) / 5.0f;

  Matrix2x2<> covG;
  for(int i = 0; i < 5; i++)
  {
    sigmaPoints[i] -= mean;
    covG[0][0] += sigmaPoints[i][0] * sigmaPoints[i][0];
    covG[1][1] += sigmaPoints[i][1] * sigmaPoints[i][1];
    const float cov01 = sigmaPoints[i][0] * sigmaPoints[i][1];
    covG[1][0] += cov01;
    covG[0][1] += cov01;
  }
  covG /= 2.0f;

  covariance = covG + R;
}

void UKF::update(const Vector2<> measurement,
                 const Matrix2x2<>& Q,
                 const CameraMatrix& theCameraMatrix,
                 const CameraInfo& theCameraInfo,
                 const ImageCoordinateSystem& theImageCoordinateSystem)
{
  Matrix2x2<> L = Covariance::choleskyDecomposition(covariance);
  std::vector<Vector2<> > sigmaPoints(5);
  sigmaPoints[0] = (*h)(mean, theCameraMatrix, theCameraInfo, theImageCoordinateSystem);
  for(int i = 0; i < 2; i++)
  {
    sigmaPoints[i + 1] = (*h)(mean + L[i], theCameraMatrix, theCameraInfo, theImageCoordinateSystem);
    sigmaPoints[i + 3] = (*h)(mean - L[i], theCameraMatrix, theCameraInfo, theImageCoordinateSystem);
  }

  Vector2<> meanH = (sigmaPoints[0] + sigmaPoints[1] + sigmaPoints[2] + sigmaPoints[3] + sigmaPoints[4]) / 5.0f;

  Matrix2x2<> covH;
  for(int i = 0; i < 5; i++)
  {
    sigmaPoints[i] -= meanH;
    covH[0][0] += sigmaPoints[i][0] * sigmaPoints[i][0];
    covH[1][1] += sigmaPoints[i][1] * sigmaPoints[i][1];
    const float cov01 = sigmaPoints[i][0] * sigmaPoints[i][1];
    covH[0][1] += cov01;
    covH[1][0] += cov01;
  }
  covH /= 2.0f;

  Matrix2x2<> covHx;
  for(int i = 1; i < 5; i++)
  {
    const float s = i > 2 ? -1.0f : 1.0f;
    const int Lcol = i % 2 == 1 ? 0 : 1;
    covHx[0][0] += sigmaPoints[i][0] * s * L[Lcol][0];
    covHx[1][1] += sigmaPoints[i][1] * s * L[Lcol][1];
    covHx[1][0] += sigmaPoints[i][0] * s * L[Lcol][1];
    covHx[0][1] += sigmaPoints[i][1] * s * L[Lcol][0];
  }
  covHx /= 2.0f;

  const Matrix2x2<> K = covHx.transpose() * (covH + Q).invert();
  const Vector2<> innovation = measurement - meanH;
  mean += K * innovation;
  covariance -= K * covHx;
}

float UKF::approximateProbabilityAtMeanArea(float halfSquareLength) const
{
  return 4.0f * halfSquareLength * halfSquareLength / (pi2 * sqrt(covariance.det()));
}
