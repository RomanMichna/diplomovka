/**
* @file CameraInfo.h
*
* Declaration of class CameraInfo
*
* @author <a href="mailto:juengel@informatik.hu-berlin.de">Matthias Juengel</a>
* @author <a href="mailto:walter.nistico@uni-dortmund.de">Walter Nistico</a>
*/

#pragma once

#include "Tools/Math/Vector2.h"

/**
* Information about the camera which provides the images for the robot
*/
class CameraInfo : public Streamable
{
public:
  int resolutionWidth;
  int resolutionHeight;
  float openingAngleWidth;
  float openingAngleHeight;

  /** Intrinsic camera parameters: axis skew is modelled as 0 (90° perfectly orthogonal XY)
  * and the same has been modeled for focal axis aspect ratio; distortion is considering
  * only 2nd and 4th order coefficients of radial model, which account for about 95% of total.
  */
  float focalLength;
  float focalLengthInv; // (1/focalLength) used to speed up certain calculations
  Vector2<> opticalCenter;
  float focalLenPow2;
  float focalLenPow4;

  /**
  * Default constructor.
  */
  CameraInfo();

private:
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(resolutionWidth);
    STREAM(resolutionHeight);
    STREAM(openingAngleWidth);
    STREAM(openingAngleHeight);
    STREAM(focalLength);
    STREAM(opticalCenter);
    STREAM_REGISTER_FINISH;
    if(in)
    {
      focalLength = resolutionWidth / (2.f * tan(openingAngleWidth / 2.f));
      focalLengthInv = 1.0f / focalLength;
      focalLenPow2 = focalLength * focalLength;
      focalLenPow4 = focalLenPow2 * focalLenPow2;
    }
  }

  /**
  * Calculates some additional constants based on focal length for faster calculations.
  */
  void calcAdditionalConstants();
};
