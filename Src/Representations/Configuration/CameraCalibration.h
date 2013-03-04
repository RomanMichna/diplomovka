/**
* @file CameraCalibration.h
* Declaration of a class for representing the calibration values of the camera.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#pragma once

#include "Tools/Math/Pose3D.h"
#include "Tools/Math/Vector3.h"
#include "Tools/Math/Common.h"

class CameraCalibration : public Streamable
{
public:
  enum ColorTemperature // no ENUM, because the string representations are different
  {
    defaultCamera, /**< coltable.c64 */
    simCamera, /**< coltableSim.c64 (simulated camera of simulated robotes) */
    newCamera, /**< coltableNew.c64 */
    oldCamera, /**< coltableOld.c64 */
    numOfColorTemperatures,
  };

  float cameraTiltCorrection, /**< The correction of the camera tilt angle in radians. */
        cameraRollCorrection, /**< The correction of the camera roll angle in radians. */
        cameraPanCorrection, /**< The correction of the camera pan angle in radians. */
        bodyTiltCorrection, /**< The correction of the body tilt angle in radians. */
        bodyRollCorrection; /**< The correction of the body roll angle in radians. */
  Vector3<> bodyTranslationCorrection; /**< The correction of the body translation in mm. */
  Vector3<> upper2lowerRotation; /**< The rotaional offset of the upper relative to the lower camera in angle-axis format. */
  Vector3<> upper2lowerTranslation; /** Translation of the upper camera relative to the lower camera. */

  ColorTemperature colorTemperature; /**< The type of the camera. (Since "new" and "old" Nao cameras privide images with different color temperature.) */

  /**
  * Default constructor.
  */
  CameraCalibration() : cameraTiltCorrection(0.0f), cameraRollCorrection(0.0f), cameraPanCorrection(0.0f),
    bodyTiltCorrection(0.0f), bodyRollCorrection(0.0f), colorTemperature(defaultCamera) {}

  /**
  * The function returns the name of a color temperature.
  * @param colorTemperature The color temperature the name of which is returned.
  * @return The corresponding name or 0 if parameter is out of range.
  */
  static const char* getName(ColorTemperature colorTemperature)
  {
    switch(colorTemperature)
    {
    case defaultCamera:
      return "default";
    case simCamera:
      return "sim";
    case newCamera:
      return "new";
    case oldCamera:
      return "old";
    default:
      return 0;
    }
  }

private:
  /**
  * The method makes the object streamable.
  * @param in The stream from which the object is read
  * @param out The stream to which the object is written
  */
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(cameraTiltCorrection);
    STREAM(cameraRollCorrection);
    STREAM(cameraPanCorrection);
    STREAM(bodyTiltCorrection);
    STREAM(bodyRollCorrection);
    STREAM(bodyTranslationCorrection);
    STREAM(upper2lowerRotation);
    STREAM(upper2lowerTranslation);
    STREAM(colorTemperature);
    STREAM_REGISTER_FINISH;
  }
};
