/**
* @file CameraMatrix.h
* Declaration of CameraMatrix and RobotCameraMatrix representation.
* @author Colin Graf
*/

#pragma once

#include "Tools/Math/Pose3D.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Configuration/CameraCalibration.h"

/**
* Matrix describing transformation from center of hip to camera.
*/
class RobotCameraMatrix : public Pose3D
{
public:
  /** Draws the camera matrix. */
  void draw();

  void computeRobotCameraMatrix(const RobotDimensions& robotDimensions, float headYaw, float headPitch, const CameraCalibration& cameraCalibration, bool upperCamera);
  RobotCameraMatrix() {}
  RobotCameraMatrix(const RobotDimensions& robotDimensions, const float headYaw, const float headPitch, const CameraCalibration& cameraCalibration, bool upperCamera);
};

/**
* Matrix describing transformation from ground (center between booth feet) to camera.
*/
class CameraMatrix : public Pose3D
{
public:
  bool isValid; /**< Matrix is only valid if motion was stable. */

  /** Kind of copy-constructor.
  * @param pose The other pose.
  */
  CameraMatrix(const Pose3D& pose): Pose3D(pose), isValid(true) {}

  /** Default constructor. */
  CameraMatrix() : isValid(true) {}

  void computeCameraMatrix(const Pose3D& torsoMatrix, const Pose3D& robotCameraMatrix, const CameraCalibration& cameraCalibration);
  CameraMatrix(const Pose3D& torsoMatrix, const Pose3D& robotCameraMatrix, const CameraCalibration& cameraCalibration);

  /** Draws the camera matrix. */
  void draw();

private:
  /**
  * The method makes the object streamable.
  * @param in The stream from which the object is read.
  * @param out The stream to which the object is written.
  */
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM_BASE(Pose3D);
    STREAM(isValid);
    STREAM_REGISTER_FINISH;
  }
};

class CameraMatrixOther : public CameraMatrix {};
class RobotCameraMatrixOther : public RobotCameraMatrix {};
