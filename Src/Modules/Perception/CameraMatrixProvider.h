/**
* @file CameraMatrixProvider.h
* This file declares a class to calculate the position of the camera for the Nao.
* @author <a href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</a>
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/MotionInfo.h"

MODULE(CameraMatrixProvider)
  REQUIRES(FrameInfo)
  REQUIRES(RobotInfo)
  REQUIRES(CameraCalibration)
  REQUIRES(RobotDimensions)
  REQUIRES(FilteredJointData)
  REQUIRES(MotionInfo)
  REQUIRES(FallDownState)
  REQUIRES(TorsoMatrix)
  REQUIRES(RobotCameraMatrix)
  REQUIRES(RobotCameraMatrixOther)
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(CameraMatrixOther)
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(CameraMatrix);
  USES(CameraMatrix)
  USES(FieldDimensions) // for debug drawing
  USES(RobotPose) // for debug drawing
  USES(CameraInfo) // for debug drawing
END_MODULE

class CameraMatrixProvider: public CameraMatrixProviderBase
{
private:
  void update(CameraMatrixOther& cameraMatrixOther);
  void update(CameraMatrix& cameraMatrix);

  void camera2image(const Vector3<>& camera, Vector2<>& image) const;
  bool intersectLineWithCullPlane(const Vector3<>& lineBase, const Vector3<>& lineDir,
                                  Vector3<>& point) const;
  void drawFieldLines(const CameraMatrix& cameraMatrix) const;
};
