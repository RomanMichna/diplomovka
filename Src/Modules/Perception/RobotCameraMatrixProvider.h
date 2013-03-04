/**
* @file RobotCameraMatrixProvider.h
* This file declares a class to calculate the position of the camera relative to the body for the Nao.
* @author <a href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</a>
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Infrastructure/ImageInfo.h"
#include "Representations/Perception/CameraMatrix.h"

MODULE(RobotCameraMatrixProvider)
  REQUIRES(CameraCalibration)
  REQUIRES(RobotDimensions)
  REQUIRES(FilteredJointData)
  REQUIRES(ImageInfo)
  PROVIDES_WITH_MODIFY_AND_DRAW(RobotCameraMatrix);
  PROVIDES(RobotCameraMatrixOther);
END_MODULE

class RobotCameraMatrixProvider: public RobotCameraMatrixProviderBase
{
private:
  void update(RobotCameraMatrix& robotCameraMatrix);
  void update(RobotCameraMatrixOther& robotCameraMatrixOther);
};
