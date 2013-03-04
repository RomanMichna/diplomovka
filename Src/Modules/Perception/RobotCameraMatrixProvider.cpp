/**
* @file RobotCameraMatrixProvider.cpp
* This file implements a class to calculate the position of the camera relative to the body for the Nao.
* @author <a href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</a>
* @author Colin Graf
*/

#include "RobotCameraMatrixProvider.h"

MAKE_MODULE(RobotCameraMatrixProvider, Perception);

void RobotCameraMatrixProvider::update(RobotCameraMatrixOther& robotCameraMatrixOther)
{
  robotCameraMatrixOther.computeRobotCameraMatrix(theRobotDimensions, theFilteredJointData.angles[JointData::HeadYaw], theFilteredJointData.angles[JointData::HeadPitch], theCameraCalibration, theImageInfo.fromLowerCamera());
}

void RobotCameraMatrixProvider::update(RobotCameraMatrix& robotCameraMatrix)
{
  robotCameraMatrix.computeRobotCameraMatrix(theRobotDimensions, theFilteredJointData.angles[JointData::HeadYaw], theFilteredJointData.angles[JointData::HeadPitch], theCameraCalibration, !theImageInfo.fromLowerCamera());
}
