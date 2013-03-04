/**
* @file BodyContourProvider.h
* This file implements a module that provides the contour of the robot's body in the image.
* The contour can be used to exclude the robot's body from image processing.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#include "BodyContourProvider.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Configuration/ConfigMap.h"


BodyContourProvider::BodyContourProvider()
{
  InConfigMap stream("bodyContour.cfg");
  ASSERT(stream.exists());
  stream >> parameters;
}

void BodyContourProvider::update(BodyContour& bodyContour)
{
  DECLARE_DEBUG_DRAWING3D("module:BodyContourProvider:contour", "origin");

  MODIFY("parameters:BodyContourProvider", parameters);

  bodyContour.cameraResolution.x = theCameraInfo.resolutionWidth;
  bodyContour.cameraResolution.y = theCameraInfo.resolutionHeight;
  bodyContour.lines.clear();

  robotCameraMatrixInverted = theRobotCameraMatrix.invert();

  add(Pose3D(), parameters.torso, 1, bodyContour);
  add(theRobotModel.limbs[MassCalibration::bicepsLeft], parameters.upperArm, 1, bodyContour);
  add(theRobotModel.limbs[MassCalibration::bicepsRight], parameters.upperArm, -1, bodyContour);
  add(theRobotModel.limbs[MassCalibration::foreArmLeft], parameters.lowerArm, 1, bodyContour);
  add(theRobotModel.limbs[MassCalibration::foreArmRight], parameters.lowerArm, -1, bodyContour);
  add(theRobotModel.limbs[MassCalibration::thighLeft], parameters.upperLeg1, 1, bodyContour);
  add(theRobotModel.limbs[MassCalibration::thighRight], parameters.upperLeg1, -1, bodyContour);
  add(theRobotModel.limbs[MassCalibration::thighLeft], parameters.upperLeg2, 1, bodyContour);
  add(theRobotModel.limbs[MassCalibration::thighRight], parameters.upperLeg2, -1, bodyContour);
  add(theRobotModel.limbs[MassCalibration::footLeft], parameters.foot, 1, bodyContour);
  add(theRobotModel.limbs[MassCalibration::footRight], parameters.foot, -1, bodyContour);
}

void BodyContourProvider::add(const Pose3D& origin, const std::vector<Vector3<> >& c, float sign,
                              BodyContour& bodyContour)
{
  Vector2<int> q1,
               q2;
  Vector3<> p1 = origin * Vector3<>(c[0].x, c[0].y * sign, c[0].z);
  bool valid1 = calculatePointInImage(p1, q1);
  if(valid1)
  {
    Vector2<> v = theImageCoordinateSystem.fromCorrectedApprox(q1);
    q1 = Vector2<int>(int(floor(v.x)), int(floor(v.y)));
  }

  for(unsigned i = 1; i < c.size(); ++i)
  {
    Vector3<> p2 = origin * Vector3<>(c[i].x, c[i].y * sign, c[i].z);
    bool valid2 = calculatePointInImage(p2, q2);
    if(valid2)
    {
      Vector2<> v = theImageCoordinateSystem.fromCorrectedApprox(q2);
      q2 = Vector2<int>(int(floor(v.x)), int(floor(v.y)));
    }

    if(valid1 && valid2 &&
       (q1.y < theCameraInfo.resolutionHeight || q2.y < theCameraInfo.resolutionHeight) &&
       (q1.x >= 0 || q2.x >= 0) &&
       (q1.x < theCameraInfo.resolutionWidth || q2.x < theCameraInfo.resolutionWidth))
      bodyContour.lines.push_back(BodyContour::Line(q1, q2));

    q1 = q2;
    valid1 = valid2;

    COMPLEX_DRAWING3D("module:BodyContourProvider:contour",
    {
      LINE3D("module:BodyContourProvider:contour", p1.x, p1.y, p1.z, p2.x, p2.y, p2.z, 1, ColorRGBA(255, 0, 0));
      p1 = p2;
    });
  }
}

bool BodyContourProvider::calculatePointInImage(const Vector3<>& pointInWorld, Vector2<int>& pointInImage) const
{
  Vector3<> pointInCamera(robotCameraMatrixInverted * pointInWorld);
  const bool valid = pointInCamera.x > 0.0f;
  if(pointInCamera.x == 0.0f)
  {
    return false;
  }
  pointInCamera *= theCameraInfo.focalLength / pointInCamera.x;
  const Vector2<> resultFloat(theCameraInfo.opticalCenter - Vector2<>(pointInCamera.y, pointInCamera.z));
  pointInImage.x = (int)(resultFloat.x + 0.5f);
  pointInImage.y = (int)(resultFloat.y + 0.5f);
  return valid;
}

MAKE_MODULE(BodyContourProvider, Perception)
