/**
* @file CameraControlEngine.cpp
* @author Felix Wenk
*/

#include "Tools/Streams/InStreams.h"
#include "Tools/Math/Geometry.h"
#include "Representations/Configuration/MassCalibration.h"

#include "CameraControlEngine.h"

MAKE_MODULE(CameraControlEngine, Behavior Control);

CameraControlEngine::CameraControlEngine()
  : lastRequestedLowerCamera(true), requestedLowerCamera(true), lowerHidden(false),
    minPan(fromDegrees(-119.0f)), maxPan(fromDegrees(119.0f)),
    boundTolerance(fromDegrees(2.5f)), lowerBound(500.0f), range(2000.0f), slope(3.0f), maxTiltOffsetFactor(0.7f),
    upperReachable(true), lowerReachable(true), upperClippedAngle(0.0f), lowerClippedAngle(0.0f)
{
  InConfigMap parameterStream("cameraControlEngine.cfg");
  if(parameterStream.exists())
    parameterStream >> p;
}

void CameraControlEngine::calculateTiltAngles(const Vector3<>& hip2Target, float tiltOffset, bool lowerCamera, Vector2<>& panTilt) const
{
  InverseKinematic::calcHeadJoints(hip2Target, pi_2 + tiltOffset, theRobotDimensions, lowerCamera, panTilt);
}

float CameraControlEngine::calculateTiltAngleOffset(const float groundTargetDistance) const
{
  const float exponent = -2 * slope / range * (groundTargetDistance - lowerBound - range / 2);
  const float scale = 1 / (1 + std::exp(exponent));
  return theCameraInfo.openingAngleHeight / 2.0f * maxTiltOffsetFactor * scale;
}

void CameraControlEngine::adjustTiltBoundsToShoulder(const float imageTiltOffset, const float pan,
                                                     const bool lowerCamera, Vector2<>& bounds) const
{
  MassCalibration::Limb shoulder = pan > 0.0f ? MassCalibration::shoulderLeft : MassCalibration::shoulderRight;
  const Vector3<>& shoulderVector = theRobotModel.limbs[shoulder].translation;
  RobotCameraMatrix rcm(theRobotDimensions, pan, 0.0f, theCameraCalibration, !lowerCamera);
  Vector3<> intersection;
  if(theHeadLimits.intersectionWithShoulderEdge(rcm, shoulderVector, intersection))
  {
    Vector2<> intersectionPanTilt;
    calculateTiltAngles(intersection, imageTiltOffset, lowerCamera, intersectionPanTilt);
    if(intersectionPanTilt.y > bounds.y) // if(tilt greater than lower bound)
      bounds.y = intersectionPanTilt.y;
  }
  COMPLEX_DRAWING("module:CameraControlEngine:shoulder",
  {
    if(lastRequestedLowerCamera)
    {
      Vector2<int> point;
      Geometry::calculatePointInImage(intersection, theRobotCameraMatrix,
                                      theCameraInfo, point);
      MID_DOT("module:CameraControlEngine:shoulder", point.x, point.y,
              ColorClasses::yellow, ColorClasses::yellow);
    }
  });
}

bool CameraControlEngine::isReachable(const Vector2<>& tiltBounds, const float tilt, const bool lastReachable, float& clippedAngle) const
{
  const float tolerance = lastReachable ? boundTolerance : -boundTolerance;
  clippedAngle = tilt - (tiltBounds.x + tolerance);
  if(clippedAngle > 0.0f) // Tilt larger than upper bound.
    return false;
  clippedAngle = tiltBounds.y - tolerance - tilt;
  return clippedAngle <= 0.0f; // Return false if tilt smaller than lower bound
}

void CameraControlEngine::update(HeadAngleRequest& headAngleRequest)
{
  DECLARE_DEBUG_DRAWING("module:CameraControlEngine:target", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CameraControlEngine:shoulder", "drawingOnImage");
  MODIFY("parameters:CameraControlEngine", p);
  if(p.disableUpperCamera && p.disableLowerCamera)
  {
    OUTPUT(idText, text, "CameraControlEngine: Both cameras disabled. Enabling at lower camera.");
    p.disableLowerCamera = false;
  }

  // Calculate target angles.
  float imageTiltOffset = 0.0f;
  Vector2<> targetPanTiltCurrent;
  Vector2<> targetPanTiltOther;
  if(theHeadMotionRequest.mode == HeadMotionRequest::panTiltMode)
  {
    targetPanTiltCurrent.x = theHeadMotionRequest.pan;
    targetPanTiltCurrent.y = theHeadMotionRequest.tilt - theRobotDimensions.getHeadTiltToCameraTilt(!lastRequestedLowerCamera);
    targetPanTiltOther.x = theHeadMotionRequest.pan;
    targetPanTiltOther.y = theHeadMotionRequest.tilt - theRobotDimensions.getHeadTiltToCameraTilt(lastRequestedLowerCamera);
  }
  else
  {
    Vector3<> hip2Target; // Target relative to center of hip.
    if(theHeadMotionRequest.mode == HeadMotionRequest::targetMode)
    {
      hip2Target = theHeadMotionRequest.target;
    }
    else
    {
      hip2Target = theTorsoMatrix.invert() * theHeadMotionRequest.target;
      if(theHeadMotionRequest.watchField)
        imageTiltOffset = -calculateTiltAngleOffset(theHeadMotionRequest.target.abs());
    }
    calculateTiltAngles(hip2Target, imageTiltOffset, lastRequestedLowerCamera, targetPanTiltCurrent);
    calculateTiltAngles(hip2Target, imageTiltOffset, !lastRequestedLowerCamera, targetPanTiltOther);
  }

  MID_DOT("module:CameraControlEngine:target", theCameraInfo.opticalCenter.x,
          theCameraInfo.opticalCenter.y - theCameraInfo.focalLength * tan(-imageTiltOffset),
          ColorClasses::robotBlue, ColorClasses::robotBlue);

  // Clip pan angles.
  if(targetPanTiltCurrent.x < minPan)
  {
    targetPanTiltCurrent.x = minPan; // Pan angles are the same.
    targetPanTiltOther.x = minPan;
  }
  else if(targetPanTiltCurrent.x > maxPan)
  {
    targetPanTiltCurrent.x = maxPan;
    targetPanTiltOther.x = maxPan;
  }

  // Calculate tilt bounds.
  Vector2<> lowerCameraTiltBounds = theHeadLimits.getTiltBound(targetPanTiltCurrent.x); // Pan angles are the same for both cameras.
  Vector2<> upperCameraTiltBounds(lowerCameraTiltBounds);
  ASSERT(upperCameraTiltBounds.x != JointData::off);
  adjustTiltBoundsToShoulder(0.0f /*imageTiltOffset*/, targetPanTiltCurrent.x, true, lowerCameraTiltBounds);
  adjustTiltBoundsToShoulder(0.0f /*imageTiltOffset*/, targetPanTiltCurrent.x, false, upperCameraTiltBounds);

  // Select camera
  const Vector2<>& targetPanTiltUpper = lastRequestedLowerCamera ? targetPanTiltOther : targetPanTiltCurrent;
  const Vector2<>& targetPanTiltLower = lastRequestedLowerCamera ? targetPanTiltCurrent : targetPanTiltOther;
  upperReachable = isReachable(upperCameraTiltBounds, targetPanTiltUpper.y, upperReachable, upperClippedAngle);
  lowerReachable = isReachable(lowerCameraTiltBounds, targetPanTiltLower.y, lowerReachable, lowerClippedAngle);
  if(p.disableLowerCamera)
  {
    requestedLowerCamera = false;
  }
  else if(p.disableUpperCamera)
  {
    requestedLowerCamera = true;
  }
  else if(theHeadMotionRequest.cameraControlMode == HeadMotionRequest::lowerCamera)
  {
    requestedLowerCamera = true;
  }
  else if(theHeadMotionRequest.cameraControlMode == HeadMotionRequest::upperCamera)
  {
    requestedLowerCamera = false; // The rest is only reached if cameraControlMode is autoCamera.
  }
  else if(upperReachable)
  {
    requestedLowerCamera = false;
  }
  else if(lowerReachable)
  {
    requestedLowerCamera = true;
  }
  else
  {
    requestedLowerCamera = lowerClippedAngle <= upperClippedAngle;
  }
  const Vector2<>& tiltBounds = requestedLowerCamera ? lowerCameraTiltBounds : upperCameraTiltBounds;

  // Set head angle request
  if(lastRequestedLowerCamera == requestedLowerCamera)
  {
    headAngleRequest.pan = targetPanTiltCurrent.x;
    headAngleRequest.tilt = targetPanTiltCurrent.y;
  }
  else
  {
    headAngleRequest.pan = targetPanTiltOther.x;
    headAngleRequest.tilt = targetPanTiltOther.y;
  }

  // Clip tilt angle
  if(headAngleRequest.tilt > tiltBounds.x)
    headAngleRequest.tilt = tiltBounds.x;
  else if(headAngleRequest.tilt < tiltBounds.y)
    headAngleRequest.tilt = tiltBounds.y;

  if(theHeadMotionRequest.mode == HeadMotionRequest::panTiltMode)
  {
    if(theHeadMotionRequest.tilt == JointData::off)
      headAngleRequest.tilt = JointData::off;
    if(theHeadMotionRequest.pan == JointData::off)
      headAngleRequest.pan = JointData::off;
  }
  headAngleRequest.speed = theHeadMotionRequest.speed;
  lastRequestedLowerCamera = requestedLowerCamera;
}

void CameraControlEngine::update(ImageRequest& imageRequest)
{
  DECLARE_PLOT("module:CameraControlEngine:imageReqest");
  PLOT("module:CameraControlEngine:imageReqest", requestedLowerCamera ? 2 : 1);

  imageRequest.requestedCamera = p.toggleCameras ? ImageInfo::toggleCameras :
                                 requestedLowerCamera ? ImageInfo::lowerCamera : ImageInfo::upperCamera;
}

void CameraControlEngine::init()
{
  minPan = theHeadLimits.minPan();
  maxPan = theHeadLimits.maxPan();
}
