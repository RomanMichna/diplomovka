/**
* @file CameraControlEngine.h
* To execute the current HeadMotionRequest the CameraControlEngine
* chooses one of the two cameras of the Nao and computes the appropriate
* angles for the head joints to execute the current HeadMotionRequest.
* @author Felix Wenk
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Tools/InverseKinematic.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Configuration/HeadLimits.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Infrastructure/TeamMateData.h"
#include "Representations/Infrastructure/ImageInfo.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/GoalPercept.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/HeadJointRequest.h"
#include "Representations/MotionControl/HeadAngleRequest.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Representations/Perception/BodyContour.h"

MODULE(CameraControlEngine)
  REQUIRES(BodyContour)
  REQUIRES(FrameInfo)
  REQUIRES(RobotDimensions)
  REQUIRES(CameraCalibration)
  REQUIRES(CameraMatrix)
  REQUIRES(CameraMatrixOther)
  REQUIRES(CameraInfo)
  REQUIRES(FilteredJointData)
  REQUIRES(HeadJointRequest)
  REQUIRES(HeadLimits)
  REQUIRES(HeadMotionRequest)
  REQUIRES(HeadAngleRequest)
  REQUIRES(RobotCameraMatrix)
  REQUIRES(RobotCameraMatrixOther)
  REQUIRES(RobotModel)
  REQUIRES(RobotPose)
  REQUIRES(TorsoMatrix)
  PROVIDES_WITH_MODIFY_AND_OUTPUT(ImageRequest)
  PROVIDES_WITH_MODIFY(HeadAngleRequest)
END_MODULE

class CameraControlEngine : public CameraControlEngineBase
{
public:
  CameraControlEngine();

private:
  class Parameters : public Streamable
  {
  public:
    Parameters()
      : disableUpperCamera(false),
        disableLowerCamera(false),
        toggleCameras(false),
        defaultHeadSpeed(3.4f)
    {}

    bool disableUpperCamera; /** true to disable. */
    bool disableLowerCamera; /** true to disable. */
    bool toggleCameras; /**< Continiously switch between cameras. */
    float defaultHeadSpeed;
  private:
    /**
    * The method makes the object streamable.
    * @param in The stream from which the object is read.
    * @param out The stream to which the object is written.
    */
    virtual void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN;
      STREAM(disableUpperCamera);
      STREAM(disableLowerCamera);
      STREAM(toggleCameras);
      STREAM(defaultHeadSpeed);
      STREAM_REGISTER_FINISH;
    }
  };

  Parameters p;
  bool lastRequestedLowerCamera;
  bool requestedLowerCamera;
  bool lowerHidden;
  float minPan;
  float maxPan;
  const float boundTolerance;

  // Attributes to calculate tilt angle offset
  const float lowerBound;
  const float range;
  const float slope;
  const float maxTiltOffsetFactor;

  bool upperReachable;
  bool lowerReachable;
  float upperClippedAngle;
  float lowerClippedAngle;

  void calculateTiltAngles(const Vector3<>& hip2Target, float tiltOffset, bool lowerCamera, Vector2<>& panTilt) const;
  void adjustTiltBoundsToShoulder(const float imageTiltOffset,const float pan,
                                  const bool lowerCamera, Vector2<>& bounds) const;
  inline float calculateTiltAngleOffset(const float groundTargetDistance) const;
  bool isReachable(const Vector2<>& tiltBounds, const float tilt, const bool lastReachable, float& clippedAngle) const;
  void update(HeadAngleRequest& headAngleRequest);
  void update(ImageRequest& imageRequest);
  void init();
};
