#include <StateMachineBehavior>
 
option LookUp
{
  state lookUp()
  {
    action
    {
      theHeadMotionRequest.cameraControlMode = HeadMotionRequest::lowerCamera; 
      theHeadMotionRequest.mode = HeadMotionRequest::panTiltMode;
      theHeadMotionRequest.pan = fromDegrees(0);
      theHeadMotionRequest.tilt = fromDegrees(30);
      theHeadMotionRequest.speed = fromDegrees(150);
    }
  }
};