#include <StateMachineBehavior>
 
option LookDown
{
  state lookDown()
  {
    action
    {
      theHeadMotionRequest.cameraControlMode = HeadMotionRequest::lowerCamera; 
      theHeadMotionRequest.mode = HeadMotionRequest::panTiltMode;
      theHeadMotionRequest.pan = fromDegrees(0);
      theHeadMotionRequest.tilt = fromDegrees(-5);
      theHeadMotionRequest.speed = fromDegrees(150);
    }
  }
};
