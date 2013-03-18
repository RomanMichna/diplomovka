#include <StateMachineBehavior>

option LookUpAndDown
{
  state up()
  {
    decision
    {
      if (stateTime > 700)
        return down;
    }
    action
    {
      theHeadMotionRequest.cameraControlMode = HeadMotionRequest::lowerCamera; 
      theHeadMotionRequest.mode = HeadMotionRequest::panTiltMode;
      theHeadMotionRequest.pan = fromDegrees(0);
      theHeadMotionRequest.tilt = fromDegrees(30);
      theHeadMotionRequest.speed = fromDegrees(150);
    }
  }

  state down()
  {
    decision
    {
      if (stateTime > 700)
        return up;
    }
    action
    {
      theHeadMotionRequest.cameraControlMode = HeadMotionRequest::lowerCamera; 
      theHeadMotionRequest.mode = HeadMotionRequest::panTiltMode;
      theHeadMotionRequest.pan = fromDegrees(0);
      theHeadMotionRequest.tilt = fromDegrees(-25);
      theHeadMotionRequest.speed = fromDegrees(120);
    }
  }
};