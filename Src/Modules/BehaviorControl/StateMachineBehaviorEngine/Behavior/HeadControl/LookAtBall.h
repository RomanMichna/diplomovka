#include <StateMachineBehavior>

option LookAtBall
{
  state lookAtEstimate()
  {
    decision
    {
      if(theBallModel.timeWhenLastSeen > 2500)
      	return lookUpAndDown;
    }
    action
    {
      theHeadMotionRequest.cameraControlMode = HeadMotionRequest::autoCamera;
      theHeadMotionRequest.mode = HeadMotionRequest::targetOnGroundMode;	
      theHeadMotionRequest.target = Vector3<>(theBallModel.estimate.position[0], theBallModel.estimate.position[1], 35.0);
      theHeadMotionRequest.speed = 100;
    }
  }

  state lookUpAndDown()
  {
    decision
    {
      if(theBallModel.timeWhenLastSeen < 500)
        return lookAtEstimate;

    }
    action
    {
      LookUpAndDown();
    }
  }
};