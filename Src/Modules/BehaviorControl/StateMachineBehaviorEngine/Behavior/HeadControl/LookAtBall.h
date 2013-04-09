#include <StateMachineBehavior>

option LookAtBall
{
  state lookAtEstimate()
  {
    decision
    {
      if(timeSinceBallWasSeen() > 2500)
      	return lookUpAndDown;
    }
    action
    {
      theHeadMotionRequest.mode = HeadMotionRequest::targetOnGroundMode;
      theHeadMotionRequest.cameraControlMode = HeadMotionRequest::autoCamera;	
      theHeadMotionRequest.target = Vector3<>(theBallModel.estimate.position[0], theBallModel.estimate.position[1], 35.0);
      theHeadMotionRequest.speed = 80;
    }
  }

  state lookUpAndDown()
  {
    decision
    {
      if(timeSinceBallWasSeen() < 500)
        return lookAtEstimate;

    }
    action
    {
      LookUpAndDown();
    }
  }
  public:
    float timeSinceBallWasSeen()
    {
       return (float)theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen);
    }

};
