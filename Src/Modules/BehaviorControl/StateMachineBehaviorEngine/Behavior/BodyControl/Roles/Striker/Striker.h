#include <StateMachineBehavior>

option Striker
{
  state start()
  {
    decision
    {
      if(stateTime > 100)
        return turnToBall;
    }
    action
    {
      theMotionRequest.motion = MotionRequest::stand;
      HeadControl.mode =  HeadControl.lookUp;
    }
  }

  state turnToBall()
  {
    decision
    {
      /*if(ball.time_since_last_seen > 7000)
        goto search_for_ball;
      else if(abs( value = ball.angle ) < 5)
        goto walk_to_ball;*/
    }
    action
    {
      theMotionRequest.motion = MotionRequest::walk;
      theMotionRequest.walkRequest.mode = WalkRequest::targetMode;
      theMotionRequest.walkRequest.target = Pose2D(theBallModel.estimate.getAngle(), 0.0, 0.0); 
      theMotionRequest.walkRequest.speed = Pose2D(0.0, 50.0, 50.0);	
      HeadControl.mode =  HeadControl.lookUpAndDown;
    }
  }

};
