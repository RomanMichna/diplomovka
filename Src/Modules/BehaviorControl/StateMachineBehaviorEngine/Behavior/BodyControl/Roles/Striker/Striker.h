#include <StateMachineBehavior>

option Striker
{
  state start()
  {
    decision
    {
      if(stateTime > 1000)
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
      ballAngle = getBallAngle();
      if(timeSinceBallWasSeen() > 7000)
        return searchForBall;
      if(abs(ballAngle) < 5)
        return walkToBall;
    }
    action
    {
      ballAngle = getBallAngle();
      theMotionRequest.motion = MotionRequest::walk;
      theMotionRequest.walkRequest.mode = WalkRequest::targetMode;
      theMotionRequest.walkRequest.target = Pose2D(ballAngle); 
      theMotionRequest.walkRequest.speed = Pose2D(50.0, 50.0, 50.0);	
      HeadControl.mode =  HeadControl.lookUpAndDown;
    }
  }

  state walkToBall()
  {
    decision
    {
      if(timeSinceBallWasSeen() > 7000)
        return searchForBall;
      else if(theBallModel.estimate.getDistance() < 500)
        return alignToGoal;
    }
    action
    {
      ballAngle = getBallAngle(); 
      theMotionRequest.motion = MotionRequest::walk;
      theMotionRequest.walkRequest.mode = WalkRequest::targetMode;
      theMotionRequest.walkRequest.target = Pose2D(ballAngle, theBallModel.estimate.position[0], theBallModel.estimate.position[1]); 
      theMotionRequest.walkRequest.speed = Pose2D(0.0, 100.0, 100.0);	
      HeadControl.mode = stateTime % 6000 < 3000 ? HeadControl.lookAtBall : HeadControl.lookUpAndDown;
    }
  }

  state alignToGoal()
  {
    decision
    {
      centerAngleOfFreePart = getCenterAngleOfFreePart();
      if(timeSinceBallWasSeen() > 7000)
        return searchForBall;
      else if(abs(centerAngleOfFreePart) < 10 && abs(theBallModel.estimate.position[1]) < 100)
        return alignBehindBall;
    }
    action
    {
      centerAngleOfFreePart = getCenterAngleOfFreePart();
      theMotionRequest.motion = MotionRequest::walk;
      theMotionRequest.walkRequest.mode = WalkRequest::targetMode;
      theMotionRequest.walkRequest.target = Pose2D(centerAngleOfFreePart, theBallModel.estimate.position[0] - 400, theBallModel.estimate.position[1]); 
      theMotionRequest.walkRequest.speed = Pose2D(0.0, 100.0, 100.0);	
      HeadControl.mode = stateTime % 4000 < 2000 ? HeadControl.lookUpAndDown : HeadControl.lookAtBall;
    }
  }

  state alignBehindBall()
  {
    decision
    {
      if(timeSinceBallWasSeen() > 7000)
        return searchForBall;
      else if(between(theBallModel.estimate.position[1], 20, 50 ) && between(theBallModel.estimate.position[0], 140, 170 ))
        return checkPose;
    }
    action
    {
      centerAngleOfFreePart = getCenterAngleOfFreePart();
      theMotionRequest.motion = MotionRequest::walk;
      theMotionRequest.walkRequest.mode = WalkRequest::targetMode;
      theMotionRequest.walkRequest.target = Pose2D(centerAngleOfFreePart,theBallModel.estimate.position[0]-150,theBallModel.estimate.position[1]-30);
      theMotionRequest.walkRequest.speed = Pose2D(80.0, 80.0, 80.0);
      HeadControl.mode = HeadControl.lookAtBall;
    }
  }

  state checkPose()
  {
    decision
    {
      bool opponentGoalWasSeen = goalWasSeen();
      centerAngleOfFreePart = getCenterAngleOfFreePart();

      if(timeSinceBallWasSeen() > 7000)
        return searchForBall;
      else if(stateTime > 2000 && (opponentGoalWasSeen || stateTime > 5000) && abs(centerAngleOfFreePart) < 2)
      	return start;
    }
    action
    {
      centerAngleOfFreePart = getCenterAngleOfFreePart();
      theMotionRequest.motion = MotionRequest::walk;
      theMotionRequest.walkRequest.mode = WalkRequest::targetMode;
      theMotionRequest.walkRequest.target = Pose2D(centerAngleOfFreePart,theBallModel.estimate.position[0]-150,theBallModel.estimate.position[1]-30);
      theMotionRequest.walkRequest.speed = Pose2D(0.0, 50.0, 50.0);
      HeadControl.mode = HeadControl.lookUpAndDown;
    }
  }


  state searchForBall()
  {
    decision
    {
      if(ballWasSeen())
        return turnToBall;
    }
    action
    {
      float rotation = theBallModel.estimate.position[1] < 0 ? -360 : 360;
      theMotionRequest.motion = MotionRequest::walk;
      theMotionRequest.walkRequest.target = Pose2D(rotation, 0, 0);
      theMotionRequest.walkRequest.speed = Pose2D(50.0, 50.0, 50.0);
      HeadControl.mode = HeadControl.lookUpAndDown;
    }
  }
/*
state prepare_kick      //FCM
  {
    decision
    {
      if(motion.kick_forward( x = ball.x, y = ball.y, mirror = false, updates = true))
        goto kick;
      else
        stay;
    }
    action
    {
      motion.type = stand;
      motion.bike.mirror = true;
    }
  }
*/
/*
  state kick
  {
    decision
    {
      if(state_time > 3000)
        goto start;
      else
        stay;
    }
    action
    {
      motion.type = bike;
    }
  }
}*/

  private:
     float ballAngle;
     float centerAngleOfFreePart;
   
     float getCenterAngleOfFreePart()
     {
       Pose2D poseForOppGoalAngle = theRobotPose;
       Vector2<> leftOppGoalPostAbs  = Vector2<>((float) theFieldDimensions.xPosOpponentGroundline, (float) theFieldDimensions.yPosLeftGoal);
       Vector2<> rightOppGoalPostAbs = Vector2<>((float) theFieldDimensions.xPosOpponentGroundline, (float) theFieldDimensions.yPosRightGoal);

       if(poseForOppGoalAngle.translation.x > float(theFieldDimensions.xPosOpponentGroundline - 50))
       {
         poseForOppGoalAngle.translation.x = float(theFieldDimensions.xPosOpponentGroundline - 50);
       }

       float leftOppGoalPostAngle  = Geometry::angleTo(poseForOppGoalAngle, leftOppGoalPostAbs);
       float rightOppGoalPostAngle = Geometry::angleTo(poseForOppGoalAngle, rightOppGoalPostAbs);
 
       if(leftOppGoalPostAngle < rightOppGoalPostAngle)
       {
          leftOppGoalPostAngle += pi2;
       }

       Vector2<> centerOfFreePart = (theFreePartOfOpponentGoalModel.leftEnd + theFreePartOfOpponentGoalModel.rightEnd) / 2.0f;     
  
       float angleToCenterOfFreePart = Range<>(rightOppGoalPostAngle - getAngleToleranceToFreePart(), leftOppGoalPostAngle +    	  		getAngleToleranceToFreePart()).limit(centerOfFreePart.angle());
      
       return toDegrees(normalize(angleToCenterOfFreePart));
    }

    float getAngleToleranceToFreePart()
    {
       return max(10.0f, getOpeningAngleOfFreePart() - 10.0f);
    }    

    float getOpeningAngleOfFreePart()
    {
       float angleToLeftEnd  = theFreePartOfOpponentGoalModel.leftEnd.angle();
       float angleToRightEnd = theFreePartOfOpponentGoalModel.rightEnd.angle();

       if(angleToLeftEnd < angleToRightEnd)
       {
           angleToLeftEnd += pi2;
       }

       return toDegrees(abs(normalize(angleToLeftEnd - angleToRightEnd)));
   }

    float getBallAngle()
    {
	return toDegrees(theBallModel.estimate.getAngle());
    }

    bool between(float number, float min, float max)
    { 
        if(number > min && number < max)
          return true;
        else
          return false;
    }

    float timeSinceBallWasSeen()
    {
       return (float)theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen);
    }

    bool goalWasSeen()
    {
       float timeSinceOppGoalWasSeen = (float) theFrameInfo.getTimeSince(theGoalPercept.timeWhenGoalPostLastSeen);
       return timeSinceOppGoalWasSeen < 500;   
    }

    bool ballWasSeen()
    {
       float timeSinceBallWasSeen = (float) theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen);
       return timeSinceBallWasSeen < 500;
    }
};
