#include <StateMachineBehavior>

input
{
  FallDownState theFallDownState;
  FrameInfo theFrameInfo;
  GameInfo theGameInfo;
  KeyStates theKeyStates;
  MotionInfo theMotionInfo;
  BallModel theBallModel;
  FreePartOfOpponentGoalModel theFreePartOfOpponentGoalModel;
  RobotPose theRobotPose;
  FieldDimensions theFieldDimensions;
  GoalPercept theGoalPercept;

};

output
{
  HeadMotionRequest theHeadMotionRequest;
  MotionRequest theMotionRequest;
  SoundRequest theSoundRequest;
};



option Soccer
{
  state sitOrPlayDead()
  {
    decision
    {
      if(theKeyStates.pressed[KeyStates::chest])
        return standUp;

      // skip preinitial state in simulator
#ifdef TARGET_SIM
      return standUp;
#endif
    }
    action
    {
      theMotionRequest.motion = MotionRequest::specialAction;
      theMotionRequest.specialActionRequest.specialAction = SpecialActionRequest::playDead;
    }
  }

  state standUp()
  {
    decision
    {
      if(stateTime > 100)
        return startSoccer;
    }
    action
    {
      Stand();
    }
  }

  state sitDown()
  {
    decision
    {
      if(stateTime > 2000)
        return sitOrPlayDead;
    }
    action
    {
      theMotionRequest.motion = MotionRequest::specialAction;
      theMotionRequest.specialActionRequest.specialAction = SpecialActionRequest::sitDown;
      HeadControl.mode =  HeadControl.lookUp;
    }
  }

  state startSoccer()
  {
    decision
    {
      if(theGameInfo.state == STATE_FINISHED && theKeyStates.pressed[KeyStates::chest])
        return sitDown;
      if(theGameInfo.state == STATE_INITIAL && theFrameInfo.getTimeSince(theGameInfo.timeLastPackageReceived) < 500 && theKeyStates.pressed[KeyStates::chest])
        return sitDown;
    }
    action
    {
      //BodyControl();
      HeadControl();
      Striker();	
    }
  }
};
