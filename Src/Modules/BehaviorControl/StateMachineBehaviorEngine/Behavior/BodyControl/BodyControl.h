#include <StateMachineBehavior>

option BodyControl
{
  common()
  {
    decision
    {
      if(theGameInfo.state == STATE_INITIAL)
        return playing;
      if (theGameInfo.state == STATE_FINISHED)
        return finished;
      if(theFallDownState.state != FallDownState::upright && theFallDownState.state != FallDownState::undefined)
        return standUp;
      if (theGameInfo.state == STATE_READY)
        return ready;
      if (theGameInfo.state == STATE_SET)
        return set;
      if (theGameInfo.state == STATE_PLAYING)
        return playing;
    }
  }

 state initial()
  {
    action
    {
      HeadControl.mode = HeadControl.lookUp;
      Stand();
    }
  }

  state standUp()
  {
    action
    {
      //StandUp();
      Stand();
    }
  }

  state ready()
  {
    action
    {
      HeadControl.mode = HeadControl.lookUp;
      Stand();
    }
  }

  state set()
  {
    action
    {
      HeadControl.mode = HeadControl.lookUp;
      Stand();
    }
  }

  state playing()
  {
    action
    {
      Striker();
    }
  }


  state finished()
  {
    action
    {
      HeadControl.mode = HeadControl.lookUp;
      Stand();
    }
  }
};
