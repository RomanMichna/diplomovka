#include <StateMachineBehavior>

option Stand
{
  state stand()
  {
    action
    {
      theMotionRequest.motion = MotionRequest::stand;
    }
  }
};