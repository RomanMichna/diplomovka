#include <StateMachineBehavior>

option HeadControl
{
public:
  state mode;

  common()
  {
    decision
    {
      return mode;
    }
  }

  state none()
  {
    action
    {
    
    }
  }

  state lookUp()
  {
    action
    {
      LookUp();
    }
  }
};