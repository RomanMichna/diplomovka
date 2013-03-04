
#include <string>
#include <vector>

#include "StateMachineBehavior.h"
#include "Platform/Thread.h"

namespace StateMachineBehaviorMutex
{
  SyncObject mutex;
};

void StateMachineBehavior::lock()
{
  StateMachineBehaviorMutex::mutex.enter();
}

void StateMachineBehavior::unlock()
{
  StateMachineBehaviorMutex::mutex.leave();
}
