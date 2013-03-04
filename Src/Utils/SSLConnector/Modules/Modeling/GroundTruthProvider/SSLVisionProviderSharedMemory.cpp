#include "SSLVisionProviderSharedMemory.h"

#include "SSLConnector/Framework.h"
#include "Platform/UdpComm.h"
#include "Platform/File.h"
#include "Tools/Team.h"
#include "Tools/Settings.h"
#include "messages_robocup_ssl_detection.pb.h"
#include "messages_robocup_ssl_geometry.pb.h"
#include "messages_robocup_ssl_wrapper.pb.h"

#include <iostream>
#include <string>
#include <sstream>
#include <cerrno>

SSLVisionProviderSharedMemory::SSLVisionProviderSharedMemory()
    : framesSinceLastValidFrame(0), sharedMemory("sslvisionframe", sizeof(SSL_DetectionFrame))
{}

SSLVisionProviderSharedMemory::~SSLVisionProviderSharedMemory()
{
  sharedMemory.closeAccess();
}

void SSLVisionProviderSharedMemory::init()
{
  if(!sharedMemory.success())
  {
    OUTPUT(idText, text, "Shared memory access failed.");
    ASSERT(false);
  }
  else
	{
		OUTPUT(idText, text, "Accessed shared memory.");
	}
}

void SSLVisionProviderSharedMemory::update(SSLVisionData& sslVisionData)
{
  OUTPUT(idText, text, "Retrieving SSL vision data");
  if(!retrieveFrame())
  {
    OUTPUT(idText, text, "Failed #" << framesSinceLastValidFrame);
    if(framesSinceLastValidFrame++ >= 25)
      Framework::getInstance("SSLConnector")->requestShutdown = true;
    return;
  }

  OUTPUT(idText, text, "=== Received SSL vision data:\n"
      << "valid = " << currentFrame.valid << "\n"
      << "ballSeen = " << currentFrame.ballSeen << "\n"
      << "blue robots = " << currentFrame.blueSize << "\n"
      << "yellow robots = " << currentFrame.yellowSize << "\n"
      << "=== End");

  framesSinceLastValidFrame = 0;

  SSLVisionData::SSLVisionFrame result;
  processFrame(result);
  processRobots(result);
  processBalls(result);
  sslVisionData.recentData.add(result);

  TEAM_OUTPUT(idSSLVisionData, bin, sslVisionData);
}

bool SSLVisionProviderSharedMemory::retrieveFrame()
{
  bool success = sharedMemory.nonBlockingRead(currentFrame, 100 * 1000000); // 100 ms
  return currentFrame.valid && success;
}

void SSLVisionProviderSharedMemory::processFrame(SSLVisionData::SSLVisionFrame& result)
{
  result.timestamp = 0.0; // TODO we could set the timestamp in the ssl vision plugin
  result.receiveTimestamp = SystemCall::getCurrentSystemTime();
}

void SSLVisionProviderSharedMemory::processRobots(SSLVisionData::SSLVisionFrame& result)
{
  int yellowSize = currentFrame.yellowSize;
  result.yellowRobots.resize(yellowSize);
  result.yellowRobotIds.resize(yellowSize);
  for(int i = 0; i < yellowSize; i++)
  {
    result.yellowRobots[i] = Pose2D(currentFrame.yellow[i].rot, currentFrame.yellow[i].x, currentFrame.yellow[i].y);
    result.yellowRobotIds[i] = currentFrame.yellow[i].id;
  }
  int blueSize = currentFrame.blueSize;
  result.blueRobots.resize(blueSize);
  result.blueRobotIds.resize(blueSize);
  for(int i = 0; i < blueSize; i++)
  {
    result.blueRobots[i] = Pose2D(currentFrame.blue[i].rot, currentFrame.blue[i].x, currentFrame.blue[i].y);
    result.blueRobotIds[i] = currentFrame.blue[i].id;
  }
}

void SSLVisionProviderSharedMemory::processBalls(SSLVisionData::SSLVisionFrame& result)
{
  if(currentFrame.ballSeen)
    lastSeenBall = Vector2<>(currentFrame.ball.x, currentFrame.ball.y);
  result.ball = lastSeenBall;
}

MAKE_MODULE(SSLVisionProviderSharedMemory, Modeling)
