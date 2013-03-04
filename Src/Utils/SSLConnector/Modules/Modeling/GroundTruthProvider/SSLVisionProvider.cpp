#include "SSLVisionProvider.h"

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

SSLVisionProvider::~SSLVisionProvider()
{
  delete sock;
}

void SSLVisionProvider::init()
{
  framesWithoutConnection = 0;
  sock = new UdpComm();
  sock->setBlocking(false);
  if(!sock->bind("224.5.23.2", 10002))
  {
    OUTPUT(idText, text, "Bind failed.");
    ASSERT(false);
  }

  if(!sock->joinMulticast("224.5.23.2"))
  {
    OUTPUT(idText, text, "could not set multicast address");
  }
}

void SSLVisionProvider::update(SSLVisionData& sslVisionData)
{
  OUTPUT(idText, text, "retrieving SSL vision data");
  if(!retrieveFrame())
  {
    OUTPUT(idText, text, "failed");
    framesWithoutConnection++;
    if(framesWithoutConnection > 25)
      Framework::getInstance("SSLConnector")->requestShutdown = true;
    return;
  }

  OUTPUT(idText, text, "successful");
  framesWithoutConnection = 0;

  SSLVisionData::SSLVisionFrame result;
  processFrame(result);
  processRobots(result);
  processBalls(result);
  sslVisionData.recentData.add(result);

  TEAM_OUTPUT(idSSLVisionData, bin, sslVisionData);
}

bool SSLVisionProvider::retrieveFrame()
{
  const int readBufSize = 1024;
  char readBuf[readBufSize];
  int lastLen = 0;

  for(;;)
  {
    const int len = sock->read(readBuf, readBufSize);
    if(len == -1)
    {
      if(errno == EAGAIN)
        break; // no data with O_NONBLOCK flag
      OUTPUT(idText, text, "read failed");
      return false;
    }
    lastLen = len;
  }

  SSL_WrapperPacket packet;
  packet.ParseFromArray(readBuf, lastLen);
  if(!packet.has_detection())
    return false;

  currentFrame = packet.detection();
  return true;
}

void SSLVisionProvider::processFrame(SSLVisionData::SSLVisionFrame& result)
{
  result.timestamp = currentFrame.t_capture();
  result.receiveTimestamp = SystemCall::getCurrentSystemTime();
}

void SSLVisionProvider::processRobots(SSLVisionData::SSLVisionFrame& result)
{
  int yellowSize = currentFrame.robots_yellow_size();
  result.yellowRobots.resize(yellowSize);
  result.yellowRobotIds.resize(yellowSize);
  for(int i = 0; i < yellowSize; i++)
  {
    const SSL_DetectionRobot& robot = currentFrame.robots_yellow(i);
    float orientation = 0.0;
    if(robot.has_orientation())
      orientation = robot.orientation();
    Pose2D pose(orientation, robot.x(), robot.y());
    result.yellowRobots[i] = pose;
    result.yellowRobotIds[i] = (int) robot.robot_id();
  }
  int blueSize = currentFrame.robots_blue_size();
  result.blueRobots.resize(blueSize);
  result.blueRobotIds.resize(blueSize);
  for(int i = 0; i < blueSize; i++)
  {
    const SSL_DetectionRobot& robot = currentFrame.robots_blue(i);
    float orientation = 0.0;
    if(robot.has_orientation())
      orientation = robot.orientation();
    Pose2D pose(orientation, robot.x(), robot.y());
    result.blueRobots[i] = pose;
    result.blueRobotIds[i] = (int) robot.robot_id();
  }
}

void SSLVisionProvider::processBalls(SSLVisionData::SSLVisionFrame& result)
{
  float confidence = 0.0f;
  for(int i = 0; i < currentFrame.balls_size(); i++)
  {
    const SSL_DetectionBall& ball = currentFrame.balls(i);
    if(ball.confidence() < confidence)
      continue;

    confidence = ball.confidence();
    lastSeenBall = Vector2<>(ball.x(), ball.y());
  }
  result.ball = lastSeenBall;
}

MAKE_MODULE(SSLVisionProvider, Modeling)
