#ifndef SSLVisionProvider_H
#define SSLVisionProvider_H

#include "Tools/Module/Module.h"
#include "Representations/Modeling/SSLVisionData.h"

#include "messages_robocup_ssl_detection.pb.h"

MODULE(SSLVisionProvider)
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(SSLVisionData)
END_MODULE

class UdpComm;
class SSL_DetectionFrame;

class SSLVisionProvider : public SSLVisionProviderBase
{
  ~SSLVisionProvider();

  void init();

  void update(SSLVisionData& sslVisionData);

  /** Retrieves the last SSL vision frame via UDP. */
  bool retrieveFrame();
  /** Extracts general information from the frame. */
  void processFrame(SSLVisionData::SSLVisionFrame& result);
  /** Extracts information about the robot poses. */
  void processRobots(SSLVisionData::SSLVisionFrame& result);
  /** Extracts information about the ball position. */
  void processBalls(SSLVisionData::SSLVisionFrame& result);

  UdpComm* sock;
  SSL_DetectionFrame currentFrame;
  Vector2<> lastSeenBall;

  unsigned int framesWithoutConnection;
};

#endif
