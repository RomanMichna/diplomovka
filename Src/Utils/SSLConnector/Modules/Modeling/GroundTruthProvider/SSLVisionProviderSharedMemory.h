#ifndef SSLVISIONPROVIDERSHAREDMEMORY_H
#define SSLVISIONPROVIDERSHAREDMEMORY_H

#include "Tools/Module/Module.h"
#include "Representations/Modeling/SSLVisionData.h"

#include "SSLConnector/ExchangeFormat.h"
#include "Platform/linux/SharedMemory.h"

MODULE(SSLVisionProviderSharedMemory)
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(SSLVisionData)
END_MODULE

class SSL_DetectionFrame;

class SSLVisionProviderSharedMemory : public SSLVisionProviderSharedMemoryBase
{
  virtual ~SSLVisionProviderSharedMemory();
  virtual void init();
  virtual void update(SSLVisionData& theSSLVisionData);

  /** Reads data from shared memory. */
  bool retrieveFrame();
  /** Extracts general information from the frame. */
  void processFrame(SSLVisionData::SSLVisionFrame& result);
  /** Extracts information about the robot poses. */
  void processRobots(SSLVisionData::SSLVisionFrame& result);
  /** Extracts information about the ball position. */
  void processBalls(SSLVisionData::SSLVisionFrame& result);

  unsigned framesSinceLastValidFrame;
  SharedMemory sharedMemory;
  ExchangeFormat currentFrame;
  Vector2<> lastSeenBall;

public:
  SSLVisionProviderSharedMemory();
};

#endif // SSLVISIONPROVIDERSHAREDMEMORY_H
