/**
 * This Representation contains all the information provided by the SSL vision system.
 * @author <a href="afabisch@tzi.de">Alexander Fabisch</a>
 */

#pragma once

#include "Tools/LimitedPriorityQueue.h"
#include "Tools/Math/Pose2D.h"
#include <vector>


class SSLVisionData : public Streamable
{
  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(recentData);
    STREAM_REGISTER_FINISH;
  }
public:
  class SSLVisionFrame : public Streamable
  {
    void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN;
      STREAM(timestamp);
      STREAM(receiveTimestamp);
      STREAM(yellowRobots);
      STREAM(yellowRobotIds);
      STREAM(blueRobots);
      STREAM(blueRobotIds);
      STREAM(ball);
      STREAM_REGISTER_FINISH;
    }

  public:
    /**
     * The capture timestamp provided by the SSL vision is obtained by
     *  #include <sys/time.h>
     *  ...
     *  timeval tv;
     *  gettimeofday(&tv,NULL);
     *  result.setTime((double)tv.tv_sec + tv.tv_usec*(1.0E-6));
     */
    double timestamp;
    /** The time when the SSLConnector received this frame. */
    unsigned receiveTimestamp;
    /** The absolute poses of the yellow patterns. */
    std::vector<Pose2D> yellowRobots;
    std::vector<int> yellowRobotIds;
    /** The absolute poses of the blue patterns. */
    std::vector<Pose2D> blueRobots;
    std::vector<int> blueRobotIds;
    /** The absolute ball position. */
    Vector2<> ball;

    /** An operator to order the frames according to the timestamps. */
    bool operator<(const SSLVisionFrame& sslVisionFrame) const
    {
      return receiveTimestamp < sslVisionFrame.receiveTimestamp;
    }
  };

  LimitedPriorityQueue<SSLVisionFrame> recentData;

  void draw();
};
