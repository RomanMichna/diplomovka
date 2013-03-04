#ifndef ExchangeFormat_H
#define ExchangeFormat_H

class ExchangeFormat
{
public:
  class Robot
  {
  public:
    float x, y, rot;
    int id;
  };

  class Ball
  {
  public:
    float x, y;
  };

  Robot yellow[5];
  int yellowSize;
  Robot blue[5];
  int blueSize;
  Ball ball;
  bool ballSeen;
  bool valid;
};

#endif
