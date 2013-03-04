/**
* @file GoalPerceptor.cpp
* @author jeff
* @author moe
*/

#include "GoalPerceptor.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/Asserts.h"

ColorClasses::Color GoalPerceptor::imageColor(int x, int y)
{
  ASSERT_COORDINATES_WITHIN_IMAGE(x, y, theCameraInfo);
  const unsigned int c = theImage.image[(size_t) y][(size_t) x].color;
  return (ColorClasses::Color)theColorTable64.colorClasses[c >> 18 & 0x3f][c >> 10 & 0x3f][c >> 26 & 0x3f];
}

Vector2<> GoalPerceptor::getPointOnField(const Vector2<int> &p)
{
  const Vector2<> pc = theImageCoordinateSystem.toCorrected(p);
  Vector2<> pf;
  Geometry::calculatePointOnField((int)pc.x, (int)pc.y, theCameraMatrix, theCameraInfo, pf);
  return pf;
}

int GoalPerceptor::findBlueOrYellowRight(int x, int y, int xEnd)
{
  ASSERT_COORDINATES_WITHIN_IMAGE(x, y, theCameraInfo);
  while(x < xEnd)
  {
    const ColorClasses::Color c = imageColor(x, y);
    if(c == ColorClasses::blue || c == ColorClasses::yellow)
      return x;
    x++;
  }
  return -1;
}

int GoalPerceptor::runRight(int x, int y, ColorClasses::Color col, int xEnd, int maxSkip)
{
  ASSERT_COORDINATES_WITHIN_IMAGE(x, y, theCameraInfo);
  int lastIndex;
  for(x += maxSkip; x < xEnd; x += maxSkip)
  {
    if(imageColor(x, y) != col)
    {
      lastIndex = max(x - maxSkip, 0);
      for(--x; x > lastIndex; --x)
        if(imageColor(x, y) == col)
          break;

      if(x == lastIndex)
      {
        ++x;
        break;
      }
    }
  }

  if(x >= xEnd)
    x = xEnd - 1;

  return x;
}

int GoalPerceptor::runLeft(int x, int y, ColorClasses::Color col, int xEnd, int maxSkip)
{
  ASSERT_COORDINATES_WITHIN_IMAGE(x, y, theCameraInfo);
  int lastIndex;
  for(x -= maxSkip; x >= xEnd; x -= maxSkip)
  {
    if(imageColor(x, y) != col)
    {
      lastIndex = x + maxSkip;
      for(++x; x < lastIndex; ++x)
        if(imageColor(x, y) == col)
          break;

      if(x == lastIndex)
      {
        --x;
        break;
      }
    }
  }

  if(x < xEnd)
    x = xEnd;

  return x;
}

int GoalPerceptor::runUp(int x, int y, ColorClasses::Color col, int yEnd, int maxSkip)
{
  ASSERT_COORDINATES_WITHIN_IMAGE(x, y, theCameraInfo);
  int lastIndex;
  for(y -= maxSkip; y >= yEnd; y -= maxSkip)
  {
    if(imageColor(x, y) != col)
    {
      lastIndex = y + maxSkip;
      for(++y; y < lastIndex; ++y)
        if(imageColor(x, y) == col)
          break;

      if(y == lastIndex)
      {
        --y;
        break;
      }
    }
  }

  if(y < yEnd)
    y = yEnd;

  return y;
}

int GoalPerceptor::runDown(int x, int y, ColorClasses::Color col, int yEnd, int maxSkip)
{
  ASSERT_COORDINATES_WITHIN_IMAGE(x, y, theCameraInfo);
  int lastIndex;
  for(y += maxSkip; y < yEnd; y += maxSkip)
  {
    if(imageColor(x, y) != col)
    {
      lastIndex = max(y - maxSkip, 0);
      for(--y; y > lastIndex; --y)
        if(imageColor(x, y) == col)
          break;

      if(y == lastIndex)
      {
        ++y;
        break;
      }
    }
  }

  if(y >= yEnd)
    y = yEnd - 1;

  return y;
}

void GoalPerceptor::update(GoalPercept& percept)
{
  DECLARE_DEBUG_DRAWING("module:GoalPerceptor:findSpots", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:GoalPerceptor:width", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:GoalPerceptor:green", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:GoalPerceptor:Image", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:GoalPerceptor:Field", "drawingOnField");
  MODIFY("parameters:GoalPerceptor", parameters);

  this->percept = &percept;
  resetPercept();

  if(!theCameraMatrix.isValid)
    return;

  findSpots();
  checkSpotsWidth();
  checkGreenBelow();
  checkFieldCoordinates();
  checkMinimumDistance();
  createPercept();
}

void GoalPerceptor::findSpots()
{
  spots.reset();

  int yHorizon = theImageCoordinateSystem.fromHorizonBased(Vector2<>(0, 0)).y;

  if(yHorizon < 1)
    yHorizon = 1;
  //if(yHorizon >= theCameraInfo.resolutionHeight)
  //return;
  if(yHorizon >= theCameraInfo.resolutionHeight - 1)
    yHorizon = theCameraInfo.resolutionHeight - 2;

  ARROW("module:GoalPerceptor:findSpots", 160, yHorizon, 260, yHorizon, 1, Drawings::ps_solid, ColorRGBA(255, 255, 0));

  int x = 0;

  const int xEnd = theCameraInfo.resolutionWidth,
            yEnd = theCameraInfo.resolutionHeight;

  while(x < xEnd - 1)
  {
    x = findBlueOrYellowRight(x, yHorizon, xEnd);
    if(x == -1)
      break;
    ColorClasses::Color spotColor = imageColor(x, yHorizon);

    const int xStart = x;
    x++;

    if(x >= xEnd)
      break;

    x = runRight(x, yHorizon, spotColor, xEnd, parameters.maxSkip);
    const int xSegEnd = x;

    if(xSegEnd - xStart < parameters.minSegmentLength)
      continue;

    LINE("module:GoalPerceptor:findSpots", xStart, yHorizon, xSegEnd, yHorizon, 1, Drawings::ps_solid, ColorRGBA(0, 0, 255));

    //run down
    int lastYRunEnd;
    int yRunEnd = yHorizon;
    int xMid = (xStart + xSegEnd) / 2;
    for(;;)
    {
      lastYRunEnd = yRunEnd;
      yRunEnd = runDown(xMid, yRunEnd, spotColor, yEnd, parameters.maxSkip);
      LINE("module:GoalPerceptor:findSpots", xMid, lastYRunEnd, xMid, yRunEnd, 1, Drawings::ps_solid, ColorRGBA(0, 255, 255));

      if(yRunEnd - lastYRunEnd < parameters.minYRunDiff)
        break;

      const int xS = runLeft(xMid, yRunEnd - 1, spotColor, 0, parameters.maxSkip);
      const int xR = runRight(xMid, yRunEnd - 1, spotColor, xEnd, parameters.maxSkip);
      LINE("module:GoalPerceptor:findSpots", xMid, yRunEnd - 1, xS, yRunEnd - 1, 1, Drawings::ps_solid, ColorRGBA(0, 255, 255));
      LINE("module:GoalPerceptor:findSpots", xMid, yRunEnd - 1, xR, yRunEnd - 1, 1, Drawings::ps_solid, ColorRGBA(0, 255, 255));
      xMid = (xS + xR) / 2;
    }
    yRunEnd--;

    //run up
    int yRunEndUp = yHorizon;
    int xMidUp = (xStart + xSegEnd) / 2;
    for(;;)
    {
      lastYRunEnd = yRunEndUp;
      yRunEndUp = runUp(xMidUp, yRunEndUp, spotColor, 0, parameters.maxSkip);
      LINE("module:GoalPerceptor:findSpots", xMidUp, lastYRunEnd, xMidUp, yRunEndUp, 1, Drawings::ps_solid, ColorRGBA(0, 0, 255));

      if(yRunEndUp - lastYRunEnd < parameters.minYRunDiff)
        break;

      const int xS = runLeft(xMidUp, yRunEndUp + 1, spotColor, 0, parameters.maxSkip);
      const int xR = runRight(xMidUp, yRunEndUp + 1, spotColor, xEnd, parameters.maxSkip);
      LINE("module:GoalPerceptor:findSpots", xMidUp, yRunEndUp + 1, xS, yRunEndUp + 1, 1, Drawings::ps_solid, ColorRGBA(0, 0, 255));
      LINE("module:GoalPerceptor:findSpots", xMidUp, yRunEndUp + 1, xR, yRunEndUp + 1, 1, Drawings::ps_solid, ColorRGBA(0, 0, 255));
      xMidUp = (xS + xR) / 2;
    }
    yRunEndUp++;

    if(yRunEnd - yRunEndUp < parameters.minPostPixelHeight)
    {
      ARROW("module:GoalPerceptor:Image", xMidUp, yRunEndUp , (xMid + xMidUp) / 2, (yRunEnd + yRunEndUp) / 2, 2, Drawings::ps_dot, ColorRGBA(255, 0, 0));
      ARROW("module:GoalPerceptor:Image", xMid, yRunEnd , (xMid + xMidUp) / 2, (yRunEnd + yRunEndUp) / 2, 2, Drawings::ps_dot, ColorRGBA(255, 0, 0));
      continue;
    }


    LINE("module:GoalPerceptor:findSpots", 0, parameters.minHeadVisibleOffset, theCameraInfo.resolutionWidth, parameters.minHeadVisibleOffset, 1, Drawings::ps_solid, ColorRGBA(255, 0, 0));
    LINE("module:GoalPerceptor:findSpots", 0, theCameraInfo.resolutionHeight - parameters.minHeadVisibleOffset, theCameraInfo.resolutionWidth, theCameraInfo.resolutionHeight - parameters.minHeadVisibleOffset, 1, Drawings::ps_solid, ColorRGBA(255, 0, 0));

    const bool headVisible = yRunEndUp > parameters.minHeadVisibleOffset,
               footVisible = theCameraInfo.resolutionHeight - yRunEnd > parameters.minHeadVisibleOffset;

    if(!footVisible)
    {
      ARROW("module:GoalPerceptor:Image", xMidUp, yRunEndUp, xMidUp + (xMid - xMidUp) * 1.2, yRunEndUp + (yRunEnd - yRunEndUp) * 1.2, 2, Drawings::ps_dot, ColorRGBA(255, 0, 0));
      continue;
    }

    LINE("module:GoalPerceptor:findSpots", xMid, yRunEnd, xMidUp, yRunEndUp, 2, Drawings::ps_solid, ColorRGBA(255, 0, 255));
    Spot& spot = spots.getNew();
    spot.head.x = (float) xMidUp;
    spot.head.y = (float) yRunEndUp;
    spot.foot.x = (float) xMid;
    spot.foot.y = (float) yRunEnd;
    spot.headVisible = headVisible;
    spot.footVisible = footVisible;
    spot.color = spotColor;
    spot.dead = false;
    spot.center.x = 0;
    spot.center.y = 0;
    spot.side = Spot::unknown;
  }

  COMPLEX_DRAWING("module:GoalPerceptor:Image",
  {
    for(int i = 0; i < spots.size(); i++)
    {
      const Spot& s = spots.atConst(i);
      if(s.headVisible)
      {
        CROSS("module:GoalPerceptor:Image", s.head.x, s.head.y, 4, 1, Drawings::ps_solid, ColorRGBA(255, 0, 255));
      }
      if(s.footVisible)
      {
        CROSS("module:GoalPerceptor:Image", s.foot.x, s.foot.y, 4, 1, Drawings::ps_solid, ColorRGBA(255, 0, 255));
      }

      LINE("module:GoalPerceptor:Image", s.head.x, s.head.y, s.foot.x, s.foot.y, 2, Drawings::ps_solid, ColorRGBA(0, 255, 255));
    }
  });
}

void GoalPerceptor::checkSpotsWidth()
{
  for(int i = 0; i < spots.size(); i++)
  {
    Spot& s = spots.at(i);

    Vector2<> headToFoot(s.foot.x - s.head.x, s.foot.y - s.head.y);
    headToFoot.normalize();

    ARROW("module:GoalPerceptor:width", s.head.x, s.head.y, s.head.x + headToFoot.x * 20, s.head.y + headToFoot.y * 20, 1, Drawings::ps_solid, ColorRGBA(255, 0, 0));

    const int expectedWidth = calculateExpectedPixelWidth(Vector2<int>((int)s.foot.x, (int)s.foot.y));
    if(expectedWidth <= 0)
    {
      s.dead = true;
      continue;
    }
    LINE("module:GoalPerceptor:width", s.foot.x - expectedWidth / 2, s.foot.y, s.foot.x + expectedWidth / 2, s.foot.y, 2, Drawings::ps_solid, ColorRGBA(0, 255, 255));

    int sizeErrorCounter = 0;
    int widthCounter = 0;
    int validCounter = 0;
    Vector2<> avrgDir(0, 0);
    Vector2<> lastMid(-1, -1);
    bool crossbarRun = true;
    for(int y = int(s.head.y + parameters.widthStepSize / 2); y < s.foot.y; y += parameters.widthStepSize)
    {
      const Vector2<> mid = Vector2<>(s.head.x, s.head.y) + headToFoot * (y - s.head.y);

      ASSERT(mid.x >= 0);
      ASSERT(mid.y >= 0);
      ASSERT(mid.x < theCameraInfo.resolutionWidth);
      ASSERT(mid.y < theCameraInfo.resolutionHeight);

      const int left = runLeft((int)mid.x, (int)mid.y, s.color, 0, parameters.maxSkip) + 1;
      const int right = runRight((int)mid.x, (int)mid.y, s.color, theCameraInfo.resolutionWidth, parameters.maxSkip) - 1;
      const int width = right - left;
      const float widthErrorRatio = width / (float)expectedWidth;

      if(crossbarRun)
      {
        crossbarRun = false;
        if(widthErrorRatio > 2 * parameters.maxWidthErrorRatio)
        {
          float ltorRatio = (mid.x - left) / (right - mid.x);
          if(ltorRatio > 1.5f)  //left >> right => right post (crossbar is left of the post)
          {
            s.side = Spot::right;
            LINE("module:GoalPerceptor:width", left, mid.y, right, mid.y, 1, Drawings::ps_solid, ColorRGBA(0, 0, 255));
          }
          else if(1 / ltorRatio > 1.5f) //right >> left => left post (crossbar is right of the post)
          {
            s.side = Spot::left;
            LINE("module:GoalPerceptor:width", left, mid.y, right, mid.y, 1, Drawings::ps_solid, ColorRGBA(0, 0, 255));
          }
          else
          {
            sizeErrorCounter++;
            LINE("module:GoalPerceptor:width", left, mid.y, right, mid.y, 1, Drawings::ps_solid, ColorRGBA(255, 0, 0));
          }
        }
      }
      else if(widthErrorRatio > parameters.maxWidthErrorRatio || widthErrorRatio < parameters.minWidthErrorRatio)
      {
        sizeErrorCounter++;
        LINE("module:GoalPerceptor:width", left, mid.y, right, mid.y, 1, Drawings::ps_solid, ColorRGBA(255, 0, 0));
      }
      else
      {
        s.center.x += (left + right) / 2;
        s.center.y += y;
        s.avrgWidth += width;
        validCounter++;

        Vector2<> midNew(float((right + left) / 2), mid.y);
        if(lastMid.x != -1)
        {
          avrgDir += midNew - lastMid;
        }
        lastMid = midNew;
        LINE("module:GoalPerceptor:width", left, mid.y, right, mid.y, 1, Drawings::ps_solid, ColorRGBA(0, 255, 0));
      }
      widthCounter++;
    }
    if(sizeErrorCounter / (float)widthCounter > parameters.maxWrongSizeRatio)
    {
      s.dead = true;
      LINE("module:GoalPerceptor:Image", s.foot.x, s.foot.y, s.head.x, s.head.y, 2, Drawings::ps_dot, ColorRGBA(255, 0, 0));
      LINE("module:GoalPerceptor:Image", s.foot.x - 10, s.foot.y, s.foot.x + 10, s.foot.y, 2, Drawings::ps_dash, ColorRGBA(255, 0, 0));
      LINE("module:GoalPerceptor:Image", s.head.x - 10, s.head.y, s.head.x + 10, s.head.y, 2, Drawings::ps_dash, ColorRGBA(255, 0, 0));
      continue;
    }

    s.center /= (float)validCounter;
    s.avrgWidth /= validCounter;

    const float factor = (s.foot.y - s.head.y) / avrgDir.y;
    avrgDir *= factor;

    //correct center
    const float midY = (s.foot.y + s.head.y) / 2;
    const float yDiff = midY - s.center.y;
    const float centerCorrectionFactor = yDiff / avrgDir.y;
    s.center += avrgDir * centerCorrectionFactor;

    //improve head and foot points
    s.head = s.center - avrgDir / 2;
    s.foot = s.center + avrgDir / 2;

    //clipping to image resolution
    //Head
    if(s.head.x >= theCameraInfo.resolutionWidth)
      s.head = s.center - avrgDir / 2 * ((theCameraInfo.resolutionWidth - 2 - s.center.x) / (s.head.x - s.center.x));
    else if(s.head.x < 0)
      s.head = s.center - avrgDir / 2 * ((s.center.x - 1) / (s.center.x - s.head.x));

    if(s.head.y < 0)
      s.head = s.center - avrgDir / 2 * ((s.center.y - 1) / (s.center.y - s.head.y));
//    //This should never happen! (A goalpost whose head is under the image bottom)
//    else if (s.head.y >= theCameraInfo.resolutionHeight)
//      s.head = s.center - avrgDir/2 * ((theCameraInfo.resolutionHeight - s.center.y)/(s.head.y - s.center.y));

    //Foot
    if(s.foot.x >= theCameraInfo.resolutionWidth)
      s.foot = s.center + avrgDir / 2 * ((theCameraInfo.resolutionWidth - 2 - s.center.x) / (s.foot.x - s.center.x));
    else if(s.foot.x < 0)
      s.foot = s.center + avrgDir / 2 * ((s.center.x - 1) / (s.center.x - s.foot.x));

    if(s.foot.y >= theCameraInfo.resolutionHeight)
      s.foot = s.center + avrgDir / 2 * ((theCameraInfo.resolutionHeight - 2 - s.center.y) / (s.foot.y - s.center.y));

    LINE("module:GoalPerceptor:Image", s.center.x - avrgDir.x / 2, s.center.y - avrgDir.y / 2, s.center.x + avrgDir.x / 2, s.center.y + avrgDir.y / 2, 2, Drawings::ps_dash, ColorRGBA(255, 0, 255));
    ASSERT_INDEX_WITHIN(s.head.x, 0, theCameraInfo.resolutionWidth);
    //ASSERT(s.head.y >= 0);
    ASSERT(s.head.y < theCameraInfo.resolutionHeight);
    ASSERT_COORDINATES_WITHIN_IMAGE(s.foot.x, s.foot.y, theCameraInfo);
  }
}

void GoalPerceptor::checkGreenBelow()
{
  for(int i = 0; i < spots.size(); i++)
  {
    Spot& s = spots.at(i);

    if(s.dead)
      continue;

    int y = (int) s.foot.y;
    const int yEnd = min((int)(s.foot.y + parameters.maxGreenScan), theCameraInfo.resolutionHeight);

    ARROW("module:GoalPerceptor:green", s.foot.x, y, s.foot.x, yEnd, 2, Drawings::ps_solid, ColorRGBA(128, 255, 128));
    int greenCounter = 0;
    while(y < yEnd)
    {
      if(imageColor((int) s.foot.x, y) == ColorClasses::green)
        greenCounter++;
      y++;
    }
    if(greenCounter < parameters.minGreenBelow)
    {
      //check whether there is some green on the left and right side
      const int leftStartPoint = max(0, int(s.foot.x) - s.avrgWidth / 2);
      const int rightStartPoint = min(theCameraInfo.resolutionWidth, int(s.foot.x) + s.avrgWidth / 2);
      LINE("module:GoalPerceptor:green", leftStartPoint, s.foot.y, rightStartPoint, s.foot.y, 2, Drawings::ps_solid, ColorRGBA(128, 255, 128));
      int xEnd = max(0, (int)(leftStartPoint - parameters.maxGreenScan));
      int x = leftStartPoint;

      const int y = min((int)(s.foot.y + 2), theCameraInfo.resolutionHeight - 1);
      ARROW("module:GoalPerceptor:green", x, y, xEnd, y, 2, Drawings::ps_solid, ColorRGBA(128, 255, 128));

      int leftGreenCounter = 0;
      while(x >= xEnd)
      {
        if(imageColor(x, y) == ColorClasses::green)
          leftGreenCounter++;
        x--;
      }
      xEnd = min((int)(rightStartPoint + parameters.maxGreenScan), theCameraInfo.resolutionWidth);
      x = rightStartPoint;
      ARROW("module:GoalPerceptor:green", x, y, xEnd, y, 2, Drawings::ps_solid, ColorRGBA(128, 255, 128));
      int rightGreenCounter = 0;
      while(x < xEnd)
      {
        if(imageColor(x, y) == ColorClasses::green)
          rightGreenCounter++;
        x++;
      }
      if(leftGreenCounter < parameters.minGreenBelow || rightGreenCounter < parameters.minGreenBelow)
      {
        s.dead = true;
        CROSS("module:GoalPerceptor:Image", s.foot.x, s.foot.y, 5, 3, Drawings::ps_solid, ColorRGBA(128, 255, 128));
      }
    }
  }
}

int GoalPerceptor::calculateExpectedPixelWidth(Vector2<int> posImg)
{
  Vector2<> pos;
  Vector2<> pCorrected = theImageCoordinateSystem.toCorrected(posImg);
  if(!Geometry::calculatePointOnFieldHacked((int)pCorrected.x, (int)pCorrected.y, theCameraMatrix, theCameraInfo, pos))
    return -1;

  Vector2<> widthDir = pos;
  widthDir.rotateLeft();
  widthDir.normalize();

  const Vector2<> p1 = pos + widthDir * (float) theFieldDimensions.goalPostRadius;
  const Vector2<> p2 = pos - widthDir * (float) theFieldDimensions.goalPostRadius;

  Vector2<int> p1Img, p2Img;

  if(!Geometry::calculatePointInImage(Vector2<int>((int)p1.x, (int)p1.y), theCameraMatrix, theCameraInfo, p1Img))
    return -1;
  if(!Geometry::calculatePointInImage(Vector2<int>((int)p2.x, (int)p2.y), theCameraMatrix, theCameraInfo, p2Img))
    return -1;

  Vector2<> p1Uncor = theImageCoordinateSystem.fromCorrectedApprox(p1Img);
  Vector2<> p2Uncor = theImageCoordinateSystem.fromCorrectedApprox(p2Img);

  return (int)abs(p1Uncor.x - p2Uncor.x);
}

void GoalPerceptor::checkFieldCoordinates()
{
  for(int i = 0; i < spots.size(); i++)
  {
    Spot& s = spots.at(i);

    if(s.dead)
      continue;

    ASSERT(s.footVisible);

    Vector3<> posByHead,
              posByFoot,
              posByHeight;

    const Vector2<>& headCorrected(theImageCoordinateSystem.toCorrected(s.head)),
                   & footCorrected(theImageCoordinateSystem.toCorrected(s.foot));

    if(!Geometry::calculatePointOnField(footCorrected, 0, theCameraMatrix, theCameraInfo, posByFoot))
    {
      s.dead = true;
      CROSS("module:GoalPerceptor:Image", s.foot.x, s.foot.y, 5, 2, Drawings::ps_solid, ColorRGBA(255, 0, 0));
      LINE("module:GoalPerceptor:Image", s.foot.x, s.foot.y, s.head.x, s.head.y, 2, Drawings::ps_dot, ColorRGBA(255, 0, 0));
      continue;
    }
    CROSS("module:GoalPerceptor:Field", posByFoot.x, posByFoot.y, 100, 50, Drawings::ps_solid, ColorRGBA(0, 0, 255));
    s.onField = Vector2<>(posByFoot.x, posByFoot.y);

    if(s.headVisible)
    {
      /*
       * hmmmm this is very inaccurate
      if(!Geometry::calculatePointOnField(headCorrected, (float)theFieldDimensions.goalHeight, theCameraMatrix, theCameraInfo, posByHead))
      {
        s.dead = true;
        CROSS("module:GoalPerceptor:Image", s.head.x, s.head.y, 5, 2, Drawings::ps_solid, ColorRGBA(255,0,0));
        LINE("module:GoalPerceptor:Image", s.foot.x, s.foot.y, s.head.x, s.head.y, 2, Drawings::ps_dot, ColorRGBA(255,0,0));
        continue;
      }
      CROSS("module:GoalPerceptor:Field", posByHead.x, posByHead.y, 120, 30, Drawings::ps_solid, ColorRGBA(255,0,0));
      ARROW("module:GoalPerceptor:Field", posByFoot.x, posByFoot.y, posByHead.x, posByHead.y, 30, Drawings::ps_dot, ColorRGBA(255,0,0));
      */

      //calculateDistance by Height
      Vector2<> anglesFoot;
      Geometry::calculateAnglesForPoint(Vector2<float>(float(footCorrected.x), float(footCorrected.y)), theCameraMatrix, theCameraInfo, anglesFoot);
      Vector2<> anglesHead;
      Geometry::calculateAnglesForPoint(Vector2<float>(float(headCorrected.x), float(headCorrected.y)), theCameraMatrix, theCameraInfo, anglesHead);

      ASSERT(anglesHead.y != anglesFoot.y);

      const float /*distByHead = posByHead.abs(),*/
                  distByFoot = posByFoot.abs(),
                  distByHeight = theFieldDimensions.goalHeight / (tan(anglesHead.y) - tan(anglesFoot.y));

      posByHeight = posByFoot;
      posByHeight.normalize(distByHeight);

      CROSS("module:GoalPerceptor:Field", posByHeight.x, posByHeight.y, 80, 60, Drawings::ps_solid, ColorRGBA(255, 255, 0));
      ARROW("module:GoalPerceptor:Field", posByHeight.x, posByHeight.y, posByFoot.x, posByFoot.y, 20, Drawings::ps_solid, ColorRGBA(255, 255, 0));

      if(abs(distByHeight - distByFoot) / min<>(distByFoot, distByHeight) > parameters.maxDistanceError)
      {
        s.dead = true;
        CROSS("module:GoalPerceptor:Image", s.head.x, s.head.y, 5, 2, Drawings::ps_solid, ColorRGBA(255, 0, 0));
        CROSS("module:GoalPerceptor:Image", s.foot.x, s.foot.y, 5, 2, Drawings::ps_solid, ColorRGBA(255, 0, 0));
        LINE("module:GoalPerceptor:Image", s.foot.x, s.foot.y, s.head.x, s.head.y, 2, Drawings::ps_dot, ColorRGBA(255, 0, 0));
        continue;
      }

      s.onField = Vector2<>(posByHeight.x, posByHeight.y);
    }
    else
    {
      //head not visible, check whether the head is not above
      //a post of the given size at the position of the foot

      //calculateDistance by Height
      Vector2<> anglesFoot;
      Geometry::calculateAnglesForPoint(Vector2<float>(float(footCorrected.x), float(footCorrected.y)), theCameraMatrix, theCameraInfo, anglesFoot);
      Vector2<> anglesHead;
      Geometry::calculateAnglesForPoint(Vector2<float>(float(headCorrected.x), float(headCorrected.y)), theCameraMatrix, theCameraInfo, anglesHead);

      ASSERT(anglesHead.y != anglesFoot.y);

      const float distByFoot = posByFoot.abs(),
                  distByHeight = theFieldDimensions.goalHeight / (tan(anglesHead.y) - tan(anglesFoot.y));

      posByHeight = posByFoot;
      posByHeight.normalize(distByHeight);

      CROSS("module:GoalPerceptor:Field", posByHeight.x, posByHeight.y, 120, 30, Drawings::ps_solid, ColorRGBA(255, 0, 0));

      if(distByFoot - distByHeight > parameters.maxHeadCloserThanFoot)
      {
        s.dead = true;
        ARROW("module:GoalPerceptor:Image", s.foot.x, s.foot.y, s.head.x, s.head.y, 2, Drawings::ps_dot, ColorRGBA(255, 0, 0));
        continue;
      }
    }
  }
}

void GoalPerceptor::checkMinimumDistance()
{
  for(int i = 0; i < spots.size(); i++)
  {
    Spot& s = spots.at(i);

    if(s.dead)
      continue;

    //minimum post distance
    int lastAliveIdx = i - 1;
    while(lastAliveIdx >= 0 && spots.at(lastAliveIdx).dead)
      lastAliveIdx--;

    if(lastAliveIdx >= 0)
    {
      Spot& lastSpot = spots.at(lastAliveIdx);

      if(abs(s.center.x - lastSpot.center.x) < parameters.minCenterDistance)
      {
        s.dead = true;
        LINE("module:GoalPerceptor:Image", s.foot.x, s.foot.y, s.head.x, s.head.y, 2, Drawings::ps_dot, ColorRGBA(255, 0, 0));
        CROSS("module:GoalPerceptor:Image", s.center.x, s.center.y, 5, 3, Drawings::ps_solid, ColorRGBA(255, 0, 0));
        continue;
      }
    }
  }
}

void GoalPerceptor::resetPercept()
{
  percept->goalPosts.clear();
}

void GoalPerceptor::createPercept()
{
  for(int i = 0; i < spots.size(); i++)
  {
    Spot& s = spots.at(i);

    if(!s.dead)
    {
      GoalPost p;

      if(s.side == Spot::left)
        p.position = GoalPost::IS_LEFT;
      else if(s.side == Spot::right)
        p.position = GoalPost::IS_RIGHT;
      else
        p.position = GoalPost::IS_UNKNOWN;

      p.positionInImage = Vector2<int>((int) s.foot.x, (int) s.foot.y);
      p.positionOnField = Vector2<int>((int) s.onField.x, (int) s.onField.y);

      percept->goalPosts.push_back(p);
    }
  }

  if(percept->goalPosts.size() > 2) // There can only be two posts.
    percept->goalPosts.clear();     // If there are more, ignore them all.
  else if(percept->goalPosts.size() == 2)
  {
    // If we see two posts, the first one is left, the second is right
    GoalPost& left = percept->goalPosts.at(0);
    GoalPost& right = percept->goalPosts.at(1);

    if(left.position == GoalPost::IS_UNKNOWN)
      left.position = GoalPost::IS_LEFT;
    if(right.position == GoalPost::IS_UNKNOWN)
      right.position = GoalPost::IS_RIGHT;
    
    if(left.position != GoalPost::IS_LEFT || right.position != GoalPost::IS_RIGHT ||
       left.positionInImage.x >= right.positionInImage.x)
      percept->goalPosts.clear(); // Inconsistent state -> remove posts.
  }

  if(percept->goalPosts.size())
  {
    percept->timeWhenGoalPostLastSeen = theFrameInfo.time;
    if(percept->goalPosts.size() == 2)
      percept->timeWhenCompleteGoalLastSeen = theFrameInfo.time;
  }
  else
  {
    for (int i = 0; i<theRegionPercept.regionsCounter; i++)
    {
      const RegionPercept::Region& region = theRegionPercept.regions[i];
      if (region.childs.size() == 0)
        continue;
      if (region.color == ColorClasses::yellow)
      {
        int min = region.childs.at(0)->x;
        int max = region.childs.at(0)->x;
        for (unsigned int j = 0; j < region.childs.size(); j++) {
          if (region.childs.at(j)->x < min)
            min = region.childs.at(j)->x;
          else if (region.childs.at(j)->x > max)
            max = region.childs.at(j)->x;
        }
        int diff = max - min;
        if (diff > (theCameraInfo.resolutionWidth / 6)) {
          int y = region.min_y;
          LINE("module:GoalPerceptor:Image", min, y, max, y, 1, Drawings::ps_solid, ColorClasses::black);
          if (topYellow(diff)) {
            GoalPost p;
            p.position = GoalPost::IS_UNKNOWN;
            Vector2<> angles;
            float dist = Geometry::getDistanceBySize(theCameraInfo, (float)(theFieldDimensions.goalPostRadius * 2), (float)diff);
            Geometry::calculateAnglesForPoint(Vector2<>((float)(max - (diff / 2)), (float)y), theCameraMatrix, theCameraInfo, angles);
            p.positionInImage = Vector2<int>(max - (diff / 2), theImage.resolutionHeight);
            p.positionOnField = Vector2<int>((int)(cos(angles.x) * dist), (int)(sin(angles.x) * dist));
            percept->goalPosts.push_back(p);
          }
        }
      }
    }
  }
}

bool GoalPerceptor::topYellow(int minWidth)
{
  int num = 0; 
  for (int i = 0; i < theCameraInfo.resolutionWidth; i++) {
    if (getColor(theImage.image[3][i].color) == ColorClasses::yellow)
    {
      num++;
    }
  }
  return num >= minWidth;
}

ColorClasses::Color GoalPerceptor::getColor(int color)
{
  return (ColorClasses::Color)theColorTable64.colorClasses[color >> 18 & 0x3f][color >> 10 & 0x3f][color >> 26 & 0x3f];
}

MAKE_MODULE(GoalPerceptor, Perception)
