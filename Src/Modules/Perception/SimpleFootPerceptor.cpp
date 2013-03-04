/*
 * @file SimpleFootPerceptor.cpp
 * Implements a class, that finds evidenve for robot feet(robot parts on the ground)
 * in the current image.
 * Only works for feet that are in the vicinity and not too far away.
 * @author Tobias Kastner
 * @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
 */

#include "SimpleFootPerceptor.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Debugging/Asserts.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Settings.h"
#include "Platform/SoundPlayer.h"

void SimpleFootPerceptor::init()
{
  // load config for this module
  InConfigMap stream(Global::getSettings().expandLocationFilename("footPerceptor.cfg"));
  if(stream.exists())
    stream >> p;
  else
  {
    InConfigMap stream("footPerceptor.cfg");
    if(stream.exists())
      stream >> p;
  }
}

ColorClasses::Color SimpleFootPerceptor::imageColor(int x, int y)
{
  ASSERT_COORDINATES_WITHIN_IMAGE(x, y, theImage);
  const unsigned int c = theImage.image[(size_t) y][(size_t) x].color;
  return (ColorClasses::Color)theColorTable64.colorClasses[c >> 18 & 0x3f][c >> 10 & 0x3f][c >> 26 & 0x3f];
}

void SimpleFootPerceptor::calcTopWhiteRuns(std::vector<Run>& runs)
{
  ColorClasses::Color opponentColor = theOpponentTeamInfo.teamColour == TEAM_BLUE ? ColorClasses::robotBlue : ColorClasses::red;
  for(int x = 0; x < theImage.resolutionWidth; x += p.stepSize)
  {
    ColorClasses::Color color = imageColor(x, 0);
    // these colors are normally to be found on a robot in the image
    if(color == ColorClasses::white || color == ColorClasses::none || color == ColorClasses::red || color == opponentColor)
    {
      int start = x;
      int count = 0;
      int tolCount = 0;
      for(int i = x; i < theImage.resolutionWidth; i += p.stepSize)
      {
        ColorClasses::Color c = imageColor(i, 0);
        // these colors are normally not to be found on the ground/grass/field ... errm except of white ... maybe
        if(c != ColorClasses::white && c != ColorClasses::none && c != ColorClasses::red && c != opponentColor)
        {
          ++tolCount;
          if(tolCount > 1)
          {
            x = i;
            break;
          }
        }

        if(count > p.count && ((c != ColorClasses::white && c != ColorClasses::none && c != ColorClasses::red && c != opponentColor)
                               || i + p.stepSize > theImage.resolutionWidth - 1))
        {
          runs.push_back(Run(start, i));
          x = i + p.stepSize;
          break;
        }
        ++count;
      }
    }
  }
}

bool SimpleFootPerceptor::pixelAboveIsOrangeOrGreen(int x, int y)
{
  // clipping (NonLineSpot points are not necessarily inside of the image)
  if(x >= theImage.resolutionWidth)
    x = theImage.resolutionWidth - 1;
  if(x < 0)
    x = 0;

  // check if there is an orange or green pixel p.yStep pixel above the given one
  y -= p.yStep;
  if(y < 0 || y >= theImage.resolutionHeight)
    return false;

  ColorClasses::Color color = imageColor(x, y);
  return color == ColorClasses::orange || color == ColorClasses::green;
}

// associate NonLineSpots with the current runs
void SimpleFootPerceptor::respectThisSpot(LineSpots::NonLineSpot spot, std::vector<Run>& runs)
{
  for(std::vector<Run>::iterator it = runs.begin(); it != runs.end(); it++)
    if(spot.p1.x > it->start - p.runTolerance && spot.p1.x < it->stop + p.runTolerance
       && spot.p2.x > it->start && spot.p2.x < it->stop)
    {
      it->associatedSpots.push_back(spot);
      return;
    }
}

// only accept runs, that have enough NonLineSpots associated and
// runs that are not too far away from each other and also are
// associated with some NoneLineSpots
void SimpleFootPerceptor::checkRuns(std::vector<Run>& runs)
{
  std::vector<Run> backup = runs;
  runs.clear();

  /*for(size_t i = 0; i < backup.size(); i++)
    if(i+1 < backup.size())
      if(backup[i+1].start - backup[i].stop > p.runDistance && (!backup[i].associatedSpots.empty() || !backup[i+1].associatedSpots.empty()))
        return;*/

  for(std::vector<Run>::iterator it = backup.begin(); it != backup.end(); it++)
  {
    if(it->associatedSpots.size() >= p.minAssociatedSpots)
      runs.push_back(*it);
  }
}

// points on the convex hull are valid if there are enough points high enough in the image
bool SimpleFootPerceptor::pointsValid(const std::vector<Vector2<> >& points)
{
  int failCount = 0;
  for(std::vector<Vector2<> >::const_iterator it = points.begin(); it != points.end(); it++)
    if(it->y >= theImage.resolutionHeight - p.runTolerance)
      ++failCount;

  return failCount < p.failCount;

}

void SimpleFootPerceptor::update(FootPercept& theFootPercept)
{
  DECLARE_DEBUG_DRAWING("module:SimpleFootPerceptor:runs", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:SimpleFootPerceptor:beforeRuns", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:SimpleFootPerceptor:nls", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:SimpleFootPerceptor:points", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:SimpleFootPerceptor:bla", "drawingOnImage");

  MODIFY("module:SimpleFootPerceptor:parameters", p);

  // get the current head angles
  float headPitch = toDegrees(theFilteredJointData.angles[JointData::HeadPitch]);
  // get the current observed NonLineSpots count
  size_t n = theLineSpots.nonLineSpots.size();

  // head must be in a certain position and enough NonLineSpots must be in the image
  if(n >= p.numOfNls && headPitch > p.minHeadPitchAngle && headPitch < p.maxHeadPitchAngle)
  {
    std::vector<Run> runs;
    // calculate the top white runs ... if they are any
    calcTopWhiteRuns(runs);

    //no runs found -> abort
    if(runs.empty())
      goto fail;

    // associate NonLineSpots with the observed runs ... if the condition below holds
    for(std::vector<LineSpots::NonLineSpot>::const_iterator spot = theLineSpots.nonLineSpots.begin();
        spot != theLineSpots.nonLineSpots.end(); spot++)
    {
      int length = (spot->p1 - spot->p2).abs();
      if(length < p.maxNlsLength && length > p.minNlsLength
         && !pixelAboveIsOrangeOrGreen(spot->p2.x, spot->p2.y)
         && spot->p1.y < theImage.resolutionHeight - p.bottomTolerance)
        respectThisSpot(*spot, runs);
    }

    // draw the runs before sanity check
    COMPLEX_DRAWING("module:SimpleFootPerceptor:beforeRuns",
    {
      for(std::vector<Run>::const_iterator it = runs.begin(); it != runs.end(); it++)
        LINE("module:SimpleFootPerceptor:beforeRuns", it->start, 0, it->stop, 0, 5, Drawings::ps_solid, ColorClasses::green);
    });

    // checks for sanity of the observed runs
    checkRuns(runs);

    // if no run has survived ... abort
    if(runs.empty())
      goto fail;

    std::vector<Vector2<> > points;

    // extract points(Vector2<>) from the associated NonLineSpots
    for(std::vector<Run>::iterator it = runs.begin(); it != runs.end(); it++)
    {
      for(std::vector<LineSpots::NonLineSpot>::const_iterator spot = it->associatedSpots.begin();
          spot != it->associatedSpots.end(); spot++)
      {
        points.push_back(Vector2<>((float) spot->p1.x, (float) spot->p1.y));
        points.push_back(Vector2<>((float) spot->p2.x, (float) spot->p2.y));
      }
    }


    // draw the extracted points before they get changed by convex hull algorihm
    COMPLEX_DRAWING("module:SimpleFootPerceptor:points",
    {
      for(vector<Vector2<> >::const_iterator it = points.begin(); it != points.end(); ++it)
        CROSS("module:SimpleFootPerceptor:points", it->x, it->y, 10, 1, Drawings::ps_solid, ColorClasses::red);
    });

    // if there are enough points too low in the image ... abort
    if(!pointsValid(points))
      goto fail;


    theFootPercept.bodyPoints = points;
    theFootPercept.wasSeen = true;

    Vector2<> left(99999.f, -99999.f), right(-99999.f, -99999.f), nearest(99999.f, 99999.f), tmpImage;
    for(std::vector<Vector2<> >::const_iterator it = points.begin(); it != points.end(); ++it)
    {
      if(it->x < left.x)
        left = *it;
      if(it->x > right.x)
        right = *it;
      Vector2<> tmp;
      Geometry::calculatePointOnField(*it, theCameraMatrix, theCameraInfo, tmp);
      if(tmp.abs() < nearest.abs())
      {
        nearest = tmp;
        tmpImage = *it;
      }
    }

    COMPLEX_DRAWING("module:SimpleFootPerceptor:bla",
    {
      CIRCLE("module:SimpleFootPerceptor:bla", left.x, left.y, 8, 0, Drawings::ps_solid, ColorClasses::black, Drawings::bs_solid, ColorRGBA(0, 0, 0, 0));    
      CIRCLE("module:SimpleFootPerceptor:bla", right.x, right.y, 8, 0, Drawings::ps_solid, ColorClasses::black, Drawings::bs_solid, ColorRGBA(0, 0, 0, 0));    
      CIRCLE("module:SimpleFootPerceptor:bla", tmpImage.x, tmpImage.y, 16, 5, Drawings::ps_solid, ColorClasses::black, Drawings::bs_solid, ColorRGBA(0, 130, 0, 0));    
    });

    Geometry::calculatePointOnField(left, theCameraMatrix, theCameraInfo, left);
    Geometry::calculatePointOnField(right, theCameraMatrix, theCameraInfo, right);
    left = abs(left.angle()) > abs(right.angle()) ? left.normalize(nearest.abs() - 50.f) : left.normalize(nearest.abs());
    right = abs(right.angle()) > abs(left.angle()) ? right.normalize(nearest.abs() - 50.f) : right.normalize(nearest.abs());
    theFootPercept.leftPart = left;
    theFootPercept.rightPart = right;

    //draw some interesting stuff
    draw(runs, points);
    return;
  }

// there are some reasons why observation of feet may fail.
// they all end up here
fail:
  theFootPercept.wasSeen = false;
}

// draw
void SimpleFootPerceptor::draw(const std::vector<Run>& runs, const std::vector<Vector2<> >& points)
{
  COMPLEX_DRAWING("module:SimpleFootPerceptor:runs",
  {
    for(std::vector<Run>::const_iterator it = runs.begin(); it != runs.end(); it++)
    {
      LINE("module:SimpleFootPerceptor:runs", it->start, 0, it->stop, 0, 5, Drawings::ps_solid, ColorClasses::red);
      DRAWTEXT("module:SimpleFootPerceptor:runs", it->start, 0, 20, ColorClasses::black, (unsigned) it->associatedSpots.size());
    }
  });

  COMPLEX_DRAWING("module:SimpleFootPerceptor:nls",
  {
    for(std::vector<Run>::const_iterator it = runs.begin(); it != runs.end(); ++it)
      for(vector<LineSpots::NonLineSpot>::const_iterator i = it->associatedSpots.begin(); i != it->associatedSpots.end(); ++i)
        ARROW("module:SimpleFootPerceptor:nls", i->p1.x, i->p1.y, i->p2.x, i->p2.y, 2, Drawings::ps_solid, ColorClasses::black);
  });

}

void SimpleFootPerceptor::update(SimpleFootModel& theSimpleFootModel)
{
  theSimpleFootModel.isValid = theFrameInfo.getTimeSince(timeLastSeenPercept) < 3000;

  if(theFootPercept.wasSeen)
    ++seenFrames;
  else
    seenFrames = 0;

  if(seenFrames > 4) // use perception
  {
    timeLastSeenPercept = theFrameInfo.time;
    theSimpleFootModel.leftPart = theFootPercept.leftPart;
    theSimpleFootModel.rightPart = theFootPercept.rightPart;
    lastOdometryWhenFootSeen = theOdometryData;
    lastFootPercept = theFootPercept;
    /*if(seenFrames == 6)
    {
      if(abs(theFootPercept.leftPart.angle()) > abs(theFootPercept.rightPart.angle()))
        SoundPlayer::play("left.wav");
      else
        SoundPlayer::play("right.wav");
    }*/

  }
  else if(theSimpleFootModel.isValid) // propagate last perception by odometry
  {
    Pose2D odometryOffset = lastOdometryWhenFootSeen - theOdometryData;
    theSimpleFootModel.leftPart = odometryOffset * lastFootPercept.leftPart;
    theSimpleFootModel.rightPart = odometryOffset * lastFootPercept.rightPart;
  }
}

MAKE_MODULE(SimpleFootPerceptor, Perception)
