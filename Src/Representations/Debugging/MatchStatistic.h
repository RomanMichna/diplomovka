/**
* @file MatchStatistic.h
* Declaration of class MatchStatistic
* @author Tobias Kastner
*/

#pragma once

#include <vector>
#include <string>
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Tools/Math/Vector2.h"
#include "Tools/Math/Pose2D.h"
#include "Tools/Streams/Streamable.h"
#include "Tools/Math/Random.h"
#include "Tools/Debugging/DebugDrawings.h"

/**
* @class ScoreInfo
*
* Information when, wherefrom and how a possibly goal was scored.
*/
class ScoreInfo : public Streamable
{
  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(kickPose);
    STREAM(timeWhenKickWasPerformed);
    STREAM(timeOfGameController);
    STREAM(mirror);
    STREAM(motion, MotionRequest);
    STREAM(bikeID, BikeRequest);
    STREAM(walkKick, WalkRequest);
    STREAM(effect);
    STREAM_REGISTER_FINISH;
  }

public:
  ENUM(Effect,
    hit,          /**< kick resulted in goal */
    wentOut,      /**< kick went out of the field */
    noGoal        /**< kick resulted in no goal */
  );

  /** Constructor */
  ScoreInfo() {};
  ScoreInfo(Pose2D pose, unsigned twkwp, unsigned timeOfGameController, MotionRequest::Motion motion,
            BikeRequest::BMotionID bikeID, Effect effect, int mirror) :
    kickPose(pose), timeWhenKickWasPerformed(twkwp), timeOfGameController(timeOfGameController),
    motion(motion), bikeID(bikeID), effect(effect), mirror(mirror) {};

  ScoreInfo(Pose2D pose, unsigned twkwp, unsigned timeOfGameController, MotionRequest::Motion motion,
            WalkRequest::KickType walkKick, Effect effect, int mirror) :
    kickPose(pose), timeWhenKickWasPerformed(twkwp), timeOfGameController(timeOfGameController),
    motion(motion), walkKick(walkKick), effect(effect), mirror(mirror) {};

  Pose2D kickPose; /**< the pose of this kick */
  unsigned timeWhenKickWasPerformed; /**< timestamp of this kick */
  unsigned timeOfGameController; /**< time left communicated by the gameController */
  MotionRequest::Motion motion; /**< what kick was performed */
  BikeRequest::BMotionID bikeID; /**< what bike kick */
  WalkRequest::KickType walkKick; /**< what walkingEngine kick */
  Effect effect; /**< what was the result of this kick */
  int mirror; /**< was this kick mirrored */
};

/**
* @class FallDownInfo
*
* Information when, how and maybe why this robot fell down
*/
class FallDownInfo : public Streamable
{
  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(position);
    STREAM(afterKick);
    STREAM(mirror);
    STREAM(specialAction, SpecialActionRequest);
    STREAM(bikeID, BikeRequest);
    STREAM(walkKick, WalkRequest);
    STREAM_REGISTER_FINISH;
  }

public:
  /** Constructor */
  FallDownInfo() {};
  FallDownInfo(Vector2<> position, bool afterKick, SpecialActionRequest::SpecialActionID specialAction,
               MotionRequest::Motion motion, BikeRequest::BMotionID bikeID, WalkRequest::KickType walkKick, bool mirror) :
    position(position), afterKick(afterKick), specialAction(specialAction), motion(motion),
    bikeID(bikeID), walkKick(walkKick), mirror(mirror) {}

  Vector2<> position; /**< where this robot was fallen */
  bool afterKick; /**< wether this fall caused by a kick */
  SpecialActionRequest::SpecialActionID specialAction; /**< standUp motion performed */
  MotionRequest::Motion motion; /**< to decide what kick was performed before falling */
  BikeRequest::BMotionID bikeID; /**< potential bike kick causing a fall */
  WalkRequest::KickType walkKick; /**< potential walk kick causing a fall */
  bool mirror; /**< was this fall causing kick mirrored */
};

/**
* @class TracePart
*
* This class represents a Pose2D on the field with additional information
* like the role at that moment.
* Needed to visualise a trace used by this robot.
*/
class TracePart : public Streamable
{
  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(position);
    STREAM(role, BehaviorData);
    STREAM_REGISTER_FINISH;
  }

public:
  /** Constructor */
  TracePart() {};
  TracePart(Vector2<> position, BehaviorData::Role role) : position(position), role(role) {};

  static const Vector2<> notValid; /**< delimeter for new traces */
  Vector2<> position; /**< position on field */
  BehaviorData::Role role; /**< role at this position */
};

class Trace : public Streamable
{
  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(trace);
    STREAM(length);
    STREAM(r);
    STREAM(g);
    STREAM(b);
    STREAM(a);
    STREAM_REGISTER_FINISH;
  }

public:
  Trace() : length(0.f)
  {
    trace.reserve(32);
    r = (unsigned char) random(256);
    g = (unsigned char) random(256);
    b = (unsigned char) random(256);
    a = (unsigned char) 180;
  };

  std::vector<TracePart> trace;
  float length;
  unsigned char r;
  unsigned char g;
  unsigned char b;
  unsigned char a;
};

/** static initialization */
const Vector2<> TracePart::notValid = Vector2<>(99999.f, 99999.f);

/**
* @class SearchForBallInfo
*
* Information where, when and how long a search took.
* Additionally this class indicates wether the ball was found while patroling
* or even if it was not even observed by this robot.
*/
class SearchForBallInfo : public Streamable
{
  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(searchPose);
    STREAM(searchTime);
    STREAM(ballWasFound);
    STREAM(patrol);
    STREAM(locatedBallPos);
    STREAM(patrolPose);
    STREAM_REGISTER_FINISH;
  }

public:
  /** Constructor */
  SearchForBallInfo() {};
  SearchForBallInfo(Pose2D searchPose, unsigned searchTime, int ballWasFound, int patrol, Vector2<> locatedBallPos, Pose2D patrolPose) :
    searchPose(searchPose), searchTime(searchTime), ballWasFound(ballWasFound), patrol(patrol), locatedBallPos(locatedBallPos), patrolPose(patrolPose) {};

  Pose2D searchPose; /**< Pose2D where the search started */
  unsigned searchTime; /** time this search took ... not saying, that a ball was finally observerd */
  int ballWasFound; /** as the name says ... */
  int patrol; /** search ended while patroling */
  Vector2<> locatedBallPos; /** where was the ball observed (if so) */
  Pose2D patrolPose; /** pose where this search might have ended while patroling */
};

/**
* @class PenaltyInfo
*
* Information why and when a robot was penalized.
*/
class PenaltyInfo : public Streamable
{
  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(forWhat);
    STREAM(secondsLeft);
    STREAM_REGISTER_FINISH;
  }

public:
  PenaltyInfo() {};
  PenaltyInfo(unsigned int forWhat, unsigned secondsLeft) : forWhat(forWhat), secondsLeft(secondsLeft) {};

  unsigned int forWhat; /**< the reason for this penalisation */
  unsigned secondsLeft; /**< seconds left for this halftime */

  static const char* getPenaltyName(unsigned int penalty)
  {
    switch(penalty)
    {
    case PENALTY_NONE:
      return "none";
    case PENALTY_SPL_BALL_HOLDING:
      return "ball holding";
    case PENALTY_SPL_PLAYER_PUSHING:
      return "pushing";
    case PENALTY_SPL_OBSTRUCTION:
      return "obstruction";
    case PENALTY_SPL_INACTIVE_PLAYER:
      return "inactive robot";
    case PENALTY_SPL_ILLEGAL_DEFENDER:
      return "illegal defender";
    case PENALTY_SPL_LEAVING_THE_FIELD:
      return "leaving field";
    case PENALTY_SPL_PLAYING_WITH_HANDS:
      return "hands";
    case PENALTY_SPL_REQUEST_FOR_PICKUP:
      return "request for pickup";
    default:
      return 0;
    }
  };
};

/**
* @class HalfTimeInfo
*
* A bunch of information concerning the current halftime.
* Have a look at the members for further information.
*/
class HalfTimeInfo : public Streamable
{
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(visitedFieldGrid);
    STREAM(roleTimes);
    STREAM(longestTimeWhenNoBallWasSeen);
    STREAM(timeWithoutGameController);
    STREAM(timeWastedNotBeingInPlayingState);
    STREAM(timeWhenMatchStartedThisHalfTime);
    STREAM(timeWhenMatchFinishedThisHalfTime);
    STREAM(longestSearchForBallTime);
    STREAM(meanTimeToScoreRobot);
    STREAM(meanTimeToScoreTeam);
    STREAM(timeWithoutGroundContact);
    STREAM(timeBeingPenalized);
    STREAM(duelTime);
    STREAM(teamScore);
    STREAM(robotScore);
    STREAM(numOfKicks);
    STREAM(teamColor);
    STREAM(totalDistanceTraveled);
    STREAM(distanceTraveledPlaying);
    STREAM(numOfLosts);
    STREAM(scoreInformation);
    STREAM(fallDownInformation);
    STREAM(traceOnField);
    STREAM(searchForBallInformation);
    STREAM(penaltyInformation);
    STREAM_REGISTER_FINISH;
  }

public:
  /** Constructor */
  HalfTimeInfo() : longestTimeWhenNoBallWasSeen(0), timeWithoutGameController(0),
    timeWastedNotBeingInPlayingState(0), timeWhenMatchStartedThisHalfTime(0),
    timeWhenMatchFinishedThisHalfTime(0), longestSearchForBallTime(0), meanTimeToScoreRobot(0),
    meanTimeToScoreTeam(0), timeWithoutGroundContact(0), timeBeingPenalized(0), duelTime(0),
    teamScore(0), robotScore(0), numOfKicks(0), teamColor(0), totalDistanceTraveled(0.f), distanceTraveledPlaying(0.f),
    numOfLosts(0)
  {
    memset(visitedFieldGrid, 0, sizeof(visitedFieldGrid));
    memset(roleTimes, 0, sizeof(roleTimes));
    scoreInformation.reserve(10);
    fallDownInformation.reserve(16);
    traceOnField.reserve(32);
    searchForBallInformation.reserve(16);
    penaltyInformation.reserve(8);
  };

  static const int xSteps = 12; /**< count of cells in a row */
  static const int ySteps = 8;  /**< count of cells in a column */
  static const float gridCell; /**< size of a quadratic cell */
  static const float fieldSizeX; /**< x size of the field in mm ... maybe there is a way to avoid setting this by hand ... */
  static const float fieldSizeY; /**< y size of the field in mm ... maybe there is a way to avoid setting this by hand ... */
  static const float cellGap; /**< gap between cells ... for visualisation issues */
  unsigned char visitedFieldGrid[xSteps* ySteps];  /**< the data for the grid/heatMap */
  unsigned roleTimes[4]; /**< time spent in a certain role */
  unsigned longestTimeWhenNoBallWasSeen; /**< longest time a ball was not seen in this halftime */
  unsigned timeWithoutGameController; /**< sum of ms when no gameController packages were received */
  unsigned timeWastedNotBeingInPlayingState; /**< time wasted not playing ... maybe interesting for preliminary round */
  unsigned timeWhenMatchStartedThisHalfTime; /**< time stamp when this halftime started */
  unsigned timeWhenMatchFinishedThisHalfTime; /**< time stamp when this halftime ended ... */
  unsigned longestSearchForBallTime; /**< longest time spent on searching a ball */
  unsigned meanTimeToScoreRobot; /**< mean time for this robot to score */
  unsigned meanTimeToScoreTeam; /**< mean time for this robots team to score */
  unsigned timeWithoutGroundContact; /** as the name says... */
  unsigned timeBeingPenalized; /**< how much time this robot spend being penalized */
  unsigned duelTime; /**< time spent duelling */
  unsigned char teamScore; /**< goals of this robots team */
  unsigned char robotScore; /**< goals scored of this robot */
  unsigned char numOfKicks; /**< total number of kicks performed this halftime excluding kicks performed while dueling */
  unsigned char teamColor; /**< robots/teams color in this halftime */
  float totalDistanceTraveled; /**< total distance traveled on field this halftime */
  float distanceTraveledPlaying; /**< distance traveled on field this halftime in plazing state */
  unsigned char numOfLosts; /**< how often was this robot lost int this halftime */

  std::vector<ScoreInfo> scoreInformation; /**< stores every Pose2D of a kick */
  std::vector<FallDownInfo> fallDownInformation; /**< stores every Pose2D where this robot fell down */
  std::vector<Trace> traceOnField; /**< stores traces the robot went this halftime */
  std::vector<SearchForBallInfo> searchForBallInformation; /**< stores interesting information on ball searching */
  std::vector<PenaltyInfo> penaltyInformation; /**< stores information on penalty issues */

  void reset()
  {
    memset(visitedFieldGrid, 0, sizeof(visitedFieldGrid));
    memset(roleTimes, 0, sizeof(roleTimes));
    longestTimeWhenNoBallWasSeen = 0;
    timeWithoutGameController = 0;
    timeWastedNotBeingInPlayingState = 0;
    timeWhenMatchStartedThisHalfTime = 0;
    timeWhenMatchFinishedThisHalfTime = 0;
    longestSearchForBallTime = 0;
    meanTimeToScoreRobot = 0;
    meanTimeToScoreTeam = 0;
    timeWithoutGroundContact = 0;
    timeBeingPenalized = 0;
    duelTime = 0;
    teamScore = 0;
    robotScore = 0;
    teamColor = 0;
    totalDistanceTraveled = 0;
    distanceTraveledPlaying = 0;
    numOfLosts = 0;
    scoreInformation.clear();
    fallDownInformation.clear();
    traceOnField.clear();
    searchForBallInformation.clear();
    penaltyInformation.clear();
  }
};

/** static initialization */
const float HalfTimeInfo::gridCell = 500.f;
const float HalfTimeInfo::fieldSizeX = 3000.f;
const float HalfTimeInfo::fieldSizeY = 2000.f;
const float HalfTimeInfo::cellGap = 50.f;

/**
* @class MatchStatistic
*
* The representation itself, incorporating all information/classes above.
*/
class MatchStatistic : public Streamable
{
  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(robotName);
    STREAM(data);
    STREAM(robotNumber);
    STREAM(ht);
    STREAM_REGISTER_FINISH;
  }

public:
  /** Constructor */
  MatchStatistic()
  {
    robotName.reserve(16);
  };

  HalfTimeInfo data; /**< holds relevant half time information */
  std::string robotName;  /**< the name of this robot */
  int robotNumber;
  int ht; /** the halftime */

  void reset()
  {
    data.reset();
  }

  /** This function visualises any information above-mentioned. */
  void draw()
  {
    DECLARE_DEBUG_DRAWING("representation:MatchStatistic:grid", "drawingOnField");
    DECLARE_DEBUG_DRAWING("representation:MatchStatistic:scorePositions", "drawingOnField");
    DECLARE_DEBUG_DRAWING("representation:MatchStatistic:traceOnField", "drawingOnField");
    DECLARE_DEBUG_DRAWING("representation:MatchStatistic:fallDownPositions", "drawingOnField");
    DECLARE_DEBUG_DRAWING("representation:MatchStatistic:stat", "drawingOnField");
    DECLARE_DEBUG_DRAWING("representation:MatchStatistic:searchForBallInfo", "drawingOnField");


    /** draws the grid/heatMap of the desired halftime */
    COMPLEX_DRAWING("representation:MatchStatistic:grid",
    {
      for(size_t x = 0; x < (size_t) HalfTimeInfo::xSteps; x++)
        for(size_t y = 0; y < (size_t) HalfTimeInfo::ySteps; y++)
        {
          unsigned char value = data.visitedFieldGrid[x * HalfTimeInfo::ySteps + y];
          value = value > 25 ? (unsigned char) 255 : (unsigned char)(value * 10);

          float xX = float(x) * HalfTimeInfo::gridCell - HalfTimeInfo::fieldSizeX;
          float yY = float(y) * HalfTimeInfo::gridCell - HalfTimeInfo::fieldSizeY;
          QUADRANGLE("representation:MatchStatistic:grid",
          xX + HalfTimeInfo::cellGap, yY + HalfTimeInfo::cellGap,
          xX + HalfTimeInfo::gridCell - HalfTimeInfo::cellGap, yY + HalfTimeInfo::cellGap,
          xX + HalfTimeInfo::gridCell - HalfTimeInfo::cellGap, yY + HalfTimeInfo::gridCell - HalfTimeInfo::cellGap,
          xX + HalfTimeInfo::cellGap, yY + HalfTimeInfo::gridCell - HalfTimeInfo::cellGap,
          20, Drawings::ps_solid, ColorRGBA(255, 0, 0, value));
        }
    });

    /** draws the robots positions where a kick was performed */
    COMPLEX_DRAWING("representation:MatchStatistic:scorePositions",
    {
      for(std::vector<ScoreInfo>::const_iterator it = data.scoreInformation.begin();
      it != data.scoreInformation.end(); it++)
      {
        ColorClasses::Color color;
        switch(it->effect)
        {
        case ScoreInfo::hit:
          color = ColorClasses::green;
          break;
        case ScoreInfo::wentOut:
          color = ColorClasses::red;
          break;
        default:
          color = ColorClasses::yellow;
          break;
        }
        DRAW_ROBOT_POSE("representation:MatchStatistic:scorePositions", it->kickPose, color);
        char kick[30] = "";
        switch(it->motion)
        {
        case MotionRequest::bike:
          strcpy(kick, BikeRequest::getName(it->bikeID));
          break;
        case MotionRequest::walk:
          strcpy(kick, WalkRequest::getName(it->walkKick));
          break;
        default:
          break;
        }
        DRAWTEXT("representation:MatchStatistic:scorePositions", it->kickPose.translation.x, it->kickPose.translation.y, 20, ColorClasses::white, kick);
        if(it->mirror)
          DRAWTEXT("representation:MatchStatistic:scorePositions", it->kickPose.translation.x, it->kickPose.translation.y - 130, 20, ColorClasses::white, "mirror");
        if(it->effect == ScoreInfo::hit)
          DRAWTEXT("representation:MatchStatistic:scorePositions", it->kickPose.translation.x, it->kickPose.translation.y + 80, 20, ColorClasses::white,
                   600 - it->timeOfGameController << "s");
      }
    });

    /** draws the positions on the field where this robot fell down */
    COMPLEX_DRAWING("representation:MatchStatistic:fallDownPositions",
    {
      for(std::vector<FallDownInfo>::const_iterator it = data.fallDownInformation.begin();
      it != data.fallDownInformation.end(); it++)
      {
        Vector2<> pos = it->position;
        bool aK = it->afterKick;
        CROSS("representation:MatchStatistic:fallDownPositions", pos.x, pos.y, 150, 50, Drawings::ps_solid, aK ? ColorClasses::red : ColorClasses::green);

        char standUp[32] = "";
        strcpy(standUp, SpecialActionRequest::getName(it->specialAction));
        float gap = 0.f;
        DRAWTEXT("representation:MatchStatistic:fallDownPositions", pos.x, pos.y, 20, ColorClasses::white, standUp;);
        if(aK)
        {
          char kick[30] = "";
          switch(it->motion)
          {
          case MotionRequest::bike:
            strcpy(kick, BikeRequest::getName(it->bikeID));
            break;
          case MotionRequest::walk:
            strcpy(kick, WalkRequest::getName(it->walkKick));
            break;
          default:
            break;
          }
          gap -= 130;
          DRAWTEXT("representation:MatchStatistic:fallDownPositions", pos.x, pos.y + gap, 20, ColorClasses::white, kick);
          if(it->mirror)
          {
            gap -= 130.f;
            DRAWTEXT("representation:MatchStatistic:fallDownPositions", pos.x, pos.y + gap, 20, ColorClasses::white, "mirror");
          }
        }
      }
    });

    /** draws this robots trace on the field */
    COMPLEX_DRAWING("representation:MatchStatistic:traceOnField",
    {
      for(std::vector<Trace>::const_iterator trace = data.traceOnField.begin();
      trace != data.traceOnField.end(); trace++)
      {
        if(trace->trace.size() < 2)
          continue;

        ColorRGBA color(trace->r, trace->g, trace->b, trace->a);
        for(size_t i = 0; i < trace->trace.size(); i++)
        {
          Vector2<> from = trace->trace[i].position;
          if(i == trace->trace.size() - 1)
          {
            CIRCLE("representation:MatchStatistic:traceOnField", from.x, from.y, 50, 0,
            Drawings::ps_solid, ColorClasses::black, Drawings::bs_solid, color);
            DRAWTEXT("representation:MatchStatistic:traceOnField", from.x, from.y + 150, 30, color, trace->length / 1000.f << "m");
            break;
          }
          Vector2<> to = trace->trace[i + 1].position;
          Drawings::PenStyle ps = Drawings::ps_null;
          BehaviorData::Role role = trace->trace[i + 1].role;
          switch(role)
          {
          case BehaviorData::striker:
          case BehaviorData::keeper:
            ps = Drawings::ps_solid;
            break;
          case BehaviorData::supporter:
            ps = Drawings::ps_dot;
            break;
          case BehaviorData::defender:
            ps = Drawings::ps_dash;
            break;
          default:
            break;
          }
          ARROW("representation:MatchStatistic:traceOnField", from.x, from.y, to.x, to.y, 20, ps, color);
        }
      }
    });

    /** draws textual information */
    COMPLEX_DRAWING("representation:MatchStatistic:stat",
    {
      Vector2<> offset(-3650.f, 2600.f);
      int fontSize = 20;
      ColorClasses::Color color = ColorClasses::white;
      float rowGap = 130;
      float columnGap = 1500;

      DRAWTEXT("representation:MatchStatistic:stat", offset.x, offset.y, fontSize, color,
      "Robot: " << robotName;
              );
      offset.y -= rowGap;
      std::string tc = data.teamColor == TEAM_BLUE ? "blue" : (data.teamColor == TEAM_RED ? "red"  : "unknown");
      DRAWTEXT("representation:MatchStatistic:stat", offset.x, offset.y, fontSize, color,
      "Robot ID: " << tc << " " << robotNumber
              );
      offset.y -= rowGap;
      DRAWTEXT("representation:MatchStatistic:stat", offset.x, offset.y, fontSize, color,
      "Halftime: " << ht + 1 ;
              );
      offset.y -= rowGap;
      DRAWTEXT("representation:MatchStatistic:stat", offset.x, offset.y, fontSize, color,
      "Time in set/ready state: " << data.timeWastedNotBeingInPlayingState / 1000 << "s"
              );
      offset.y -= rowGap;
      DRAWTEXT("representation:MatchStatistic:stat", offset.x, offset.y, fontSize, color,
      "Time without GC-Packages: " << data.timeWithoutGameController / 1000 << "s"
              );
      offset.x += columnGap;
      offset.y = 2600;
      DRAWTEXT("representation:MatchStatistic:stat", offset.x, offset.y, fontSize, color,
      "Scored team goals: " << data.teamScore
              );
      offset.y -= rowGap;
      DRAWTEXT("representation:MatchStatistic:stat", offset.x, offset.y, fontSize, color,
      "Scored own goals: " << data.robotScore
              );
      offset.y -= rowGap;
      int meanTimeTeam = data.teamScore < 1 ? 0 : data.meanTimeToScoreTeam / data.teamScore;
      DRAWTEXT("representation:MatchStatistic:stat", offset.x, offset.y, fontSize, color,
      "Teams mean time to score: " <<  meanTimeTeam / 1000 << "s"
              );
      offset.y -= rowGap;
      int meanTimeRobot = data.robotScore < 1 ? 0 : data.meanTimeToScoreRobot / data.robotScore;
      DRAWTEXT("representation:MatchStatistic:stat", offset.x, offset.y, fontSize, color,
      "Robots mean time to score: " <<  meanTimeRobot / 1000 << "s"
              );
      offset.y -= rowGap;
      float hitRatio = data.numOfKicks < 1 ? 0.f : 100.f* float(data.robotScore) / float(data.numOfKicks);
      DRAWTEXT("representation:MatchStatistic:stat", offset.x, offset.y, fontSize, color,
      "Hit ratio:  " <<  hitRatio << "%"
              );
      offset.x += columnGap;
      offset.y = 2600;
      DRAWTEXT("representation:MatchStatistic:stat", offset.x, offset.y, fontSize, color,
      "Kick count:  " << data.numOfKicks << " times"
              );
      offset.y -= rowGap;
      DRAWTEXT("representation:MatchStatistic:stat", offset.x, offset.y, fontSize, color,
      "Fall down count: " << (unsigned) data.fallDownInformation.size() << " times"
              );
      offset.y -= rowGap;
      DRAWTEXT("representation:MatchStatistic:stat", offset.x, offset.y, fontSize, color,
      "Longest searchForBall time: " << data.longestSearchForBallTime / 1000 << "s"
              );
      offset.y -= rowGap;
      DRAWTEXT("representation:MatchStatistic:stat", offset.x, offset.y, fontSize, color,
      "Longest time ball was not seen: " << data.longestTimeWhenNoBallWasSeen / 1000 << "s"
              );
      offset.y -= rowGap;
      DRAWTEXT("representation:MatchStatistic:stat", offset.x, offset.y, fontSize, color,
      "Distance walked in playing state: " << data.distanceTraveledPlaying / 1000.f << "m"
              );
      offset.x += columnGap;
      offset.y = 2600;
      DRAWTEXT("representation:MatchStatistic:stat", offset.x, offset.y, fontSize, color,
      "Total distance walked: " << data.totalDistanceTraveled / 1000.f << "m"
              );
      offset.y -= rowGap;
      DRAWTEXT("representation:MatchStatistic:stat", offset.x, offset.y, fontSize, color,
      "Time being penalised: " << data.timeBeingPenalized / 1000 << " s"
              );
      offset.y -= rowGap;
      DRAWTEXT("representation:MatchStatistic:stat", offset.x, offset.y, fontSize, color,
      "Time without ground contact: " << data.timeWithoutGroundContact / 1000 << " s"
              );
      offset.y -= rowGap;
      DRAWTEXT("representation:MatchStatistic:stat", offset.x, offset.y, fontSize, color,
      "Time duelling: " << data.duelTime / 1000 << " s"
              );
      offset.y -= rowGap;
      DRAWTEXT("representation:MatchStatistic:stat", offset.x, offset.y, fontSize, color,
      "#Losts: " << data.numOfLosts << " times"
              );
      offset = Vector2<>(-3650, -2150);
      if(robotNumber == 1)
      {
        DRAWTEXT("representation:MatchStatistic:stat", offset.x, offset.y, fontSize, color,
        "Keeper time: " << data.roleTimes[0] / 1000 << "s"
                );
        offset.y -= rowGap;
      }
      DRAWTEXT("representation:MatchStatistic:stat", offset.x, offset.y, fontSize, color,
      "Supporter time: " << data.roleTimes[1] / 1000 << "s"
              );
      offset.y -= rowGap;
      DRAWTEXT("representation:MatchStatistic:stat", offset.x, offset.y, fontSize, color,
      "Striker time: " << data.roleTimes[2] / 1000 << "s"
              );
      offset.y -= rowGap;
      DRAWTEXT("representation:MatchStatistic:stat", offset.x, offset.y, fontSize, color,
      "Defender time: " << data.roleTimes[3] / 1000 << "s"
              );
      offset = Vector2<>(-2050.f, -2150.f);
      if(data.penaltyInformation.empty())
        DRAWTEXT("representation:MatchStatistic:stat", offset.x, offset.y, 20, ColorClasses::white, "No penalties! :-)");
      for(std::vector<PenaltyInfo>::const_iterator it = data.penaltyInformation.begin();
      it != data.penaltyInformation.end(); it++)
      {
        DRAWTEXT("representation:MatchStatistic:stat", offset.x, offset.y, 20, ColorClasses::white,
        600 - it->secondsLeft << "s: " << PenaltyInfo::getPenaltyName(it->forWhat)
                );
        offset.y -= rowGap;
      }
    });

    /** draws searchForBall information */
    COMPLEX_DRAWING("representation:MatchStatistic:searchForBallInfo",
    {
      for(std::vector<SearchForBallInfo>::const_iterator it = data.searchForBallInformation.begin();
      it != data.searchForBallInformation.end(); it++)
      {
        DRAW_ROBOT_POSE("representation:MatchStatistic:searchForBallInfo", it->searchPose, ColorClasses::orange);
        Vector2<> sp(it->searchPose.translation);
        Vector2<> pp(it->patrolPose.translation);
        if(it->patrol)
        {
          Vector2<> sp(it->searchPose.translation);
          Vector2<> pp(it->patrolPose.translation);
          DRAW_ROBOT_POSE("representation:MatchStatistic:searchForBallInfo", it->patrolPose, ColorClasses::red);
          ARROW("representation:MatchStatistic:searchForBallInfo", sp.x, sp.y, pp.x, pp.y, 10, Drawings::ps_solid, ColorClasses::blue);
          if(it->ballWasFound)
            ARROW("representation:MatchStatistic:searchForBallInfo", pp.x, pp.y, it->locatedBallPos.x, it->locatedBallPos.y, 10, Drawings::ps_solid, ColorClasses::white);
          continue;
        }

        if(it->ballWasFound)
          ARROW("representation:MatchStatistic:searchForBallInfo", sp.x, sp.y, it->locatedBallPos.x, it->locatedBallPos.y, 10, Drawings::ps_solid, ColorClasses::white);

        int fontSize = 20;
        DRAWTEXT("representation:MatchStatistic:searchForBallInfo", sp.x, sp.y, fontSize, ColorClasses::white, it->searchTime << "ms");
        CIRCLE("representation:MatchStatistic:searchForBallInfo", it->locatedBallPos.x, it->locatedBallPos.y, 45, 0, Drawings::ps_solid, ColorRGBA(0, 0, 0, 220),
               Drawings::bs_solid, ColorRGBA(255, 128, 128, 220));
      }
    });
  }
};
