/**
* @file ArmContactModelProvider.h
*
* This module detects whether the robot touches a potential obstacle with an arm.
* This is done with comparing the current arm position and the position where the arm should be.
* The difference of those values is then stored into a buffer for each arm and if that buffer represents
* an error large enough, a positive arm contact for that arm is reported.
*
* If the robot fell down, all error buffers are resetted. Additionally, arm contact may only be reported
* if the robot is standing or walking and not penalized.
*
* Declaration of class ArmContactModelProvider.
* @author <a href="mailto:fynn@informatik.uni-bremen.de">Fynn Feldpausch</a>
* @author <a href="mailto:simont@informatik.uni-bremen.de">Simon Taddiken</a>
* @author <a href="mailto:arneboe@informatik.uni-bremen.de">Arne Böckmann</a>
*/

#pragma once

#include "Tools/RingBuffer.h"
#include "Tools/RingBufferWithSum.h"
#include "Tools/Module/Module.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Modeling/ArmContactModel.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/RoboCupGameControlData.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Tools/Math/Vector.h"


/** maximum numbers of frames to buffer */
#define FRAME_BUFFER_SIZE 5

/** number of angle differences to buffer */
#define ERROR_BUFFER_SIZE 100

/** serves as input for the checkArms method if checking the left arm */
#define LEFT true

/** serves as input for the checkArms method if checking the right arm */
#define RIGHT false

MODULE(ArmContactModelProvider)
  REQUIRES(FilteredJointData)
  USES(MotionInfo)
  USES(JointRequest)
  REQUIRES(GameInfo)
  REQUIRES(RobotInfo)
  REQUIRES(FrameInfo)
  REQUIRES(FallDownState)
  REQUIRES(RobotModel)
  REQUIRES(OdometryData)
  PROVIDES_WITH_MODIFY(ArmContactModel)
END_MODULE


/**
* @class ArmContactModelProvider
*
*/
class ArmContactModelProvider: public ArmContactModelProviderBase
{
public:
  /** Constructor */
  ArmContactModelProvider();

private:
  /**
  * A collection of parameters for this module.
  */
  class Parameters : public Streamable
  {
  public:
    /** Default constructor. */
    Parameters() {}

    float errorXThreshold;          /**< Maximum divergence of arm angleX (in degrees) that is not treated as an obstacle detection */
    float errorYThreshold;          /**< Maximum divergence of arm angleY (in degrees) that is not treated as an obstacle detection */
    unsigned malfunctionThreshold;  /**< Duration of contact in frames after a contact is ignored */
    unsigned int frameDelay;        /**< The size of the delay in frames */
    bool debugMode;                 /**< Enable debug mode */
    float speedBasedErrorReduction; /**< At this translational hand speed, the angular error will be ignored (in mm/s). */

  private:
    /**
    * The method makes the object streamable.
    * @param in The stream from which the object is read.
    * @param out The stream to which the object is written.
    */
    virtual void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN;
      STREAM(errorXThreshold);
      STREAM(errorYThreshold);
      STREAM(debugMode);
      STREAM(malfunctionThreshold);
      STREAM(frameDelay);
      STREAM_REGISTER_FINISH;
    }
  };

  struct ArmAngles
  {
    float leftX,             /**< X angle of left arm */
          leftY,             /**< Y angle of left arm */
          rightX,            /**< X angle of right arm */
          rightY;            /**< Y angle of right arm */
  };

  Parameters p;                                                       /**< The parameters of this module */
  RingBuffer<ArmAngles, FRAME_BUFFER_SIZE> angleBuffer;               /**< Buffered arm angles to eliminate delay */
  /** 
  * Error vector:
  * y-Component: error of shoulder roll
  * x-Component: error of shoulder pitch
  */
  RingBufferWithSum<Vector2f, ERROR_BUFFER_SIZE> leftErrorBuffer;     /**< Buffered error over ERROR_BUFFER_SIZE frames */
  RingBufferWithSum<Vector2f, ERROR_BUFFER_SIZE> rightErrorBuffer;

  Vector2f errorLeft;
  Vector2f errorRight;

  const int soundDelay;         /**< Length of debug sound */
  unsigned int lastSoundTime;   /**< Time of last debug sound */
  Vector2<> lastLeftHandPos;    /**< Last 2-D position of the left hand */
  Vector2<> lastRightHandPos;   /**< Last 2-D position of the right hand */
  Pose2D lastOdometry;          /**< Odometry inthe previous frame. */

  /** Fills the error buffers with differences of current and requested
  * joint angles
  * @param left Check the left or the right arm for collisions?
  * @param factor A factor used to reduce the error entered into the buffer.
  */
  void checkArm(bool left, float factor);


  /** Executes this module.
  * @param ArmContactModel The data structure that is filled by this module.
  */
  void update(ArmContactModel& ArmContactModel);


  /** Dertemines the push direction for an arm. That is, the direction in which the specified arm is being
  * pushed. That means that the obstacle causing the contact is located on the opposite direction of
  * the push direction.
  * The directions are given as compass directions with NOTH being the direction in which the robot
  * currently looks.
  *
  * @param left Check the left arm (true) or the right (false)
  * @param Is error.x above the x-axis threshold?
  * @param Is error.y above the y-axis threshold?
  * @param Current error vector.
  * @return The direction in which the specified arm is being pushed.
  */
  ArmContactModel::PushDirection getDirection(bool left, bool contactX, bool contactY, Vector2f error);
};
