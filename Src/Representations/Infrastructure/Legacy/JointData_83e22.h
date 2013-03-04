/**
* @file JointData_83e22.h
*
* This file declares a class to represent the joint angles sent to the robot.
*
* @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
*/

#pragma once

#include "Tools/Streams/Streamable.h"
#include "Tools/Math/Common.h"
#include "Tools/Debugging/Debugging.h"
#include "Platform/BHAssert.h"
#include "Tools/Enum.h"
/**
* @class JointData_83e22
* A class to represent the joint angles sent to the robot.
*/
class JointData_83e22 : public Streamable
{
protected:
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(angles);
    STREAM(timeStamp);
    STREAM_REGISTER_FINISH;
  }

public:
  ENUM(Joint,
    HeadYaw,
    HeadPitch,
    LShoulderPitch,
    LShoulderRoll,
    LElbowYaw,
    LElbowRoll,
    RShoulderPitch,
    RShoulderRoll,
    RElbowYaw,
    RElbowRoll,
    LHipYawPitch,
    LHipRoll,
    LHipPitch,
    LKneePitch,
    LAnklePitch,
    LAnkleRoll,
    RHipYawPitch,
    RHipRoll,
    RHipPitch,
    RKneePitch,
    RAnklePitch,
    RAnkleRoll
  );


  // If you change those values be sure to change them in MofCompiler.cpp too. (Line ~280)
  enum
  {
    off    = 1000, /**< Special angle for switching off a joint. */
    ignore = 2000  /**< Special angle for not overwriting the previous setting. */
  };
  float angles[numOfJoints]; /**< The angles of all joints. */
  unsigned timeStamp; /**< The time when these angles were received. */

  /**
  * Default constructor.
  * Switches off all joints.
  */
  JointData_83e22() : timeStamp(0)
  {
    for(int i = 0; i < numOfJoints; ++i)
      angles[i] = off;
  }

  /**
  * The method returns the angle of the mirror (left/right) of the given joint.
  * @param joint The joint the mirror of which is returned.
  * @return The angle of the mirrored joint.
  */
  float mirror(Joint joint) const
  {
    switch(joint)
    {
      // don't mirror an invalid joint value (!)
    case HeadYaw:
      return angles[HeadYaw] == off || angles[HeadYaw] == ignore ? angles[HeadYaw] : -angles[HeadYaw];
    case LShoulderPitch:
      return angles[RShoulderPitch];
    case LShoulderRoll:
      return angles[RShoulderRoll];
    case LElbowYaw:
      return angles[RElbowYaw];
    case LElbowRoll:
      return angles[RElbowRoll];
    case RShoulderPitch:
      return angles[LShoulderPitch];
    case RShoulderRoll:
      return angles[LShoulderRoll];
    case RElbowYaw:
      return angles[LElbowYaw];
    case RElbowRoll:
      return angles[LElbowRoll];
    case LHipYawPitch:
      return angles[RHipYawPitch];
    case LHipRoll:
      return angles[RHipRoll];
    case LHipPitch:
      return angles[RHipPitch];
    case LKneePitch:
      return angles[RKneePitch];
    case LAnklePitch:
      return angles[RAnklePitch];
    case LAnkleRoll:
      return angles[RAnkleRoll];
    case RHipYawPitch:
      return angles[LHipYawPitch];
    case RHipRoll:
      return angles[LHipRoll];
    case RHipPitch:
      return angles[LHipPitch];
    case RKneePitch:
      return angles[LKneePitch];
    case RAnklePitch:
      return angles[LAnklePitch];
    case RAnkleRoll:
      return angles[LAnkleRoll];
    default:
      return angles[joint];
    }
  }

  /**
  * The method initializes the joint angles as a mirror of a set of other joint angles.
  * @param other The set of joint angles that are mirrored.
  */
  virtual void mirror(const JointData_83e22& other)
  {
    for(int i = 0; i < numOfJoints; ++i)
      angles[i] = other.mirror((Joint) i);
    timeStamp = other.timeStamp;
  }
};

/**
* @class JointData_83e22Deg
* A class that wraps joint data to be transmitted in degrees.
*/
class JointData_83e22Deg : public JointData_83e22
{
private:
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    if(jointData)
    {
      ASSERT(out);
      for(int i = 0; i < JointData_83e22::numOfJoints; ++i)
        angles[i] = jointData->angles[i] == JointData_83e22::off ? JointData_83e22::off
                    : floor(toDegrees(jointData->angles[i]) * 10.0f + 0.5f) / 10.0f;
      timeStamp = jointData->timeStamp;

      STREAM(angles);
      STREAM(timeStamp);
    }
    else
    {
      STREAM_BASE(JointData_83e22);
    }
    STREAM_REGISTER_FINISH;
  }

  JointData_83e22* jointData; /**< The joint data that is wrapped. */

public:
  /**
  * Default constructor.
  */
  JointData_83e22Deg() : jointData(0) {}

  /**
  * Constructor.
  * @param jointData The joint data that is wrapped.
  */
  JointData_83e22Deg(JointData_83e22& jointData) : jointData(&jointData) {}

  /**
  * Assignment operator.
  */
  JointData_83e22Deg& operator=(const JointData_83e22Deg& other)
  {
    if(jointData)
      for(int i = 0; i < JointData_83e22::numOfJoints; ++i)
        jointData->angles[i] = other.angles[i] == JointData_83e22::off ? JointData_83e22::off
                               : fromDegrees(other.angles[i]);
    else
      *((JointData_83e22*) this) = other;
    return *this;
  }
};

/**
 * @class HardnessData_83e22
 * This class represents the joint hardness in a jointRequest.
 * It loads the default hardness values from hardnessSettings.cfg.
 */
class HardnessData_83e22 : public Streamable
{
public:
  enum Hardness
  {
    useDefault = -1,
  };

  int hardness[JointData_83e22::numOfJoints]; /**< the custom hardness for each joint */

  /**
   * Default Constructor
   */
  HardnessData_83e22()
  {
    resetToDefault();
  }

  /**
  * The method returns the hardness of the mirror (left/right) of the given joint.
  * @param joint The joint the mirror of which is returned.
  * @return The output hardness of the mirrored joint.
  */
  int mirror(const JointData_83e22::Joint joint) const
  {
    switch(joint)
    {
    case JointData_83e22::HeadYaw:
      return hardness[JointData_83e22::HeadYaw];
    case JointData_83e22::LShoulderPitch:
      return hardness[JointData_83e22::RShoulderPitch];
    case JointData_83e22::LShoulderRoll:
      return hardness[JointData_83e22::RShoulderRoll];
    case JointData_83e22::LElbowYaw:
      return hardness[JointData_83e22::RElbowYaw];
    case JointData_83e22::LElbowRoll:
      return hardness[JointData_83e22::RElbowRoll];
    case JointData_83e22::RShoulderPitch:
      return hardness[JointData_83e22::LShoulderPitch];
    case JointData_83e22::RShoulderRoll:
      return hardness[JointData_83e22::LShoulderRoll];
    case JointData_83e22::RElbowYaw:
      return hardness[JointData_83e22::LElbowYaw];
    case JointData_83e22::RElbowRoll:
      return hardness[JointData_83e22::LElbowRoll];
    case JointData_83e22::LHipYawPitch:
      return hardness[JointData_83e22::RHipYawPitch];
    case JointData_83e22::LHipRoll:
      return hardness[JointData_83e22::RHipRoll];
    case JointData_83e22::LHipPitch:
      return hardness[JointData_83e22::RHipPitch];
    case JointData_83e22::LKneePitch:
      return hardness[JointData_83e22::RKneePitch];
    case JointData_83e22::LAnklePitch:
      return hardness[JointData_83e22::RAnklePitch];
    case JointData_83e22::LAnkleRoll:
      return hardness[JointData_83e22::RAnkleRoll];
    case JointData_83e22::RHipYawPitch:
      return hardness[JointData_83e22::LHipYawPitch];
    case JointData_83e22::RHipRoll:
      return hardness[JointData_83e22::LHipRoll];
    case JointData_83e22::RHipPitch:
      return hardness[JointData_83e22::LHipPitch];
    case JointData_83e22::RKneePitch:
      return hardness[JointData_83e22::LKneePitch];
    case JointData_83e22::RAnklePitch:
      return hardness[JointData_83e22::LAnklePitch];
    case JointData_83e22::RAnkleRoll:
      return hardness[JointData_83e22::LAnkleRoll];
    default:
      return hardness[joint];
    }
  }

  /**
   * initializes this instance with the mirrored values of other
   * @param other the HardnessData_83e22 to be mirrored
   */
  void mirror(const HardnessData_83e22& other)
  {
    for(int i = 0; i < JointData_83e22::numOfJoints; ++i)
      hardness[i] = other.mirror((JointData_83e22::Joint)i);
  }

  /**
   * This function resets the hardness for all joints to the default value.
   */
  inline void resetToDefault()
  {
    for(int i = 0; i < JointData_83e22::numOfJoints; ++i)
      hardness[i] = useDefault;
  }

private:
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(hardness);
    STREAM_REGISTER_FINISH;
  }
};

class HardnessSettings_83e22 : public HardnessData_83e22 {};

/**
 * @class JointRequest_83e22
 */
class JointRequest_83e22 : public JointData_83e22
{
public:
  HardnessData_83e22 jointHardness; /**< the hardness for all joints*/

  /**
   * Initializes this instance with the mirrored data from a other JointRequest_83e22
   * @param other the JointRequest_83e22 to be mirrored
   */
  void mirror(const JointRequest_83e22& other)
  {
    JointData_83e22::mirror(other);
    jointHardness.mirror(other.jointHardness);
  }

  /**
   * Returns the mirrored angle of joint
   * @param joint the joint to be mirrored
   */
  float mirror(const JointData_83e22::Joint joint)
  {
    return JointData_83e22::mirror(joint);
  }

protected:
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM_BASE(JointData_83e22);
    STREAM(jointHardness);
    STREAM_REGISTER_FINISH;
  }
};

class FilteredJointData_83e22 : public JointData_83e22 {};
class FilteredJointData_83e22Prev : public FilteredJointData_83e22 {};
class UnstableJointRequest_83e22 : public JointRequest_83e22 {};
