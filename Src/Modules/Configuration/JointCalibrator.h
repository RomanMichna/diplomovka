/**
* @file Modules/Infrastructure/JointCalibrator.h
* This file declares a module with tools for calibrating leg joints.
* @author Colin Graf
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Tools/Math/Vector3.h"
#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Infrastructure/JointData.h"

MODULE(JointCalibrator)
  USES(JointRequest)
  USES(JointData)
  REQUIRES(RobotDimensions)
  REQUIRES(MassCalibration)
  PROVIDES_WITH_MODIFY(JointCalibration)
END_MODULE

class JointCalibrator : public JointCalibratorBase
{
private:
  /**
  * An offset which is added to the target of the inverse kinematics to detemermine the calibration offset for each joint angle.
  */
  class Offset : public Streamable
  {
  public:
    /** Default constructor. */
    Offset() {}

    Vector3<> translation;
    Vector3<> rotation;

    bool operator!=(const Offset& other) const
    {
      return translation != other.translation || rotation != other.rotation;
    }

    Offset& operator-=(const Offset& other)
    {
      translation -= other.translation;
      rotation -= other.rotation;
      return *this;
    }

  private:
    /**
    * The method makes the object streamable.
    * @param in The stream from which the object is read.
    * @param out The stream to which the object is written.
    */
    virtual void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN;
      STREAM(translation);
      STREAM(rotation);
      STREAM_REGISTER_FINISH;
    }
  };

  /**
  * Calibration offsets for each foot.
  */
  class Offsets : public Streamable
  {
  public:
    /** Default constructor. */
    Offsets() {}

    Vector3<> bodyRotation;
    Offset leftFoot;
    Offset rightFoot;

    bool operator!=(const Offsets& other) const
    {
      return bodyRotation != other.bodyRotation || leftFoot != other.leftFoot || rightFoot != other.rightFoot;
    }

    Offsets& operator-=(const Offsets& other)
    {
      bodyRotation -= other.bodyRotation;
      leftFoot -= other.leftFoot;
      rightFoot -= other.rightFoot;
      return *this;
    }

  private:
    /**
    * The method makes the object streamable.
    * @param in The stream from which the object is read.
    * @param out The stream to which the object is written.
    */
    virtual void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN;
      STREAM(bodyRotation);
      STREAM(leftFoot);
      STREAM(rightFoot);
      STREAM_REGISTER_FINISH;
    }
  };

  Offsets offsets;
  Offsets lastOffsets;

  /**
  * The central update method to provide the calibration
  * @param jointCalibration The jointCalibration
  */
  void update(JointCalibration& jointCalibration);
};
