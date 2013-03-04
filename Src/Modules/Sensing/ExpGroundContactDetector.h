/**
* @file ExpGroundContactDetector.h
* Declaration of module ExpGroundContactDetector.
* @author Colin Graf
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/InertiaSensorData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Representations/MotionControl/MotionInfo.h"

MODULE(ExpGroundContactDetector)
  REQUIRES(RobotModel)
  REQUIRES(InspectedInertiaSensorData)
  REQUIRES(SensorData)
  REQUIRES(FrameInfo)
  REQUIRES(MotionRequest)
  USES(TorsoMatrix)
  USES(InertiaSensorData)
  USES(MotionInfo)
  PROVIDES_WITH_MODIFY(GroundContactState)
END_MODULE

/**
* @class ExpGroundContactDetector
* A module for sensor data filtering.
*/
class ExpGroundContactDetector : public ExpGroundContactDetectorBase
{
public:
  /** Default constructor. */
  ExpGroundContactDetector();

private:
  /**
  * A collection of parameters for the ground contact detector.
  */
  class Parameters : public Streamable
  {
  public:
    /** Default constructor. */
    Parameters() {}

    float noContactMinAccXNoise;
    float noContactMinAccYNoise;
    float noContactMinAccZNoise;
    float noContactMinGyroNoise;

    float contactMaxAngleNoise;
    float contactAngleActivationNoise;
    float contactMaxAccZ;

  private:
    /**
    * The method makes the object streamable.
    * @param in The stream from which the object is read.
    * @param out The stream to which the object is written. 
    */
    virtual void serialize(In* in, Out* out)
    {  
      STREAM_REGISTER_BEGIN;
      STREAM(noContactMinAccXNoise);
      STREAM(noContactMinAccYNoise);
      STREAM(noContactMinAccZNoise);
      STREAM(noContactMinGyroNoise);
      STREAM(contactMaxAngleNoise);
      STREAM(contactAngleActivationNoise);
      STREAM(contactMaxAccZ);
      STREAM_REGISTER_FINISH;
    }
  };

  Parameters p;

  bool contact; /**< Whether the robot has ground contact or not */
  unsigned int contactStartTime; /**< Time when the robot started having ground contact */
  bool useAngle; /**< Whether the estimated angle will be used to detect ground contact loss */

  RotationMatrix expectedRotationInv;

  /**
  * An averager for computing the average of up to \c n entries. 
  * Accumulating an error will be avoided by gradually recounting the sum of all added entries.
  */
  template <typename C, int n, typename T = float> class Averager
  {
  public:
    Averager() {clear();}

    void clear() {oldSum = newSum = C();pos = count = 0;}

    void add(const C& val)
    {
      unsigned int posRaw = pos + 1;
      pos = posRaw % n;
      if(count == n)
      {
        if(posRaw == n)
        {
          oldSum = newSum;
          newSum = C();
        }
        oldSum -= data[pos];
      }
      else
        ++count;
      data[pos] = val;
      newSum += val;
    }

    bool isFull() const {return count == n;}
    C getAverage() const {return (oldSum + newSum) / T(count);}

  private:
    C data[n];
    C oldSum;
    C newSum;
    unsigned int pos;
    unsigned int count;
  };

  Averager<Vector2<>, 60> angleNoises;
  Averager<Vector3<>, 60> accNoises;
  Averager<Vector2<>, 60> gyroNoises;
  Averager<Vector3<>, 60> accValues;
  Averager<Vector2<>, 60> gyroValues;
  Averager<float, 5> calibratedAccZValues;

  /**
  * Updates the GroundContactState representation .
  * @param groundContactState The ground contact representation which is updated by this module.
  */
  void update(GroundContactState& groundContactState);
};
