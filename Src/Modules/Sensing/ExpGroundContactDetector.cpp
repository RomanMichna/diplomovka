/**
* @file ExpGroundContactDetector.cpp
* Implementation of module ExpGroundContactDetector.
* @author Colin Graf
*/

#include "ExpGroundContactDetector.h"
#include "Tools/Debugging/DebugDrawings.h" // PLOT
#include "Tools/Streams/InStreams.h"
#include "Platform/SoundPlayer.h"

MAKE_MODULE(ExpGroundContactDetector, Sensing)

ExpGroundContactDetector::ExpGroundContactDetector() : contact(false), contactStartTime(0), useAngle(false)
{
  p.noContactMinAccXNoise = 0.08f;
  p.noContactMinAccYNoise = 2.f;
  p.noContactMinAccZNoise = 3.f;
  p.noContactMinGyroNoise = 0.04f;

  p.contactMaxAngleNoise = 0.01f;
  p.contactAngleActivationNoise = 0.007f;
  p.contactMaxAccZ = 10.f;

  InConfigMap stream("expGroundContact.cfg");
  if(stream.exists())
    stream >> p;
}

void ExpGroundContactDetector::update(GroundContactState& groundContactState)
{
  MODIFY("module:ExpGroundContactDetector:parameters", p);

  DECLARE_PLOT("module:ExpGroundContactDetector:angleNoiseX");
  DECLARE_PLOT("module:ExpGroundContactDetector:angleNoiseY");
  DECLARE_PLOT("module:ExpGroundContactDetector:accNoiseX");
  DECLARE_PLOT("module:ExpGroundContactDetector:accNoiseY");
  DECLARE_PLOT("module:ExpGroundContactDetector:accNoiseZ");
  DECLARE_PLOT("module:ExpGroundContactDetector:gyroNoiseX");
  DECLARE_PLOT("module:ExpGroundContactDetector:gyroNoiseY");

  MODIFY("module:ExpGroundContactDetector:contact", contact);
  bool ignoreSensors = (theMotionInfo.motion != MotionRequest::walk && theMotionInfo.motion != MotionRequest::stand) ||
                       (theMotionRequest.motion != MotionRequest::walk && theMotionRequest.motion != MotionRequest::stand) ||
                       (theMotionInfo.motion == MotionRequest::walk && theMotionInfo.walkRequest.kickType != WalkRequest::none);
  if(!ignoreSensors)
  {
    if(contact)
    {
      if(theInertiaSensorData.acc.z != InertiaSensorData::off)
        calibratedAccZValues.add(theInertiaSensorData.acc.z);

      Vector3<> angleDiff = ((const RotationMatrix&)(theTorsoMatrix.rotation * expectedRotationInv)).getAngleAxis();
      angleNoises.add(Vector2<>(sqr(angleDiff.x), sqr(angleDiff.y)));
      Vector2<> angleNoise = angleNoises.getAverage();
      PLOT("module:ExpGroundContactDetector:angleNoiseX", angleNoise.x);
      PLOT("module:ExpGroundContactDetector:angleNoiseY", angleNoise.y);

      if(!useAngle && angleNoises.isFull() && angleNoise.x < p.contactAngleActivationNoise && angleNoise.y < p.contactAngleActivationNoise)
        useAngle = true;

      if((useAngle && (angleNoise.x > p.contactMaxAngleNoise || angleNoise.y > p.contactMaxAngleNoise)) ||
        (calibratedAccZValues.isFull() && calibratedAccZValues.getAverage() > p.contactMaxAccZ))
      {
        /*
        if((useAngle && (angleNoise.x > p.contactMaxAngleNoise || angleNoise.y > p.contactMaxAngleNoise)))
          OUTPUT_ERROR("lost ground contact via angle");
        if((calibratedAccZValues.isFull() && calibratedAccZValues.getAverage() > p.contactMaxAccZ))
          OUTPUT_ERROR("lost ground contact via acc");
        */

        contact = false;
        accNoises.clear();
        gyroNoises.clear();
        accValues.clear();
        gyroValues.clear();
        angleNoises.clear();
#ifndef TARGET_SIM
        if(contactStartTime != 0)
          SoundPlayer::play("high.wav");
#endif
      }
    }
    else
    {
      Vector3<> accAverage = accValues.getAverage();
      Vector2<> gyroAverage = gyroValues.getAverage();
      if(theInspectedInertiaSensorData.acc.x != InertiaSensorData::off)
      {
        accValues.add(theInspectedInertiaSensorData.acc);
        gyroValues.add(theInspectedInertiaSensorData.gyro);
        if(accValues.isFull())
        {
          accNoises.add(Vector3<>(sqr(theInspectedInertiaSensorData.acc.x - accAverage.x), sqr(theInspectedInertiaSensorData.acc.y - accAverage.y), sqr(theInspectedInertiaSensorData.acc.z - accAverage.z)));
          gyroNoises.add(Vector2<>(sqr(theInspectedInertiaSensorData.gyro.x - gyroAverage.x), sqr(theInspectedInertiaSensorData.gyro.y - gyroAverage.y)));
        }
      }
      Vector3<> accNoise = accNoises.getAverage();
      Vector2<> gyroNoise = gyroNoises.getAverage();
      PLOT("module:ExpGroundContactDetector:accNoiseX", accNoise.x);
      PLOT("module:ExpGroundContactDetector:accNoiseY", accNoise.y);
      PLOT("module:ExpGroundContactDetector:accNoiseZ", accNoise.z);
      PLOT("module:ExpGroundContactDetector:gyroNoiseX", gyroNoise.x);
      PLOT("module:ExpGroundContactDetector:gyroNoiseY", gyroNoise.y);

      if(accNoises.isFull() &&
        accAverage.z < -5.f && std::abs(accAverage.x) < 5.f && std::abs(accAverage.y) < 5.f &&
        accNoise.x < p.noContactMinAccXNoise && accNoise.y < p.noContactMinAccYNoise && accNoise.z < p.noContactMinAccZNoise &&
        gyroNoise.x < p.noContactMinGyroNoise && gyroNoise.y < p.noContactMinGyroNoise)
      {
        contact = true;
        useAngle = false;
        contactStartTime = theFrameInfo.time;
        angleNoises.clear();
        calibratedAccZValues.clear();
      }
    }
  }

  groundContactState.contact = contact;
  
  expectedRotationInv = theRobotModel.limbs[MassCalibration::footLeft].translation.z > theRobotModel.limbs[MassCalibration::footRight].translation.z ? theRobotModel.limbs[MassCalibration::footLeft].rotation : theRobotModel.limbs[MassCalibration::footRight].rotation;
}
