/**
* @file InertiaSensorCalibrator.cpp
* Implementation of module InertiaSensorCalibrator.
* @author Colin Graf
*/

#include "InertiaSensorCalibrator.h"
#include "Tools/Math/Pose3D.h"

MAKE_MODULE(InertiaSensorCalibrator, Sensing)

InertiaSensorCalibrator::InertiaSensorCalibrator()
{
  reset();

  p.timeFrame = 1500;
  p.penalizedTimeFrame = 4000;
  p.gyroBiasProcessNoise = Vector2<>(0.05f, 0.05f);
  p.gyroBiasStandMeasurementNoise = Vector2<>(0.01f, 0.01f);
  p.gyroBiasWalkMeasurementNoise = Vector2<>(0.1f, 0.1f);
  p.accBiasProcessNoise = Vector3<>(0.1f, 0.1f, 0.1f);
  p.accBiasStandMeasurementNoise = Vector3<>(0.1f, 0.1f, 0.1f);
  p.accBiasWalkMeasurementNoise = Vector3<>(1.f, 1.f, 1.f);
}

void InertiaSensorCalibrator::reset()
{
  lastTime = 0;
  lastMotion = MotionRequest::specialAction;
  calibrated = false;
  timeWhenPenalized = 0;
  collectionStartTime = 0;
  cleanCollectionStartTime = 0;
  safeGyro = Vector2<>();
  safeAcc = Vector3<>(0.f, 0.f, -9.80665f);
  inertiaSensorDrops = 1000;
}

void InertiaSensorCalibrator::update(InertiaSensorData& inertiaSensorData)
{
  MODIFY("module:InertiaSensorCalibrator:parameters", p);
  DEBUG_RESPONSE_ONCE("module:InertiaSensorCalibrator:reset", reset(););

  // frame time check
  if(theFrameInfo.time <= lastTime)
  {
    if(theFrameInfo.time == lastTime)
      return; // done!
    reset();
  }

  // update timeLastPenalty
  if(theRobotInfo.penalty != PENALTY_NONE && lastPenalty == PENALTY_NONE)
    timeWhenPenalized = theFrameInfo.time;

  // detect changes in joint calibration
#ifndef RELEASE
  bool jointCalibrationChanged = false;
  for(int i = JointData::LHipYawPitch; i <= JointData::LAnkleRoll; ++i)
    if(theJointCalibration.joints[i].offset != lastJointCalibration.joints[i].offset)
    {
      jointCalibrationChanged = true;
      lastJointCalibration.joints[i].offset = theJointCalibration.joints[i].offset;
    }
  for(int i = JointData::RHipYawPitch; i <= JointData::RAnkleRoll; ++i)
    if(theJointCalibration.joints[i].offset != lastJointCalibration.joints[i].offset)
    {
      jointCalibrationChanged = true;
      lastJointCalibration.joints[i].offset = theJointCalibration.joints[i].offset;
    }
  if(jointCalibrationChanged)
    reset();
#endif

  // check for dropped sensor readings
  const Vector2<>& newGyro = theInspectedInertiaSensorData.gyro;
  const Vector3<>& newAcc = theInspectedInertiaSensorData.acc;
  if(theInspectedInertiaSensorData.acc.x == InertiaSensorData::off)
    ++inertiaSensorDrops;
  else
  {
    inertiaSensorDrops = 0;
    safeGyro = newGyro;
    safeAcc = newAcc;
  }

  // it's prediction time!
  if(lastTime && calibrated)
  {
    const float timeDiff = float(theFrameInfo.time - lastTime) * 0.001f; // in seconds
    accXBias.predict(0.f, sqr(p.accBiasProcessNoise.x * timeDiff));
    accYBias.predict(0.f, sqr(p.accBiasProcessNoise.y * timeDiff));
    accZBias.predict(0.f, sqr(p.accBiasProcessNoise.z * timeDiff));
    gyroXBias.predict(0.f, sqr(p.gyroBiasProcessNoise.x * timeDiff));
    gyroYBias.predict(0.f, sqr(p.gyroBiasProcessNoise.y * timeDiff));
  }

  // detect unstable stuff...
  const MotionRequest::Motion& currentMotion(theMotionSelection.targetMotion);
  bool unstable = false;
  if(currentMotion != lastMotion || // motion change
     currentMotion != theMotionInfo.motion ||  // interpolating
     !theGroundContactState.contact)
    unstable = true;
  else if(currentMotion == MotionRequest::walk &&
          (abs(theWalkingEngineOutput.speed.translation.y) > 10.f ||
           abs(theWalkingEngineOutput.speed.translation.x) > 20.f ||
           abs(theWalkingEngineOutput.speed.rotation) > 0.15f))
    unstable = true;
  else if(inertiaSensorDrops >= 2)
    unstable = true;
  else if(currentMotion != MotionRequest::walk && currentMotion != MotionRequest::stand &&
          !(currentMotion == MotionRequest::specialAction && theMotionSelection.specialActionRequest.specialAction == SpecialActionRequest::sitDownKeeper))
    unstable = true;
  else if(theRobotInfo.penalty != PENALTY_NONE && ((theRobotInfo.secsTillUnpenalised * 1000 < (int) p.penalizedTimeFrame && theFrameInfo.getTimeSince(theGameInfo.timeLastPackageReceived) < 2000) || theFrameInfo.getTimeSince(timeWhenPenalized) < (int) p.penalizedTimeFrame))
    unstable = true;
  else if(accValues.getNumberOfEntries() >= accValues.getMaxEntries())
    unstable = true;

  // update cleanCollectionStartTime
  if(unstable)
    cleanCollectionStartTime = 0;
  else if(!cleanCollectionStartTime)
    cleanCollectionStartTime = theFrameInfo.time;

  // restart sensor value collecting?
  const bool standing = currentMotion == MotionRequest::stand || (currentMotion == MotionRequest::walk && theWalkingEngineOutput.standing);
  const bool walking = currentMotion == MotionRequest::walk && !theWalkingEngineOutput.standing;
  if(unstable || (walking && theWalkingEngineOutput.positionInWalkCycle < lastPositionInWalkCycle) || (standing && theFrameInfo.time - collectionStartTime > 1000))
  {
    // add collection within the time frame to the collection buffer
    ASSERT(accValues.getNumberOfEntries() == gyroValues.getNumberOfEntries());
    if(cleanCollectionStartTime && theFrameInfo.time - cleanCollectionStartTime > p.timeFrame &&
       accValues.getNumberOfEntries())
    {
      ASSERT(collections.getNumberOfEntries() < collections.getMaxEntries());
      collections.add(Collection(accValues.getSum() / float(accValues.getNumberOfEntries()),
                                 gyroValues.getSum() / float(gyroValues.getNumberOfEntries()),
                                 collectionStartTime + (theFrameInfo.time - collectionStartTime) / 2, standing));
    }

    // restart collecting
    accValues.init();
    gyroValues.init();
    collectionStartTime = 0;

    // look if there are any useful buffered collections
    for(int i = collections.getNumberOfEntries() - 1; i >= 0; --i)
    {
      const Collection& collection(collections.getEntry(i));
      if(theFrameInfo.time - collection.timeStamp < p.timeFrame)
        break;
      if(cleanCollectionStartTime && cleanCollectionStartTime < collection.timeStamp)
      {
        // use this collection
        Vector3<>& accBiasMeasurementNoise = collection.standing ? p.accBiasStandMeasurementNoise : p.accBiasWalkMeasurementNoise;
        Vector2<>& gyroBiasMeasurementNoise = collection.standing ? p.gyroBiasStandMeasurementNoise : p.gyroBiasWalkMeasurementNoise;
        if(!calibrated)
        {
          calibrated = true;
          accXBias.init(collection.accAvg.x, sqr(accBiasMeasurementNoise.x));
          accYBias.init(collection.accAvg.y, sqr(accBiasMeasurementNoise.y));
          accZBias.init(collection.accAvg.z, sqr(accBiasMeasurementNoise.z));
          gyroXBias.init(collection.gyroAvg.x, sqr(gyroBiasMeasurementNoise.x));
          gyroYBias.init(collection.gyroAvg.y, sqr(gyroBiasMeasurementNoise.y));
        }
        else
        {
          accXBias.update(collection.accAvg.x, sqr(accBiasMeasurementNoise.x));
          accYBias.update(collection.accAvg.y, sqr(accBiasMeasurementNoise.y));
          accZBias.update(collection.accAvg.z, sqr(accBiasMeasurementNoise.z));
          gyroXBias.update(collection.gyroAvg.x, sqr(gyroBiasMeasurementNoise.x));
          gyroYBias.update(collection.gyroAvg.y, sqr(gyroBiasMeasurementNoise.y));
        }
      }
      collections.removeFirst();
    }
  }

  // collecting....
  if(!unstable)
  {
    // calculate rotation based on foot - torso transformation
    const Pose3D& footLeft(theRobotModel.limbs[MassCalibration::footLeft]);
    const Pose3D& footRight(theRobotModel.limbs[MassCalibration::footRight]);
    const Pose3D footLeftInvert(footLeft.invert());
    const Pose3D footRightInvert(footRight.invert());
    if(abs(footLeftInvert.translation.z - footRightInvert.translation.z) < 3.f/* magic number */)
    {
      // use average of the calculated rotation of each leg
      calculatedRotation = RotationMatrix(Vector3<>(
                                            (atan2(footLeftInvert.rotation.c1.z, footLeftInvert.rotation.c2.z) + atan2(footRightInvert.rotation.c1.z, footRightInvert.rotation.c2.z)) * 0.5f,
                                            (atan2(-footLeftInvert.rotation.c0.z, footLeftInvert.rotation.c2.z) + atan2(-footRightInvert.rotation.c0.z, footRightInvert.rotation.c2.z)) * 0.5f,
                                            0.f));
    }
    else if(footLeftInvert.translation.z > footRightInvert.translation.z)
    {
      // use left foot
      calculatedRotation = footLeftInvert.rotation;
    }
    else
    {
      // use right foot
      calculatedRotation = footRightInvert.rotation;
    }

    // calculate expected acceleration sensor reading
    Vector3<> accGravOnly(calculatedRotation.c0.z, calculatedRotation.c1.z, calculatedRotation.c2.z);
    accGravOnly *= -9.80665f;
    accGravOnly.x /= theSensorCalibration.accXGain;
    accGravOnly.y /= theSensorCalibration.accYGain;
    accGravOnly.z /= theSensorCalibration.accZGain;

    // add sensor reading to the collection
    ASSERT(accValues.getNumberOfEntries() < accValues.getMaxEntries());
    accValues.add(safeAcc - accGravOnly);
    gyroValues.add(safeGyro);
    if(!collectionStartTime)
      collectionStartTime = theFrameInfo.time;
  }

  // provide calibrated inertia readings
  inertiaSensorData.calibrated = calibrated;
  if(!calibrated || newGyro.x == SensorData::off)
  {
    inertiaSensorData.gyro.x = inertiaSensorData.gyro.y = InertiaSensorData::off;
    inertiaSensorData.acc.x = inertiaSensorData.acc.y = inertiaSensorData.acc.z = InertiaSensorData::off;
  }
  else
  {
    inertiaSensorData.gyro.x = newGyro.x - gyroXBias.value;
    inertiaSensorData.gyro.y = newGyro.y - gyroYBias.value;
    inertiaSensorData.acc.x = newAcc.x - accXBias.value;
    inertiaSensorData.acc.y = newAcc.y - accYBias.value;
    inertiaSensorData.acc.z = newAcc.z - accZBias.value;

    inertiaSensorData.gyro.x *= theSensorCalibration.gyroXGain;
    inertiaSensorData.gyro.y *= theSensorCalibration.gyroYGain;
    inertiaSensorData.acc.x *= theSensorCalibration.accXGain;
    inertiaSensorData.acc.y *= theSensorCalibration.accYGain;
    inertiaSensorData.acc.z *= theSensorCalibration.accZGain;
  }


  MODIFY("module:InertiaSensorCalibrator:calibrated", calibrated);
  MODIFY("module:InertiaSensorCalibrator:gyroXBias", gyroXBias.value);
  MODIFY("module:InertiaSensorCalibrator:gyroYBias", gyroYBias.value);
  MODIFY("module:InertiaSensorCalibrator:accXBias", accXBias.value);
  MODIFY("module:InertiaSensorCalibrator:accYBias", accYBias.value);
  MODIFY("module:InertiaSensorCalibrator:accZBias", accZBias.value);

  // store some values for the next iteration
  lastTime = theFrameInfo.time;
  lastMotion = theMotionSelection.targetMotion;
  lastPositionInWalkCycle = theWalkingEngineOutput.positionInWalkCycle;
  lastPenalty = theRobotInfo.penalty;
}
