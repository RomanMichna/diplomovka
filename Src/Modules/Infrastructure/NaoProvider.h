/**
* @file Modules/Infrastructure/NaoProvider.h
* The file declares a module that provides information from the Nao via DCM.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Representations/Configuration/SensorCalibration.h"
#include "Representations/Infrastructure/KeyStates.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/LEDRequest.h"
#include "Representations/Infrastructure/USRequest.h"
#include "Platform/Linux/NaoBody.h"

MODULE(NaoProvider)
  REQUIRES(JointCalibration)
  REQUIRES(JointData)
  REQUIRES(LEDRequest)
  REQUIRES(SensorCalibration)
  REQUIRES(CognitionFrameInfo)
  REQUIRES(USRequest)
  PROVIDES_WITH_MODIFY_AND_OUTPUT(JointData)
  PROVIDES_WITH_MODIFY_AND_OUTPUT(SensorData)
  PROVIDES_WITH_MODIFY_AND_OUTPUT(KeyStates)
  PROVIDES_WITH_MODIFY(FrameInfo)
  USES(JointRequest) // Will be accessed in send()
END_MODULE


#ifdef TARGET_ROBOT

/**
* @class NaoProvider
* A module that provides information from the Nao.
*/
class NaoProvider : public NaoProviderBase
{
private:
  PROCESS_WIDE_STORAGE_STATIC(NaoProvider) theInstance; /**< The only instance of this module. */

  NaoBody naoBody;
  SensorData sensorData; /**< The last sensor data received. */
  KeyStates keyStates; /**< The last key states received. */

#ifndef RELEASE
  float clippedLastFrame[JointData::numOfJoints]; /**< Array that indicates whether a certain joint value was clipped in the last frame (and what was the value)*/
#endif

  void update(JointData& jointData);
  void update(SensorData& sensorData) {sensorData = this->sensorData;}
  void update(KeyStates& keyStates) {keyStates = this->keyStates;}
  void update(FrameInfo& frameInfo) {frameInfo.time = theJointData.timeStamp; frameInfo.cycleTime = 0.01f;}

  /**
  * The function sends a command to the Nao.
  */
  void send();

public:
  /**
  * Constructor.
  */
  NaoProvider();

  /**
  * Destructor.
  */
  ~NaoProvider();

  /**
  * The method is called by process Motion to send the requests to the Nao.
  */
  static void finishFrame();

  static bool isFrameDataComplete();

  static void waitForFrameData();
};

#else
//TARGET_ROBOT not defined here (Simulator).

class NaoProvider : public NaoProviderBase
{
private:
  void update(JointData& jointData) {}
  void update(SensorData& sensorData) {}
  void update(KeyStates& keyStates) {}
  void update(FrameInfo& frameInfo) {}
  void send();
  
public:
  NaoProvider() {}
  ~NaoProvider() {}
  static void finishFrame() {}
  static bool isFrameDataComplete() {return true;}
  static void waitForFrameData() {}
};

#endif
