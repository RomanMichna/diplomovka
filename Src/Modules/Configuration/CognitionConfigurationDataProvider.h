/**
* @file CognitionConfigurationDataProvider.h
* This file declares a module that provides data loaded from configuration files.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Tools/MessageQueue/InMessage.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/ColorTable64.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Configuration/CameraSettings.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Configuration/PassParameters.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Configuration/HeadLimits.h"
#include "Representations/Configuration/ColorConfiguration.h"

MODULE(CognitionConfigurationDataProvider)
  USES(CameraCalibration)
  USES(ColorTable64)
  PROVIDES_WITH_DRAW(FieldDimensions)
  PROVIDES_WITH_MODIFY(CameraInfo)
  PROVIDES_WITH_MODIFY(CameraSettings)
  PROVIDES_WITH_MODIFY(CameraCalibration)
  PROVIDES_WITH_OUTPUT(ColorTable64)
  PROVIDES_WITH_MODIFY(RobotDimensions)
  PROVIDES_WITH_MODIFY(PassParameters)
  PROVIDES_WITH_MODIFY(DamageConfiguration)
  PROVIDES_WITH_MODIFY_AND_DRAW(HeadLimits)
  PROVIDES_WITH_MODIFY(ColorConfiguration)
END_MODULE

class CognitionConfigurationDataProvider : public CognitionConfigurationDataProviderBase
{
private:
  PROCESS_WIDE_STORAGE_STATIC(CognitionConfigurationDataProvider) theInstance; /**< Points to the only instance of this class in this process or is 0 if there is none. */

  FieldDimensions* theFieldDimensions;
  CameraInfo* theCameraInfo;
  CameraSettings* theCameraSettings;
  CameraCalibration* theCameraCalibration;
  ColorTable64* theColorTable64;
  RobotDimensions* theRobotDimensions;
  PassParameters* thePassParameters;
  DamageConfiguration* theDamageConfiguration;
  HeadLimits* theHeadLimits;
  ColorConfiguration* theColorConfiguration;

  void update(FieldDimensions& fieldDimensions);
  void update(CameraInfo& cameraInfo);
  void update(CameraSettings& cameraSettings);
  void update(CameraCalibration& cameraCalibration);
  void update(ColorTable64& colorTable64);
  void update(RobotDimensions& robotDimensions);
  void update(PassParameters& passParameters);
  void update(DamageConfiguration& damageConfiguration);
  void update(HeadLimits& headLimits);
  void update(ColorConfiguration& colorConfiguration);

  void readFieldDimensions();
  void readCameraInfo();
  void readCameraSettings();
  void readCameraCalibration();
  void readColorTable64();
  void readRobotDimensions();
  void readPassParameters();
  void readDamageConfiguration();
  void readHeadLimits();
  void readColorConfiguration();

  /**
  * The method is called for every incoming debug message by handleMessage.
  * @param message An interface to read the message from the queue.
  * @return Was the message handled?
  */
  bool handleMessage2(InMessage& message);

public:
  /**
  * Default constructor.
  */
  CognitionConfigurationDataProvider();

  /**
  * Destructor.
  */
  ~CognitionConfigurationDataProvider();

  /**
  * The method is called for every incoming debug message.
  * @param message An interface to read the message from the queue.
  * @return Was the message handled?
  */
  static bool handleMessage(InMessage& message);
};
