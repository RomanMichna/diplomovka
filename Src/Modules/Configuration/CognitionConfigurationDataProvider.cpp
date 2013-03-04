/**
* @file CognitionConfigurationDataProvider.cpp
* This file implements a module that provides data loaded from configuration files.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#include <cstdio>

#include "CognitionConfigurationDataProvider.h"
#include "Tools/Configuration/ConfigMap.h"
#include "Tools/Settings.h"
#include "Platform/File.h"

PROCESS_WIDE_STORAGE(CognitionConfigurationDataProvider) CognitionConfigurationDataProvider::theInstance = 0;

CognitionConfigurationDataProvider::CognitionConfigurationDataProvider() :
  theFieldDimensions(0),
  theCameraInfo(0),
  theCameraSettings(0),
  theCameraCalibration(0),
  theColorTable64(0),
  theRobotDimensions(0),
  thePassParameters(0),
  theDamageConfiguration(0),
  theHeadLimits(0),
  theColorConfiguration(0)
{
  theInstance = this;

  readFieldDimensions();
  readCameraInfo();
  readCameraSettings();
  readCameraCalibration();
  readColorTable64();
  readRobotDimensions();

  readPassParameters();
  readDamageConfiguration();
  readHeadLimits();
  readColorConfiguration();
}

CognitionConfigurationDataProvider::~CognitionConfigurationDataProvider()
{
  if(theFieldDimensions)
    delete theFieldDimensions;
  if(theCameraInfo)
    delete theCameraInfo;
  if(theCameraSettings)
    delete theCameraSettings;
  if(theCameraCalibration)
    delete theCameraCalibration;
  if(theColorTable64)
    delete theColorTable64;
  if(thePassParameters)
    delete thePassParameters;
  if(theDamageConfiguration)
    delete theDamageConfiguration;
  if(theHeadLimits)
    delete theHeadLimits;
  if(theColorConfiguration)
    delete theColorConfiguration;
  theInstance = 0;
}

void CognitionConfigurationDataProvider::update(FieldDimensions& fieldDimensions)
{
  if(theFieldDimensions)
  {
    fieldDimensions = *theFieldDimensions;
    delete theFieldDimensions;
    theFieldDimensions = 0;
  }
}

void CognitionConfigurationDataProvider::update(CameraInfo& cameraInfo)
{
  if(theCameraInfo)
  {
    cameraInfo = *theCameraInfo;
    delete theCameraInfo;
    theCameraInfo = 0;
  }
}

void CognitionConfigurationDataProvider::update(CameraSettings& cameraSettings)
{
  if(theCameraSettings)
  {
    cameraSettings = *theCameraSettings;
    delete theCameraSettings;
    theCameraSettings = 0;
  }
}

void CognitionConfigurationDataProvider::update(CameraCalibration& cameraCalibration)
{
  if(theCameraCalibration)
  {
    cameraCalibration = *theCameraCalibration;
    delete theCameraCalibration;
    theCameraCalibration = 0;
  }
}

void CognitionConfigurationDataProvider::update(ColorTable64& colorTable64)
{
  if(theColorTable64)
  {
    colorTable64 = *theColorTable64;
    delete theColorTable64;
    theColorTable64 = 0;
  }
  DEBUG_RESPONSE("module:CognitionConfigurationDataProvider:hashct",
  {
    OUTPUT(idText, text, "colortable hash: " << colorTable64.hash());
  });
}

void CognitionConfigurationDataProvider::update(RobotDimensions& robotDimensions)
{
  if(theRobotDimensions)
  {
    robotDimensions = *theRobotDimensions;
    delete theRobotDimensions;
    theRobotDimensions = 0;
  }
}

void CognitionConfigurationDataProvider::update(PassParameters& passParameters)
{
  if(thePassParameters)
  {
    passParameters = *thePassParameters;
    delete thePassParameters;
    thePassParameters = 0;
  }
}

void CognitionConfigurationDataProvider::update(DamageConfiguration& damageConfiguration)
{
  if(theDamageConfiguration)
  {
    damageConfiguration = *theDamageConfiguration;
    delete theDamageConfiguration;
    theDamageConfiguration = 0;
  }
}

void CognitionConfigurationDataProvider::update(HeadLimits& headLimits)
{
  if(theHeadLimits)
  {
    headLimits = *theHeadLimits;
    delete theHeadLimits;
    theHeadLimits = 0;
  }
}

void CognitionConfigurationDataProvider::update(ColorConfiguration& colorConfiguration)
{
  if(theColorConfiguration)
  {
    colorConfiguration = *theColorConfiguration;
    delete theColorConfiguration;
    theColorConfiguration = 0;
  }

#ifndef RELEASE
  ColorClasses::Color colorClass = ColorClasses::none;
  MODIFY_ENUM("module:CognitionConfigurationDataProvider:colorClass", colorClass, ColorClasses);
  DEBUG_RESPONSE_ONCE("module:CognitionConfigurationDataProvider:getColorFromColorTable",
    if(colorClass == ColorClasses::none)
    { // secret signal to grab all color classes
      for(int i = 0; i < ColorClasses::numOfColors; ++i)
      {
        colorConfiguration.colors[i].color = ColorConfiguration::ColorPrototype(CognitionConfigurationDataProviderBase::theColorTable64.getAverageColor((ColorClasses::Color)i));
      }
    }
    else
    {
      colorConfiguration.colors[colorClass].color = ColorConfiguration::ColorPrototype(CognitionConfigurationDataProviderBase::theColorTable64.getAverageColor(colorClass));
    }
  );
#endif
}

void CognitionConfigurationDataProvider::readFieldDimensions()
{
  ASSERT(!theFieldDimensions);

  InConfigFile stream(Global::getSettings().expandLocationFilename("fieldDimensions.cfg"));
  if(!stream.exists())
    return;
  theFieldDimensions = new FieldDimensions;
  theFieldDimensions->load();
}

void CognitionConfigurationDataProvider::readCameraInfo()
{
  ASSERT(!theCameraInfo);

  InConfigMap stream(Global::getSettings().expandRobotFilename("cameraInfo.cfg"));
  if(stream.exists())
  {
    theCameraInfo = new CameraInfo;
    stream >> *theCameraInfo;
  }
}

void CognitionConfigurationDataProvider::readCameraSettings()
{
  ASSERT(!theCameraSettings);

  InConfigMap* stream = new InConfigMap(Global::getSettings().expandRobotLocationFilename("cameraSettings.cfg"));
  if(!stream->exists())
  {
    delete stream;
    stream = new InConfigMap(Global::getSettings().expandLocationFilename("cameraSettings.cfg"));
  }

  if(stream->exists())
  {
    theCameraSettings = new CameraSettings;
    *stream >> *theCameraSettings;
  }

  delete stream;
}

void CognitionConfigurationDataProvider::readCameraCalibration()
{
  ASSERT(!theCameraCalibration);

  InConfigMap stream(Global::getSettings().expandRobotFilename("cameraCalibration.cfg"));
  if(stream.exists())
  {
    theCameraCalibration = new CameraCalibration;
    stream >> *theCameraCalibration;
  }
}

void CognitionConfigurationDataProvider::readColorTable64()
{
  ASSERT(!theColorTable64);

  std::string ctName = "coltable";
  const CameraCalibration& cameraCalibration = theCameraCalibration ? *theCameraCalibration : CognitionConfigurationDataProviderBase::theCameraCalibration;
  if(cameraCalibration.colorTemperature > CameraCalibration::defaultCamera)
  {
    std::string ext = CameraCalibration::getName(cameraCalibration.colorTemperature);
    ext[0] = toupper(ext[0]);
    ctName += ext;
  }
  ctName += ".c64";

  std::string fileName = Global::getSettings().expandRobotLocationFilename(ctName);
  InBinaryFile* stream = new InBinaryFile(fileName);
  if(!stream->exists())
  {
    delete stream;
    std::string newFileName = Global::getSettings().expandLocationFilename(ctName);
    printf("Cannot read \"%s\"\n\t trying \"%s\"\n", fileName.c_str(), newFileName.c_str());
    fileName = newFileName;
    stream = new InBinaryFile(fileName);
  }
  if(!stream->exists())
  {
    delete stream;
    std::string newFileName = Global::getSettings().expandRobotLocationFilename("coltable.c64");
    printf("Cannot read \"%s\"\n\t trying \"%s\"\n", fileName.c_str(), newFileName.c_str());
    fileName = newFileName;
    stream = new InBinaryFile(fileName);
  }
  if(!stream->exists())
  {
    delete stream;
    std::string newFileName = Global::getSettings().expandLocationFilename("coltable.c64");
    printf("Cannot read \"%s\"\n\t trying \"%s\"\n", fileName.c_str(), newFileName.c_str());
    fileName = newFileName;
    stream = new InBinaryFile(fileName);
  }

  if(stream->exists())
  {
    theColorTable64 = new ColorTable64;
    *stream >> *theColorTable64;
    printf("colortable hash: \"%s\"\n", theColorTable64->hash().c_str());
  }

  delete stream;
}


void CognitionConfigurationDataProvider::readRobotDimensions()
{
  ASSERT(!theRobotDimensions);

  InConfigMap stream(Global::getSettings().expandRobotFilename("robotDimensions.cfg"));
  if(stream.exists())
  {
    theRobotDimensions = new RobotDimensions;
    stream >> *theRobotDimensions;
  }
}


void CognitionConfigurationDataProvider::readPassParameters()
{
  ASSERT(!thePassParameters);

  InConfigMap stream(Global::getSettings().expandLocationFilename("passParameters.cfg"));
  if(stream.exists())
  {
    thePassParameters = new PassParameters;
    stream >> *thePassParameters;
  }
}

void CognitionConfigurationDataProvider::readDamageConfiguration()
{
  ASSERT(!theDamageConfiguration);

  InConfigMap stream(Global::getSettings().expandRobotFilename("damageConfiguration.cfg"));
  if(stream.exists())
  {
    theDamageConfiguration = new DamageConfiguration;
    stream >> *theDamageConfiguration;
  }
}

void CognitionConfigurationDataProvider::readHeadLimits()
{
  ASSERT(!theHeadLimits);

  InConfigMap stream("headLimits.cfg");
  if(stream.exists())
  {
    theHeadLimits = new HeadLimits;
    stream >> *theHeadLimits;
  }
}

void CognitionConfigurationDataProvider::readColorConfiguration()
{
  ASSERT(!theColorConfiguration);

  InConfigFile stream(Global::getSettings().expandLocationFilename("color.cfg"));
  if(stream.exists())
  {
    theColorConfiguration = new ColorConfiguration;
    stream >> *theColorConfiguration;
  }
}

bool CognitionConfigurationDataProvider::handleMessage(InMessage& message)
{
  CognitionConfigurationDataProvider* instrance = theInstance;
  return instrance && instrance->handleMessage2(message);
}

bool CognitionConfigurationDataProvider::handleMessage2(InMessage& message)
{
  switch(message.getMessageID())
  {
  case idColorTable64:
    if(!theColorTable64)
      theColorTable64 = new ColorTable64;
    message.bin >> *theColorTable64;
    return true;

  case idWriteColorTable64:
    if(!theColorTable64)
      theColorTable64 = new ColorTable64;
    message.bin >> *theColorTable64;
    {
      std::string ctName = "coltable";
      const CameraCalibration& cameraCalibration = theCameraCalibration ? *theCameraCalibration : CognitionConfigurationDataProviderBase::theCameraCalibration;
      if(cameraCalibration.colorTemperature > CameraCalibration::defaultCamera)
      {
        std::string ext = CameraCalibration::getName(cameraCalibration.colorTemperature);
        ext[0] = toupper(ext[0]);
        ctName += ext;
      }
      ctName += ".c64";
      OutBinaryFile stream(Global::getSettings().expandRobotLocationFilename(ctName));
      if(stream.exists())
      {
        stream << *theColorTable64;
      }
      else
      {
        OutBinaryFile stream(Global::getSettings().expandLocationFilename(ctName));
        stream << *theColorTable64;
      }
    }
    return true;

  default:
    return false;
  }
}

MAKE_MODULE(CognitionConfigurationDataProvider, Infrastructure)
