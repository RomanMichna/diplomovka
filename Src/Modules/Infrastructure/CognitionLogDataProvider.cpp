/**
* @file CognitionLogDataProvider.cpp
* This file implements a module that provides data replayed from a log file.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas R�fer</a>
*/

#include "CognitionLogDataProvider.h"
#include "Tools/Settings.h"

PROCESS_WIDE_STORAGE(CognitionLogDataProvider) CognitionLogDataProvider::theInstance = 0;

#define ASSIGN(target, source) \
  ALLOC(target) \
  (target&) *representationBuffer[id##target] = (target&) *representationBuffer[id##source];

CognitionLogDataProvider::CognitionLogDataProvider() :
  LogDataProvider(),
  frameDataComplete(false)
{
  theInstance = this;
}

CognitionLogDataProvider::~CognitionLogDataProvider()
{
  theInstance = 0;
}

bool CognitionLogDataProvider::handleMessage(InMessage& message)
{
  return theInstance && theInstance->handleMessage2(message);
}

bool CognitionLogDataProvider::isFrameDataComplete()
{
  if(!theInstance)
    return true;
  else if(theInstance->frameDataComplete)
  {
    OUTPUT(idLogResponse, bin, '\0');
    theInstance->frameDataComplete = false;
    return true;
  }
  else
    return false;
}

bool CognitionLogDataProvider::handleMessage2(InMessage& message)
{
  switch(message.getMessageID())
  {
    HANDLE2(Image,
    {
      ALLOC(FrameInfo)
      FrameInfo& frameInfo = (FrameInfo&) *representationBuffer[idFrameInfo];
      const Image& image = (const Image&) *representationBuffer[idImage];
      frameInfo.cycleTime = (float) (image.timeStamp - frameInfo.time) * 0.001f;
      frameInfo.time = image.timeStamp;
    })
    HANDLE(ImageOther)
    HANDLE2(CameraInfo,
    {
      ALLOC(ImageInfo);
      if(logConverter.isConversionRequired(message, idImageInfo))
      {
        if (representationBuffer[idImageInfo])
          delete representationBuffer[idImageInfo];
        representationBuffer[idImageInfo] = logConverter.newConvertedRepresentation(message, idImageInfo);
      }
    }    
    )
    HANDLE(FrameInfo)
    HANDLE2(GameInfo, ((GameInfo&) *representationBuffer[idGameInfo]).timeLastPackageReceived = theFrameInfo.time;)
    HANDLE(OwnTeamInfo)
    HANDLE(OpponentTeamInfo)
    HANDLE2(RobotInfo, ((RobotInfo&) *representationBuffer[idRobotInfo]).number = Global::getSettings().playerNumber;)
    HANDLE(LinePercept)
    HANDLE(BallPercept)
    HANDLE(GoalPercept)
    HANDLE(BallModel)
    HANDLE2(FilteredSensorData,
    {
      ALLOC(FrameInfo)
      FrameInfo& frameInfo = (FrameInfo&) *representationBuffer[idFrameInfo];
      const FilteredSensorData& filteredSensorData = (const FilteredSensorData&) *representationBuffer[idFilteredSensorData];
      frameInfo.cycleTime = (float) (filteredSensorData.timeStamp - frameInfo.time) * 0.001f;
      frameInfo.time = filteredSensorData.timeStamp;
    })
    HANDLE2(BehaviorControlOutput,
    {
      behaviorControlOutput = (const BehaviorControlOutput&) *representationBuffer[idBehaviorControlOutput];
    })
    HANDLE(ObstacleModel)
    HANDLE(FilteredJointData)
    HANDLE(RobotsModel)
    HANDLE(CombinedWorldModel)
    HANDLE2(GroundTruthRobotPose, ASSIGN(RobotPose, GroundTruthRobotPose))
    HANDLE2(GroundTruthBallModel, ASSIGN(BallModel, GroundTruthBallModel))
    HANDLE2(GroundTruthRobotsModel, ASSIGN(RobotsModel, GroundTruthRobotsModel))
    HANDLE(CameraMatrix)
    HANDLE(CameraMatrixOther)

    HANDLE(ImageCoordinateSystem)
    HANDLE(ImageInfo)
    HANDLE(PotentialRobotPose)
    HANDLE(RobotPose)
    HANDLE(SideConfidence)
    HANDLE(MotionInfo)

  case idProcessFinished:
    frameDataComplete = true;
    return true;

  case idJPEGImage:
    ALLOC(Image)
    {
      JPEGImage jpegImage;
      message.bin >> jpegImage;
      jpegImage.toImage((Image&) *representationBuffer[idImage]);
    }
    ALLOC(FrameInfo)
    ((FrameInfo&) *representationBuffer[idFrameInfo]).time = ((Image&) *representationBuffer[idImage]).timeStamp;
    return true;

  case idJPEGImageOther:
    ALLOC(ImageOther)
    {
      JPEGImage jpegImage;
      message.bin >> jpegImage;
      jpegImage.toImage((Image&) *representationBuffer[idImageOther]);
    }
    return true;
  default:
    return false;
  }
}

MAKE_MODULE(CognitionLogDataProvider, Infrastructure)
