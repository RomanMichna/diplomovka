/**
* @file ExpCameraProvider.h
* This file declares a module that provides camera images.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Tools/Settings.h"
#include "Representations/Configuration/CameraSettings.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/ImageInfo.h"

class NaoCamera;

MODULE(ExpCameraProvider)
  REQUIRES(CameraSettings)
  REQUIRES(Image)
  USES(ImageRequest)
  PROVIDES_WITH_OUTPUT(Image)
  PROVIDES_WITH_MODIFY_AND_OUTPUT(ImageInfo)
  PROVIDES_WITH_MODIFY_AND_OUTPUT(FrameInfo)
  PROVIDES_WITH_MODIFY(CognitionFrameInfo)
END_MODULE

class ExpCameraProvider : public ExpCameraProviderBase
{
private:
  PROCESS_WIDE_STORAGE_STATIC(ExpCameraProvider) theInstance; /**< Points to the only instance of this class in this process or is 0 if there is none. */

  NaoCamera* camera;
  NaoCamera* lowerCamera;
  Image lowerCameraImage;
  NaoCamera* currentImageCamera;
  float cycleTime;
  ImageInfo imageInfo;
  unsigned int imageTimeStamp;
  unsigned int otherImageTimeStamp;
  unsigned int lastImageTimeStamp;
  unsigned long long lastImageTimeStampLL;

  void update(Image& image);
  void update(ImageInfo& imageInfo);
  void update(FrameInfo& frameInfo);
  void update(CognitionFrameInfo& cognitionFrameInfo);

public:
  /**
  * Default constructor.
  */
  ExpCameraProvider();

  /**
  * Destructor.
  */
  ~ExpCameraProvider();

  /**
  * The method returns whether a new image is available.
  * @return Is an new image available?
  */
  static bool isFrameDataComplete();

  /**
  * The method waits for a new image.
  */
  static void waitForFrameData();
  void waitForFrameData2();
};
