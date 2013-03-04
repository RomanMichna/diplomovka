/**
* @file CameraProvider.h
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

MODULE(CameraProvider)
  REQUIRES(CameraSettings)
  REQUIRES(Image)
  USES(ImageRequest)
  PROVIDES_WITH_OUTPUT(Image)
  PROVIDES_WITH_OUTPUT(ImageOther)
  PROVIDES_WITH_MODIFY_AND_OUTPUT(ImageInfo)
  PROVIDES_WITH_MODIFY_AND_OUTPUT(FrameInfo)
  PROVIDES_WITH_MODIFY(CognitionFrameInfo)
END_MODULE

class CameraProvider : public CameraProviderBase
{
private:
  PROCESS_WIDE_STORAGE_STATIC(CameraProvider) theInstance; /**< Points to the only instance of this class in this process or is 0 if there is none. */

  NaoCamera* camera;
  NaoCamera* otherCamera;
  float cycleTime;
  unsigned lastSwitchTime;
  bool bothCameras;
  ImageInfo theImageInfo;

  void update(Image& image);
  void update(ImageOther& image);
  void update(ImageInfo& imageInfo);
  void update(FrameInfo& frameInfo);
  void update(CognitionFrameInfo& cognitionFrameInfo);

public:
  /**
  * Default constructor.
  */
  CameraProvider();

  /**
  * Destructor.
  */
  ~CameraProvider();

  /**
  * The method returns whether a new image is available.
  * @return Is an new image available?
  */
  static bool isFrameDataComplete();

  /**
  * The method waits for a new image.
  */
  static void waitForFrameData();
};
