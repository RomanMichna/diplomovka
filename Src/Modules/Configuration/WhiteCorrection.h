/**
* @file WhiteCorrection.cpp
* This file implements a module that can provide white corrected cameraSettings
* @author Alexander Haertl
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Configuration/CameraSettings.h"

MODULE(WhiteCorrection)
  USES(Image)
  PROVIDES_WITH_MODIFY(CameraSettings)
END_MODULE

class WhiteCorrection : public WhiteCorrectionBase
{
private:
  ENUM(State,
    Measure,
    Wait,
    Idle
  );
  State state;

  int accR, accG, accB; /**< accumulators for RGB-components */
  int measurement; /**< counter for measurements in state Measure */
  int framesWaited; /**< counter for frames waited so far in state Wait */

  void update(CameraSettings& cameraSettings);

  /**
  * computes the average color of all pixels of the camera image
  * @param color The average color of all image pixels
  */
  void getAverageYCbCr(Image::Pixel& color);
  void getAverageYCbCrMMX(Image::Pixel& color);

  int denormalizeGain(int gain);
  int normalizeGain(int gain);

public:
  /**
  * Default constructor.
  */
  WhiteCorrection();
};
