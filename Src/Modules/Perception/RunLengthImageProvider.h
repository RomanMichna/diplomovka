/**
* @file RunLengthImageProvider.h
* This file declares a class to compute a run length encoded image from the raw image.
* @author <a href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</a>
*/

#ifndef RunLengthImageProvider_H
#define RunLengthImageProvider_H

#include "Tools/Module/Module.h"
#include "Representations/Perception/RunLengthImage.h"
#include "Representations/Perception/ImageGrid.h"
#include "Tools/MMX.h"

MODULE(RunLengthImageProvider)
  REQUIRES(Image)
  REQUIRES(ImageGrid)
  PROVIDES(RunLengthImage);
END_MODULE

class RunLengthImageProvider: public RunLengthImageProviderBase
{
public:
  RunLengthImageProvider();
  static void createLutUsingC(__m64* lut, const int width);

private:
  class Parameters : public Streamable
  {
  public:
    unsigned char thresholdY, thresholdCb, thresholdCr;
    Parameters() : thresholdY(40), thresholdCb(24), thresholdCr(24) {}

    virtual void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN;
      STREAM(thresholdY);
      STREAM(thresholdCb);
      STREAM(thresholdCr);
      STREAM_REGISTER_FINISH;
    }
  } parameters;

  void update(RunLengthImage& runLengthImage);
  int segmentImageAverageUsingC(const void* pSrc, void* pDst, const int threshold, const short* grid, const int width, const int height);
  void extendRunLengthImage(RunLengthImage& runLengthImage);

  __m64 lut[cameraResolutionWidth];
};

#endif // RunLengthImageProvider_H
