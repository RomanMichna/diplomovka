/**
* @file RunLengthImage.h
* Declaration of a class that represents a run length coded image
* @author <a href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</a>
*/

#ifndef __RunLengthImage_h_
#define __RunLengthImage_h_

#include "Representations/Infrastructure/Image.h"
#include "Representations/Perception/ImageGrid.h"
#include "Tools/Debugging/DebugImages.h"

/**
* @class RunLengthImage
* A class that represents a run length encoded image
*/
class RunLengthImage : public Streamable
{
public:
  /**
  * @class Run
  * A class that represents a single run in the run length encoded image
  */
  class Run
  {
  public:
    Image::Pixel color;
    short runLength;
    short y;
    short xStart, xEnd;
    Run* firstOverlapAbove;
    Run* firstOverlapBelow;
    bool operator<(const Run& other) const;
  };

  Run image[cameraResolutionHeight * cameraResolutionWidth];
  Run voidRun;
  Run* beginOfLastLine;
  int numberOfRuns;
  unsigned timestamp;

  void toImage(Image& image, const ImageGrid& imageGrid) const;
  void draw(const ImageGrid& imageGrid) const;

  RunLengthImage() 
  {
    voidRun.color.color = 0;
    voidRun.firstOverlapBelow = voidRun.firstOverlapAbove = &voidRun;
    voidRun.runLength = voidRun.xStart = voidRun.xEnd = voidRun.y = -1;
  }

private:
  DECLARE_DEBUG_IMAGE(runLengthImage);
  // TODO specialized streaming to prevent streaming of redundant data in streaming operators of Run
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(numberOfRuns);
    STREAM(timestamp);
    for(int i = 0; i < numberOfRuns; ++i)
    {
      STREAM(image[i]);
    }
    STREAM_REGISTER_FINISH;
  }
};

/**
 * Streaming operator that writes a Run to a stream.
 * @param stream The stream to write on.
 * @param run The Run object.
 * @return The stream.
 */ 
Out& operator<<(Out& stream, const RunLengthImage::Run& run);

/**
 * Streaming operator that reads a Run from a stream.
 * @param stream The stream to read from.
 * @param run The Run object.
 * @return The stream.
 */ 
In& operator>>(In& stream, RunLengthImage::Run& run);

#endif //__RunLengthImage_h_
