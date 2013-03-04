#include "Tools/Streams/Streamable.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Platform/BHAssert.h"
#include <algorithm>
#include "Representations/Infrastructure/Image.h"
#include "Tools/Debugging/DebugImages.h"

/**
* @file ImageGrid.h
* Representation containing information how detailed lines of image are to be scanned
* @author <a href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</a>
*/

#ifndef __ImageGrid_h_
#define __ImageGrid_h_

/**
* @class ImageGrid
*/
class ImageGrid : public Streamable
{
private:
  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(lineInformation);
    STREAM(lineInformationLUT);
    STREAM_REGISTER_FINISH;
  }

  DECLARE_DEBUG_IMAGE(grid);
  DECLARE_DEBUG_IMAGE(grid2);

public:
  struct LineInformation
  {
  public:
    unsigned char rowInterval;
    unsigned char columnInterval;
    LineInformation() : rowInterval(1), columnInterval(1) {}
    LineInformation(unsigned char rowInterval, unsigned char columnInterval) : rowInterval(rowInterval), columnInterval(columnInterval) {}
  };

  LineInformation lineInformation[cameraResolutionHeight];
  LineInformation lineInformationLUT[cameraResolutionHeight];
  int numberOfLines;
  int lastYCoord;

  ImageGrid(unsigned char columnInterval = 1) : numberOfLines(cameraResolutionHeight), lastYCoord(cameraResolutionHeight - 1)
  {
    ASSERT(sizeof(short) == sizeof(LineInformation));
    LineInformation lineInf(1, columnInterval);
    fill(lineInformation, lineInformation + cameraResolutionHeight, lineInf);
    fill(lineInformationLUT, lineInformationLUT + cameraResolutionHeight, lineInf);
  }

  void draw(const Image& image) const;
  int getNumOfSamples() const;
};

/**
 * Streaming operator that writes a LineInformation to a stream.
 * @param stream The stream to write on.
 * @param run The LineInformation object.
 * @return The stream.
 */ 
Out& operator<<(Out& stream, const ImageGrid::LineInformation& run);

/**
 * Streaming operator that reads a LineInformation from a stream.
 * @param stream The stream to read from.
 * @param run The LineInformation object.
 * @return The stream.
 */ 
In& operator>>(In& stream, ImageGrid::LineInformation& run);

#endif //__ImageGrid_h_
