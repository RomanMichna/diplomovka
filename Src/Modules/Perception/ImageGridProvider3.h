/** 
* @file ImageGridProvider3.h
* Declaration of a module that provides information 
* about how detailed different parts of the image are to be scanned
* @author <A href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</A>
*/

#ifndef __ImageGridProvider3_h_
#define __ImageGridProvider3_h_

#include "Tools/Module/Module.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Perception/ImageGrid.h"
#include <vector>

MODULE(ImageGridProvider3)
  REQUIRES(CameraMatrix)
  REQUIRES(CameraInfo)
  REQUIRES(FieldDimensions)
#ifndef RELEASE 
  REQUIRES(Image) // only for debug image
#endif
  PROVIDES(ImageGrid)
END_MODULE

class ImageGridProvider3: public ImageGridProvider3Base
{
public:
  void update(ImageGrid& imageGrid);

  /**
  * Default constructor.
  */
  ImageGridProvider3();

private:
  struct ResolutionChange
  {
    short y;
    bool horizontal;

    bool operator<(const ResolutionChange& other) const { return y < other.y; }
  };

  void init();
  float approximateTime(float resolution) const;
  float timePerLine(int columnInterval) const;
  void computeResolutionChanges(vector<ResolutionChange>& resolutionChanges, ImageGrid::LineInformation& initialRes, float resolution) const;

  /**
  * A collection of parameters.
  */
  class Parameters : public Streamable
  {
  public:
    float timeFullResLine; /**< in microseconds */
    float timePerPixel; /**< in microseconds */
    float addTimePerLine; /**< in microseconds */
    float totalTime; /**< in microseconds */
    float horVerRatio;
    Vector2<int> resolutionGoal;
    Vector2<int> maxInterval;
    int maxIterations;

    /** Default constructor. */
    Parameters() : timeFullResLine(57.5f),
      timePerPixel(0.2166f),
      addTimePerLine(1.3f),
      totalTime(4500.0f),
      horVerRatio(0.7f),
      resolutionGoal(2, 4),
      maxInterval(5, 5),
      maxIterations(10)
    {}

  private:
    /**
    * The method makes the object streamable.
    * @param in The stream from which the object is read.
    * @param out The stream to which the object is written.
    */
    virtual void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN;
      STREAM(timeFullResLine);
      STREAM(timePerPixel);
      STREAM(totalTime);
      STREAM(horVerRatio);
      STREAM(resolutionGoal);
      STREAM(maxInterval);
      STREAM(maxIterations);
      STREAM_REGISTER_FINISH;
    }
  } parameters; /**< Parameters for the module. */

  float a, b;
  int yStart;
  float yStartCoord;
};

#endif // __ImageGridProvider3_h_
