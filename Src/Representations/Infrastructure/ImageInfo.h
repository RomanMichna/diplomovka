/**
* @file ImageInfo.h
* A class representing information on images.
* @author Felix Wenk
*/

#pragma once

#include "Tools/Enum.h"

/**
* @class ImageInfo
* A class representing information on images.
*/
class ImageInfo : public Streamable
{
private:
  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(prevCamera);
    STREAM(camera);
    STREAM(whiteBalanced);
    STREAM(toggling);
    STREAM_REGISTER_FINISH;
  }

public:
  /**
   * @enum Camera
   * Enum representing the possible sources of an image.
   */
  ENUM(Camera,
    upperCamera,
    lowerCamera,
    bothCameras,
    toggleCameras
  );

  Camera prevCamera;
  Camera camera;
  bool whiteBalanced;
  bool toggling;

  ImageInfo() : prevCamera(lowerCamera), camera(lowerCamera), whiteBalanced(true), toggling(false) {}
  bool fromUpperCamera() const  { return camera == upperCamera; }
  bool fromLowerCamera() const  { return camera == lowerCamera; }
  bool fromLowerPrevCamera() const { return prevCamera == lowerCamera; }
  bool cameraChanged() const { return camera != prevCamera; }
};

/**
 * @class ImageRequest
 * Class representing a request to switch (or not to switch) the camera.
 *
 * This class is complementary to ImageInfo
 */
class ImageRequest : public Streamable
{
private:
  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(requestedCamera, ImageInfo);
    STREAM_REGISTER_FINISH;
  }

public:
  ImageInfo::Camera requestedCamera;

  ImageRequest() : requestedCamera(ImageInfo::lowerCamera) {}
};
