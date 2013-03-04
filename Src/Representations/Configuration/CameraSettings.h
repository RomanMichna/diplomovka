/**
* @file CameraSettings.h
* Declaration of a class representing the settings of the PDA camera.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#pragma once

#include <list>
#include "Tools/Streams/Streamable.h"

/**
* @class Properties
* The class represents the properties of the camera.
*/
class CameraSettings : public Streamable
{
public:
  class V4L2Setting
  {
  public:
    int command;
    int value;
    int tolerance;
    V4L2Setting()
      : command(0), value(0), tolerance(0)
    {}
    V4L2Setting(int command, int value, int tolerance = 0)
      : command(command), value(value), tolerance(tolerance)
    {}
    bool operator==(const V4L2Setting& o) const { return command == o.command && value == o.value; }
    bool operator!=(const V4L2Setting& o) const { return !(*this == o); }
  };
private:
  /**
  * The method streams this class.
  * Implements an abstract method of class Streamable.
  * @param in Pointer to a stream to read from.
  * @param out Pointer to a stream to write to.
  */
  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(contrast.value);
    STREAM(saturation.value);
    STREAM(hue.value);
    STREAM(sharpness.value);
    STREAM(autoWhiteBalance.value);
    STREAM(autoExposure.value);
    STREAM(exposure.value);
    STREAM(gain.value);
    STREAM(whiteBalance.value);
    STREAM(whiteBalance.tolerance);
    STREAM_REGISTER_FINISH;
  }
public:
  static const int numSettings = 9;
  V4L2Setting autoWhiteBalance; /* 1: Use auto white balance, 0: disable auto white balance. */
  V4L2Setting autoExposure; /* 1: Use auto exposure, 0: disable auto exposure. */
  V4L2Setting contrast;   /* The contrast in range of [0 .. 127] */
  V4L2Setting saturation; /* The saturation in range of [0 .. 255] */
  V4L2Setting hue; /* The hue in range [-180 .. 180] */
  V4L2Setting sharpness; /* The sharpness in range of [0 .. 31] */
  V4L2Setting exposure; /**< The exposure time in the range of [0 .. 1023]. */
  V4L2Setting gain; /**< The gain level in the range of [0 .. 127]. */
  V4L2Setting whiteBalance; /**< The white balance... */

  /**
  * Default constructor.
  * Initializes everything with invalid values except the settings for auto features.
  * The settings for auto features are initialized so that they disable the
  * features by default.
  */
  CameraSettings();
  CameraSettings(const CameraSettings& o);
  bool operator==(const CameraSettings& o) const;
  bool operator!=(const CameraSettings& o) const {return !(*this == o);}
  std::list<V4L2Setting> getChangesAndAssign(const CameraSettings& other);
  std::list<V4L2Setting> getSettings() const;
  void setSetting(const V4L2Setting& setting);
};
