/**
* @file ColorConfiguration.h
* Declaration of a class containing the configuration of color prototypes
* @author <a href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</a>
*/

#ifndef __ColorConfiguration_H__
#define __ColorConfiguration_H__

#include "Tools/Streams/Streamable.h"
#include "Tools/ColorClasses.h"
#include "Representations/Infrastructure/Image.h"

class ColorConfiguration : public Streamable
{
private:
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(colors);
    STREAM_REGISTER_FINISH;
  }

public:
  class ColorPrototype : public Streamable
  {
  public:
    unsigned char y, cb, cr;

    ColorPrototype(unsigned char y, unsigned char cb, unsigned char cr) : y(y), cb(cb), cr(cr) {}
    ColorPrototype(const Image::Pixel& pixel) : y(pixel.y), cb(pixel.cb), cr(pixel.cr) {}
    ColorPrototype() {}

    Image::Pixel convertToPixel() const
    {
      Image::Pixel temp;
      temp.y = temp.yCbCrPadding = y;
      temp.cb = cb;
      temp.cr = cr;
      return temp;
    }

  private:
    virtual void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN;
      STREAM(y);
      STREAM(cb);
      STREAM(cr);
      STREAM_REGISTER_FINISH;
    }
  };

  class ColorDefinition : public Streamable
  {
  public:
    ColorPrototype color, threshold;

  private:
    virtual void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN;
      STREAM(color);
      STREAM(threshold);
      STREAM_REGISTER_FINISH;
    }
  };

  ColorDefinition colors[ColorClasses::numOfColors];
};

#endif
