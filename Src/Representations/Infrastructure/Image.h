/**
 * @file Image.h
 *
 * Declaration of class Image
 */

#pragma once

#include "Tools/Streams/Streamable.h"

#ifdef WIN32
#define anonymous_struct struct
#else
#define anonymous_struct __extension__ struct
#endif

const int cameraResolutionWidth = 640;
const int cameraResolutionHeight = 480;

/**
* Platform independend definition of an image class
*/
class Image : public Streamable
{
private:
  void serialize(In* in, Out* out);

public:
  /**
  * Default constructor.
  * @param initialize Whether to initialize the image in gray or not
  */
  Image(bool initialize = true);

  /** destructs an image */
  ~Image();

  /**
  * Copy constructor.
  * @param other The image this is copied from.
  */
  Image(const Image& other);

  /**
  * Assignment operator.
  * @param other The image this is copied from.
  * @return This image.
  */
  Image& operator=(const Image& other);

  /**
  * The method sets an external image.
  * @param buffer The image buffer.
  */
  void setImage(const unsigned char* buffer);

  /** Converts an YCbCr image into an RGB image.
  *  @param ycbcrImage The given YCbCr image
  */
  void convertFromYCbCrToRGB(const Image& ycbcrImage);

  /** Converts an RGB image into an YCbCr image.
  *  @param rgbImage The given RGB image
  */
  void convertFromRGBToYCbCr(const Image& rgbImage);

  /** Converts an YCbCr image into a HSI image.
  *  @param ycrcbImage The given YCbCr image
  */
  void convertFromYCbCrToHSI(const Image& ycrcbImage);

  /** Converts a HSI image into an YCbCr image.
  *  @param hsiImage The given HSI image
  */
  void convertFromHSIToYCbCr(const Image& hsiImage);

  /** Converts an YCbCr image into a TSL image.
  *  @param ycbcrImage The given YCbCr image
  */
  void convertFromYCbCrToTSL(const Image& ycbcrImage);

  /**
  * The union defines a pixel in YCbCr space.
  */
  union Pixel
  {
    unsigned color; /**< Representation as single machine word. */
    unsigned char channels[4];  /**< Representation as an array of channels. */
    anonymous_struct
    {
      unsigned char yCbCrPadding, /**< Ignore. */
                    cb, /**< Cb channel. */
                    y, /**< Y channel. */
                    cr; /**< Cr channel. */
    };
    anonymous_struct
    {
      unsigned char r, /**< R channel. */
                    g, /**< G channel. */
                    b, /**< B channel. */
                    rgbPadding; /**< Ignore. */
    };
    anonymous_struct
    {
      unsigned char h, /**< H channel. */
                    s, /**< S channel. */
                    i, /**< I channel. */
                    hsiPadding; /**< Ignore. */
    };
    anonymous_struct
    {
      unsigned char t, /**< T channel. */
                    _s, /**< S channel. */
                    l, /**< L channel. */
                    tslPadding; /**< Ignore. */
    };
  };

  Pixel (*image)[cameraResolutionWidth * 2]; /**< The image. Please note that the second half of each row must be ignored. */
  unsigned timeStamp; /**< The time stamp of this image. */
  bool isReference; /**< States whether this class holds the image, or only a reference to an image stored elsewhere. */
  int resolutionWidth; /**< The width of the image in pixel */
  int resolutionHeight; /**< The height of the image in pixel */

  /**
   * Calculates the distance between the first three bytes of two colors
   * @param a The first color
   * @param b The second color
   * @return The distance
   */
  static float getColorDistance(const Image::Pixel& a, const Image::Pixel& b);
};

class ImageOther : public Image {};
