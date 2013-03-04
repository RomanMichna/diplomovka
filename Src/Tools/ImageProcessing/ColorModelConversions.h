/**
 * @file ColorModelConversions.h
 *
 * Declaration and implementation of class ColorModelConversions
 */

#pragma once

#include "Tools/Math/Common.h"

/**
* A class that defines static conversions between color models for single pixels.
*/
class ColorModelConversions
{
public:
  /** Converts an YCbCr pixel into an RGB pixel.
   *  @param Y The Y channel of the source pixel.
   *  @param Cb The Cb channel of the source pixel.
   *  @param Cr The Cr channel of the source pixel.
   *  @param R The R channel of the target pixel.
   *  @param G The G channel of the target pixel.
   *  @param B The B channel of the target pixel.
   */
  static void fromYCbCrToRGB(unsigned char Y,
                             unsigned char Cb,
                             unsigned char Cr,
                             unsigned char& R,
                             unsigned char& G,
                             unsigned char& B)
  {
    int r = Y + ((1436 * (Cr - 128)) >> 10),
        g = Y - ((354 * (Cb - 128) + 732 * (Cr - 128)) >> 10),
        b = Y + ((1814 * (Cb - 128)) >> 10);
    if(r < 0) r = 0;
    else if(r > 255) r = 255;
    if(g < 0) g = 0;
    else if(g > 255) g = 255;
    if(b < 0) b = 0;
    else if(b > 255) b = 255;
    R = (unsigned char) r;
    G = (unsigned char) g;
    B = (unsigned char) b;
  }

  /** Converts an RGB pixel into an YCbCr pixel.
   *  @param R The R channel of the source pixel.
   *  @param G The G channel of the source pixel.
   *  @param B The B channel of the source pixel.
   *  @param Y The Y channel of the target pixel.
   *  @param Cb The Cb channel of the target pixel.
   *  @param Cr The Cr channel of the target pixel.
   */
  static void fromRGBToYCbCr(unsigned char R,
                             unsigned char G,
                             unsigned char B,
                             unsigned char& Y,
                             unsigned char& Cb,
                             unsigned char& Cr)
  {
    int y = (int)(0.2990 * R + 0.5870 * G + 0.1140 * B),
        cr = 127 + (int)(-0.1687 * R - 0.3313 * G + 0.5000 * B),
        cb = 127 + (int)(0.5000 * R - 0.4187 * G - 0.0813 * B);
    if(y < 0) y = 0;
    else if(y > 255) y = 255;
    if(cb < 0) cb = 0;
    else if(cb > 255) cb = 255;
    if(cr < 0) cr = 0;
    else if(cr > 255) cr = 255;
    Y = (unsigned char) y;
    Cb = (unsigned char) cb;
    Cr = (unsigned char) cr;
  }

  /** Converts an YCbCr pixel into an HSI pixel.
   *  @param Y The Y channel of the source pixel.
   *  @param Cb The Cb channel of the source pixel.
   *  @param Cr The Cr channel of the source pixel.
   *  @param H The H channel of the target pixel.
   *  @param S The S channel of the target pixel.
   *  @param I The I channel of the target pixel.
   */
  static void fromYCbCrToHSI(unsigned char Y,
                             unsigned char Cb,
                             unsigned char Cr,
                             unsigned char& H,
                             unsigned char& S,
                             unsigned char& I)
  {
    const float sqrt3 = 1.732050808f;
    unsigned char R,
                  G,
                  B;
    fromYCbCrToRGB(Y, Cb, Cr, R, G, B);
    I = R;
    if(G > I) I = G;
    if(B > I) I = B;
    if(I)
    {
      S = R;
      if(G < S) S = G;
      if(B < S) S = B;
      S = (unsigned char)(255 - 255 * S / I);
      if(S)
      {
        int h = int(atan2(sqrt3 * (G - B), float(2 * R - G - B)) / pi2 * 256.0f);
        if(h > 255) h -= 256;
        else if(h < 0) h += 256;
        H = (unsigned char) h;
      }
      else
        H = 0;
    }
    else
      S = H = 0;
  }

  /** Converts an HSI pixel into an YCbCr pixel.
   *  @param H The H channel of the source pixel.
   *  @param S The S channel of the source pixel.
   *  @param I The I channel of the source pixel.
   *  @param Y The Y channel of the target pixel.
   *  @param Cb The Cb channel of the target pixel.
   *  @param Cr The Cr channel of the target pixel.
   */
  static void fromHSIToYCbCr(unsigned char H,
                             unsigned char S,
                             unsigned char I,
                             unsigned char& Y,
                             unsigned char& Cb,
                             unsigned char& Cr)
  {
    float h = 1.0f - H / 255.0f,
          s = S / 255.0f,
          r,
          g,
          b;

    if(h < 1.0f / 3.0f)
    {
      g = (1 - s) / 3;
      b = (1 + s * cos(pi2 * h) / cos(pi2 * (1.0f / 6.0f - h))) / 3.0f;
      r = 1 - (g + b);
    }
    else if(h < 2.0f / 3.0f)
    {
      h -= 1.0f / 3.0f;
      b = (1 - s) / 3;
      r = (1 + s * cos(pi2 * h) / cos(pi2 * (1.0f / 6.0f - h))) / 3.0f;
      g = 1 - (b + r);
    }
    else
    {
      h -= 2.0f / 3.0f;
      r = (1 - s) / 3;
      g = (1 + s * cos(pi2 * h) / cos(pi2 * (1.0f / 6.0f - h))) / 3.0f;
      b = 1 - (r + g);
    }

    r *= I * 3;
    g *= I * 3;
    b *= I * 3;
    if(r > 255)
      r = 255;
    if(g > 255)
      g = 255;
    if(b > 255)
      b = 255;

    fromRGBToYCbCr((unsigned char) r,
                   (unsigned char) g,
                   (unsigned char) b,
                   Y, Cb, Cr);
  }

  /** Converts an YCbCr pixel into a TSL pixel.
   *  @param Y The Y channel of the source pixel.
   *  @param Cb The Cb channel of the source pixel.
   *  @param Cr The Cr channel of the source pixel.
   *  @param T The T channel of the target pixel.
   *  @param S The S channel of the target pixel.
   *  @param L The L channel of the target pixel.
   */
  static void fromYCbCrToTSL(unsigned char Y,
                             unsigned char Cb,
                             unsigned char Cr,
                             unsigned char& T,
                             unsigned char& S,
                             unsigned char& L)
  {
    const float pi2_div = 0.15915494309189533576888376337251f;  /* 1.0f / (PI * 2.0f) */

    float cb = Cb - 128.0f,
          cr = Cr - 128.0f,
          tmp = 1.0f / (4.3403f * Y + 2.0f * cr + cb),
          tmp_r = (-0.6697f * cr + 1.6959f * cb) * tmp,
          tmp_g = (-1.168f  * cr - 1.3626f * cb) * tmp,
          tmp_b = (1.8613f * cr - 0.331f  * cb) * tmp;
    float t_out;
    if(tmp_g > 0.0f)
      t_out = (atan2(tmp_r, tmp_g) * pi2_div + 0.25f) * 255.0f;
    else if(tmp_g < 0.0f)
      t_out = (atan2(-tmp_r, -tmp_g) * pi2_div + 0.75f) * 255.0f;
    else
      t_out = 0.0f;
    float s_out = sqrt(1.8f * (tmp_r * tmp_r + tmp_g * tmp_g + tmp_b * tmp_b)) * 255.0f;

    if(t_out < 0.0f)
      t_out = 0.0f;
    else if(t_out > 255.0f)
      t_out = 255.0f;
    if(s_out < 0.0f)
      s_out = 0.0f;
    else if(s_out > 255.0f)
      s_out = 255.0f;

    T = (unsigned char) t_out;
    S = (unsigned char) s_out;
    L = Y;
  }

  /** Converts a TSL pixel into an YCbCr pixel.
   *  @param T The T channel of the source pixel.
   *  @param S The S channel of the source pixel.
   *  @param L The L channel of the source pixel.
   *  @param Y The Y channel of the target pixel.
   *  @param Cb The Cr channel of the target pixel.
   *  @param Cr The Cr channel of the target pixel.
   */
  static void fromTSLToYCbCr(unsigned char T,
                             unsigned char S,
                             unsigned char L,
                             unsigned char& Y,
                             unsigned char& Cb,
                             unsigned char& Cr)
  {
    float rad = S * 0.555f / 255.0f,
          phi = (T * 2 / 255.0f + 0.25f) * pi,
          cb = 128 + rad * cos(phi) * 255.0f,
          cr = 128 - rad * sin(phi) * 255.0f;
    Y = L;

    if(cb < 0)
      cb = 0;
    else if(cb > 255)
      cb = 255;
    if(cr < 0)
      cr = 0;
    else if(cr > 255)
      cr = 255;
    Cb = (unsigned char) cb;
    Cr = (unsigned char) cr;
  }

  /** Converts a TSL pixel into an YCbCr pixel.
   *  @param t The T channel of the source pixel.
   *  @param s The S channel of the source pixel.
   *  @param l The L channel of the source pixel.
   *  @param r The R channel of the target pixel.
   *  @param g The G channel of the target pixel.
   *  @param b The B channel of the target pixel.
   */
  static void fromTSLToRGB(unsigned char t,
                           unsigned char s,
                           unsigned char l,
                           unsigned char& r,
                           unsigned char& g,
                           unsigned char& b)
  {
    float y1, u1, v1, r1, g1, b1, rad, phi;

    /* Convert TSL to YUV */
    rad = (((float) s) * 0.555f) / 255.0f;
    phi = (((float) t) * 2.0f * 3.14159f) / 255.0f;
    phi += 0.25f * 3.14159f;
    y1 = (float) l;
    u1 = (rad * cos(phi)) * 255.0f;
    v1 = -(rad * sin(phi)) * 255.0f;

    /* Crop UV */
    if(u1 < -128.0f)
    {
      u1 = -128.0f;
    }
    else if(u1 > 127.0f)
    {
      u1 = 127.0f;
    }
    if(v1 < -128.0f)
    {
      v1 = -128.0f;
    }
    else if(v1 > 127.0f)
    {
      v1 = 127.0f;
    }

    /* Convert YUV to RGB */
    r1 = y1 + 1.371f * v1;
    g1 = y1 - 0.336f * u1 - 0.698f * v1;
    b1 = y1 + 1.732f * u1;

    /* Crop RGB */
    if(r1 < 0.0f)
    {
      r1 = 0.0f;
    }
    else if(r1 > 255.0f)
    {
      r1 = 255.0f;
    }
    if(g1 < 0.0f)
    {
      g1 = 0.0f;
    }
    else if(g1 > 255.0f)
    {
      g1 = 255.0f;
    }
    if(b1 < 0.0f)
    {
      b1 = 0.0f;
    }
    else if(b1 > 255.0f)
    {
      b1 = 255.0f;
    }

    /* Return values */
    r = (unsigned char) r1;
    g = (unsigned char) g1;
    b = (unsigned char) b1;
  }
};
