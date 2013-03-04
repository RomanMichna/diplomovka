/*
 * AutoColorTableCreator.cpp
 *
 *  Created on: 23.03.2012
 *      Author: Robin
 */

#include "AutoColorTableCreator.h"
#include "Tools/Math/Common.h"

int AutoColorTableCreator::getColorClass(int x, int y, const Image& img)
{
  return (int)ceil(img.image[y][x].cr / (127.f/3.f));
}


Image AutoColorTableCreator::detectGreen(const Image& img)
{
  Image imgTemp;
  int witdh = img.resolutionWidth;
  int height = img.resolutionHeight;
  
  /*
  for (int x = 0; x<witdh; x++) {
    for (int y = 100; y<height; y++) {
      //imgTemp->image[y][x].y = img.image[y][x].y - img.image[y][x-1].y;
      imgTemp->image[y][x].y = 150;
      //imgTemp->image[y][x].cb = 127;
      imgTemp->image[y][x].cr = getColorClass(x, y, img) * (255/8) ;
      
      //imgTemp->image[y][x].cr = 127;
      imgTemp->image[y][x].cb = getColorClass(x, y, img) * (255/8) ;
    }
  }
  */
  
  //Color in Area
  int xStart, yStart, w, h;
  int dY=0, dCr=0, dCb=0;
  
  xStart = 10;
  yStart = 200;
  w = 10;
  h = 10;
  
  for (int x = xStart; x<xStart+w; x++) {
    for (int y = yStart; y<yStart+h; y++) {
      dY += img.image[y][x].y;
      dCr += img.image[y][x].cr;
      dCb += img.image[y][x].cb;
    }
  }
  
  dY = dY / (w*h);
  dCr = dCr / (w*h);
  dCb = dCb / (w*h);
  
  
  for (int x = xStart; x<xStart+w; x++) {
    for (int y = yStart; y<yStart+h; y++) {
      imgTemp.image[y][x].y = dY;
      imgTemp.image[y][x].cr = dCr;
      imgTemp.image[y][x].cb = dCb;
    }
  }
  
  
  //Green
  int tol = 50;
  for (int x = 0; x<witdh; x++) {
    for (int y = 0; y<height; y++) {
      if (img.image[y][x].y > dY - tol && img.image[y][x].y < dY + tol ) {
        if (img.image[y][x].cr > dCr - tol && img.image[y][x].cr < dCr + tol ) {
          if (img.image[y][x].cb > dCb - tol && img.image[y][x].cb < dCb + tol ) {
            imgTemp.image[y][x].y = dY;
            imgTemp.image[y][x].cr = dCr;
            imgTemp.image[y][x].cb = dCb;
          }
        }
      }
    }
  }
  
  
  imgTemp.image[0][0].r = 200;
  imgTemp.image[0][0].g = 200;
  imgTemp.image[0][0].b = 200;
  
  return imgTemp;
}