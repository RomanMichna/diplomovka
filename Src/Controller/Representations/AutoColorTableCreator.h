/*
 * AutoColorTableCreator.h
 *
 *  Created on: 23.03.2012
 *      Author: Robin
 */

#pragma once

#include "Representations/Infrastructure/Image.h"

class AutoColorTableCreator
{
public:
  Image detectGreen(const Image& img);
  int getColorClass(int x, int y, const Image& img);
};
