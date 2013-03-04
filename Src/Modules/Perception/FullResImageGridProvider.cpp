/** 
* @file FullResImageGridProvider.cpp
* Implementation of a module that provides information 
* about how detailed different parts of the image are to be scanned
* @author <a href="allli@informatik.uni-bremen.de">Alexander Härtl</a> 
*/

#include "FullResImageGridProvider.h"
#include "Tools/Range.h"

#define LARGEST_INTERVAL 5

FullResImageGridProvider::FullResImageGridProvider()
{
}

void FullResImageGridProvider::update(ImageGrid& imageGrid)
{
  unsigned char columnInterval = 1;
  MODIFY("module:FullResImageGridProvider:columnInterval", columnInterval);
  imageGrid = ImageGrid(columnInterval);

#ifndef RELEASE
  imageGrid.draw(theImage);
#endif
}

MAKE_MODULE(FullResImageGridProvider, Perception)
