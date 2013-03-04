/** 
* @file FullResImageGridProvider.h
* Declaration of a module that provides information 
* about how detailed different parts of the image are to be scanned
* @author <A href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</A>
*/

#ifndef __FullResImageGridProvider_h_
#define __FullResImageGridProvider_h_

#include "Tools/Module/Module.h"
#include "Representations/Perception/ImageGrid.h"

MODULE(FullResImageGridProvider)
#ifndef RELEASE 
  REQUIRES(Image) // only for debug image
#endif
  PROVIDES(ImageGrid)
END_MODULE

class FullResImageGridProvider: public FullResImageGridProviderBase
{
public:
  void update(ImageGrid& imageGrid);

  /**
  * Default constructor.
  */
  FullResImageGridProvider();
};

#endif // __FullResImageGridProvider_h_
