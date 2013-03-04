#include "Controller/Representations/ClassifiedColor.h"

CONFIGMAP_STREAM_IN(ConfigMap, ClassifiedColor);
CONFIGMAP_STREAM_OUT(ConfigMap, ClassifiedColor);

ConfigMap& operator << (ConfigMap& cv, const ClassifiedColor& classifiedColor)
{
  cv["cc"] << classifiedColor.getColorClass();
  cv["y"] << classifiedColor.get(0);
  cv["u"] << classifiedColor.get(1);
  cv["v"] << classifiedColor.get(2);

  return cv;
}

const ConfigMap& operator >> (const ConfigMap& cv, ClassifiedColor& classifiedColor)
{
  int cc, y, u, v;
  cv["cc"] >> cc;
  cv["y"] >> y;
  cv["u"] >> u;
  cv["v"] >> v;
  classifiedColor = ClassifiedColor(ColorClasses::Color(cc), y, u, v);

  return cv;
}

