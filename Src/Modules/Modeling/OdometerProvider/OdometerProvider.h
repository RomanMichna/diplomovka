/*
 * OdometerProvider.h
 *
 *  Created on: 25.05.2012
 *      Author: marcel
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Modeling/Odometer.h"


MODULE(OdometerProvider)
  REQUIRES(OdometryData)
  PROVIDES_WITH_MODIFY(Odometer)
END_MODULE


class OdometerProvider : public OdometerProviderBase
{
private:
  Pose2D lastOdometryData;
  
  void update(Odometer& odometer);
};
