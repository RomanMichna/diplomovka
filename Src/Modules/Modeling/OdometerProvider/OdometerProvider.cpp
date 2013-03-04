/*
 * OdometerProvider.cpp
 *
 *  Created on: 25.05.2012
 *      Author: marcel
 */

#include "OdometerProvider.h"



void OdometerProvider::update(Odometer& odometer)
{
  odometer.distanceWalked += (theOdometryData.translation - lastOdometryData.translation).abs();
  lastOdometryData = theOdometryData;
}

MAKE_MODULE(OdometerProvider, Modeling)
