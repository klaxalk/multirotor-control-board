/*
 * battery.h
 *
 * Created: 11. 1. 2016 18:24:55
 *  Author: Franta Pucowski
 */ 

#ifndef BATTERY_H_
#define BATTERY_H_

#include "system.h"
#include "TC_driver.h"

float getBatteryVoltage();

void batteryInit();

#endif /* BATTERY_H_ */