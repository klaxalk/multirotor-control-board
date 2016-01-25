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

volatile unsigned int battery_level;
volatile float battery_voltage;
unsigned int levels[4];
unsigned char pos;

uint8_t ReadCalibrationByte(uint8_t index);

void batteryInit();

#endif /* BATTERY_H_ */