/*
 * magnetometer.h
 *
 * Created: 15.10.2015 16:56:29
 *  Author: klaxalk
 */ 


#ifndef MAGNETOMETER_H_
#define MAGNETOMETER_H_

#include "system.h"

// definitions for magnetometer i2c
#define MAG_I2C_INTERFACE	TWIC
#define MAG_I2C_TWIM		TWIC_TWIM_vect
#define MAG_I2C_BAUDRATE	100000
#define MAG_I2C_ADDRESS		0x1E

typedef struct {

	int x, y, z;
} magnetometerData_t;

magnetometerData_t magnetometerData;

void magnetometerInit(void);
void magnetometerRead(void);

#endif /* MAGNETOMETER_H_ */