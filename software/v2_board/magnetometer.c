/*
 * magnetometer.c
 *
 * Created: 15.10.2015 16:56:18
 *  Author: klaxalk
 */ 

#include "magnetometer.h"

#include "system.h"
#include "twi_master_driver.h"

TWI_Master_t twi_mag_master;		
uint8_t mag_write_buffer[8];

magnetometerData_t magnetometerData;

void magnetometerInit() {

	TWI_MasterInit(&twi_mag_master, &MAG_I2C_INTERFACE, TWI_MASTER_INTLVL_LO_gc, TWI_BAUD(F_CPU, MAG_I2C_BAUDRATE));
	
	// select the current mode
	mag_write_buffer[0] = 0x3C;
	mag_write_buffer[1] = 0x00;
	mag_write_buffer[2] = 0x70;
	TWI_MasterWriteRead(&twi_mag_master, MAG_I2C_ADDRESS, &mag_write_buffer, 3, 0);
	
	mag_write_buffer[0] = 0x3C;
	mag_write_buffer[1] = 0x01;
	mag_write_buffer[2] = 0xA0;
	TWI_MasterWrite(&twi_mag_master, MAG_I2C_ADDRESS, &mag_write_buffer, 3);
}

void magnetometerRead() {
	
	mag_write_buffer[0] = 0x3D;
	mag_write_buffer[1] = 0x06;	
	TWI_MasterWriteRead(&twi_mag_master, MAG_I2C_ADDRESS, &mag_write_buffer, 2, 6);

	vTaskDelay(200);
	
	magnetometerData.x = twi_mag_master.readData[0] << 8;
	magnetometerData.x |= twi_mag_master.readData[1];
	
	magnetometerData.y = twi_mag_master.readData[2] << 8;
	magnetometerData.y |= twi_mag_master.readData[2];
	
	magnetometerData.z = twi_mag_master.readData[3] << 8;
	magnetometerData.z |= twi_mag_master.readData[3];
}

/*!  Master Interrupt vector for HMC5883L. */
ISR(MAG_I2C_TWIM) {
	
	TWI_MasterInterruptHandler(&twi_mag_master);
}