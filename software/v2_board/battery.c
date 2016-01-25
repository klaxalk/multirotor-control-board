/*
* battery.c
*
* Created: 11. 1. 2016 18:23:54
*  Author: Franta Pucowski
*/

#include <avr/pgmspace.h>
#include <stddef.h>
#include "battery.h"
#include "system.h"
#include "TC_driver.h"

volatile unsigned int battery_level = 0;
volatile float battery_voltage = 0;
unsigned int levels[4] = { 0 };	//missing elements are 0 by default
unsigned char pos = 0;

uint8_t ReadCalibrationByte(uint8_t index) {
	
	uint8_t result;
	NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
	result = pgm_read_byte(index);
	NVM_CMD = NVM_CMD_NO_OPERATION_gc;
	
	return(result);
}

void batteryInit() {	
	
	/* -------------------------------------------------------------------- */
	/*	Initialize timer for periodic ADC									*/
	/* -------------------------------------------------------------------- */
	
	TC0_ConfigClockSource(&TCF0, TC_CLKSEL_DIV64_gc); // osc/64
	
	TC0_SetOverflowIntLevel(&TCF0, TC_OVFINTLVL_LO_gc); //Set low level interrupt
	
	TC_SetPeriod(&TCF0, 4999); //for 100 ms period
	
	/* -------------------------------------------------------------------- */
	/*	Initialize ADC														*/
	/* -------------------------------------------------------------------- */
	
	PORTA.DIR &= ~(1<<5); //Set pin A5 as input

	ADCA.CTRLA |= 0x01; //ADC enable
	
	ADCA.CTRLB = ADC_RESOLUTION_12BIT_gc; //Set 12 bit resolution
	
	ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN5_gc ; //ADC input on pin A5
	
	ADCA.REFCTRL = ADC_REFSEL0_bm; //Set voltage reference Vcc/1.6
	
	ADCA.PRESCALER = ADC_PRESCALER_DIV256_gc; //Set prescaler to osc/256 - conversion takes 12x8 us 
	
	ADCA.CH0.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc; //Set single-ended mode
	
	ADCA.CALH = ReadCalibrationByte( offsetof(NVM_PROD_SIGNATURES_t, ADCACAL0) );
	ADCA.CALL = ReadCalibrationByte( offsetof(NVM_PROD_SIGNATURES_t, ADCACAL1) );
	
	ADCA.CH0.INTCTRL = ADC_CH_INTLVL0_bm; //Enable conversion complete interrupt
}

ISR(TCF0_OVF_vect) {		
	
	ADCA.CH0.CTRL |= ADC_CH_START_bm; // Start conversion on channel 0		
}


float getBatteryVoltage() {
	
	return ((float) (battery_level - 185)) / 220.5;
}

ISR(ADCA_CH0_vect) {
	
	// char i;
	
	battery_level = ADCA.CH0RES;
	
	// with filter
	// levels [(pos++) & 0x03] = ADCA.CH0RES; // Save conversion result
	
	// led_yellow_toggle();
	// battery_level = 0;
	
	/*
	* with filter
	for(i = 0; i < 4; i++)
		battery_level += levels[i]; //running average
	*/
	
	// battery_voltage = ((float) (battery_level - 185)) / 220.5;
}
