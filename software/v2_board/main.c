/*
* main.c
*
* Created: 24.8.2014 15:10:04
*  Author: klaxalk
*/


#include <avr/io.h>
#include <util/delay.h>
#include "pmic_driver.h"
#include "TC_driver.h"
#include "port_driver.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "sysclk.h"
#include "ioport.h"
#include "usart_driver_RTOS.h"
#include "twi_master_driver.h"
#include "twi_slave_driver.h"

#define	RED	IOPORT_CREATE_PIN(PORTB, 0)
#define	BLUE IOPORT_CREATE_PIN(PORTB, 4)
#define	ORANGE	IOPORT_CREATE_PIN(PORTB, 2)
#define	GREEN	IOPORT_CREATE_PIN(PORTB, 6)
#define	YELLOW	IOPORT_CREATE_PIN(PORTB, 7)
#define	OUT1	IOPORT_CREATE_PIN(PORTD, 5)

#define pulse1_on() ioport_set_pin_level(OUT1, true);
#define pulse1_off() ioport_set_pin_level(OUT1, false);

#define PC_USART	USARTF0
#define STM_USART	USARTC1
#define XBEE_USART	USARTC0

/*! Defining an example slave address. */
#define SLAVE_ADDRESS    0x55

/*! Defining number of bytes in buffer. */
#define NUM_BYTES        8

/*! CPU speed 2MHz, BAUDRATE 100kHz and Baudrate Register Settings */
#define CPU_SPEED	32000000UL
#define BAUDRATE	100000
#define TWI_BAUDSETTING TWI_BAUD(CPU_SPEED, BAUDRATE)

/* Global variables */
TWI_Master_t twiMaster;    /*!< TWI master module. */
TWI_Slave_t twiSlave;      /*!< TWI slave module. */

/* for input PPM capture */
#define PPM_IN_MIN_LENGTH	2000
#define PPM_IN_MAX_LENGTH	4000
#define PPM_IN_TRESHOLD		5000
uint16_t PPM_in_start = 0;
uint8_t PPM_in_current_channel = 0;
volatile uint16_t RCchannel[9] = {PPM_IN_MIN_LENGTH, PPM_IN_MIN_LENGTH, PPM_IN_MIN_LENGTH, PPM_IN_MIN_LENGTH, PPM_IN_MIN_LENGTH, PPM_IN_MIN_LENGTH, PPM_IN_MIN_LENGTH, PPM_IN_MIN_LENGTH, PPM_IN_MIN_LENGTH};

// values to transmite to Flight-CTRL
#define PULSE_OUT_MIN	4000
#define PULSE_OUT_MIDDLE 6000
#define PULSE_OUT_MAX 8000
volatile uint16_t outputChannels[6] = {PULSE_OUT_MIN, PULSE_OUT_MIN, PULSE_OUT_MIN, PULSE_OUT_MIN, PULSE_OUT_MIN, PULSE_OUT_MIN};
// preserves the current out-transmitting channel number
volatile uint8_t currentChannelOut = 0;
#define NUMBER_OF_CHANNELS_OUT 6
#define PPM_FRAME_LENGTH	80000
#define PPM_PULSE	1600

void blikej(void *p) {
	
	while (1) {

		ioport_toggle_pin_level(YELLOW);
		
		vTaskDelay(100);
	}
}

// this function processes received data on the I2C Slave line
// It is call by the I2C driver
void TWIE_SlaveProcessData(void) {
	
	// takes the received character, increments it's value and sends it back
	twiSlave.sendData[0] = twiSlave.receivedData[0]+1;
}

/*! TWIC Master Interrupt vector. */
ISR(TWIC_TWIM_vect)
{
	TWI_MasterInterruptHandler(&twiMaster);
}

/*! TWIE Slave Interrupt vector. */
ISR(TWIE_TWIS_vect)
{
	TWI_SlaveInterruptHandler(&twiSlave);
}

/*
This task shows the example of TWI (I2C) master and slave. There are two TWI interfaces set up, one as the
master, one as the slave. The task receives characters from the USART, sends it to the TWI slave. The slave takes
the characters, increments their value and returns them to the master. Then they are returned through the USART.
*/
void twi_loopback(void *p) {
	
	taskDISABLE_INTERRUPTS();
	
	// Initialize TWI master on PORTC
	TWI_MasterInit(&twiMaster, &TWIC, TWI_MASTER_INTLVL_LO_gc, TWI_BAUDSETTING);

	// Initialize TWI slave on PORTE
	TWI_SlaveInitializeDriver(&twiSlave, &TWIE, TWIE_SlaveProcessData);
	TWI_SlaveInitializeModule(&twiSlave, SLAVE_ADDRESS, TWI_SLAVE_INTLVL_LO_gc);

	// Enable LO interrupt level (needed without the FreeRTOS)
	// PMIC.CTRL |= PMIC_LOLVLEN_bm;

	// turn on USART on port
	unsigned char inChar;
	UsartBuffer * pc_usart_buffer = usartBufferInitialize(&PC_USART, BAUD19200, 128);
	
	taskENABLE_INTERRUPTS();

	while (1) {
		
		// wait for some characters in USART input buffer
		while (!usartBufferGetByte(pc_usart_buffer, &inChar, 0)) {}
		
		// send the character to I2C Slave
		TWI_MasterWriteRead(&twiMaster, SLAVE_ADDRESS, &inChar, 1, 1);
		
		// Wait until transaction is complete
		while (twiMaster.status != TWIM_STATUS_READY) {}
		
		// send the received character back to the USART
		usartBufferPutByte(pc_usart_buffer, twiMaster.readData[0], 10);
	}
}

void uartTest(void *p) {
	
	unsigned char inChar;
	
	UsartBuffer * pc_usart_buffer = usartBufferInitialize(&PC_USART, BAUD19200, 128);
	usartBufferPutString(pc_usart_buffer, "\n\n\rXMEGA ready", 10);
	
	while (1) {
		
		if (usartBufferGetByte(pc_usart_buffer, &inChar, 0)) {
			
			usartBufferPutByte(pc_usart_buffer, inChar, 10);
		}
	}
}

void timerTest(void *p) {
	
	UsartBuffer * pc_usart_buffer = usartBufferInitialize(&XBEE_USART, BAUD19200, 128);
	
	int i;
	
	while (1) {

		for (i = 0; i < 4; i++) {
			
			usartBufferPutInt(pc_usart_buffer, RCchannel[i], 10, 10);
			
			if (i != 4)
				usartBufferPutByte(pc_usart_buffer, ',', 10);
		}
		
		usartBufferPutByte(pc_usart_buffer, '\r', 10);
		usartBufferPutByte(pc_usart_buffer, '\n', 10);
		
		vTaskDelay(100);
	}
}

void stm(void *p) {
	
	unsigned char inChar;
	
	UsartBuffer * stm_usart_buffer = usartBufferInitialize(&STM_USART, BAUD9600, 128);
	usartBufferPutString(stm_usart_buffer, "\n\n\rXMEGA ready", 10);
	
	UsartBuffer * pc_usart_buffer = usartBufferInitialize(&PC_USART, BAUD19200, 128);
	usartBufferPutString(stm_usart_buffer, "\n\n\rXMEGA ready", 10);
	
	while (1) {
		
		if (usartBufferGetByte(stm_usart_buffer, &inChar, 0)) {
			
			usartBufferPutByte(pc_usart_buffer, inChar, 10);
		}
		if (usartBufferGetByte(pc_usart_buffer, &inChar, 0)) {
			
			usartBufferPutByte(stm_usart_buffer, inChar, 10);
		}
	}
}

void boardInit() {
	
	//-------------- enable TCD1 for PPM input capture ------------------------//
	//select the clock source and pre-scale by 8
	TC1_ConfigClockSource(&TCD1, TC_CLKSEL_DIV8_gc);

	//-------------- enable TCD0 for PPM output -------------------------------//
	//select the clock source and pre-scale by 8
	TC0_ConfigClockSource(&TCD0, TC_CLKSEL_DIV8_gc);
	
	// enable compare A
	TC0_EnableCCChannels(&TCD0, TC0_CCAEN_bm);
	
	// set pediod
	TC_SetPeriod(&TCD0, 40000);
	
	// set the overflow interrupt
	TC0_SetOverflowIntLevel(&TCD0, TC_OVFINTLVL_LO_gc);
	
	// set the compare A low level interrupt
	TC0_SetCCAIntLevel(&TCD0, TC_CCAINTLVL_LO_gc);

	// set the lenght of the standard ppm pulse
	TC_SetCompareA(&TCD0, PPM_PULSE);
	
	//-------------- configure IN1 as a PPM receiver --------------------------//
	PORT_ConfigurePins(&PORTD, 0x10, false, false, PORT_OPC_TOTEM_gc, PORT_ISC_RISING_gc);
	PORT_SetPinsAsInput(&PORTD, 0x10);
	PORT_ConfigureInterrupt0(&PORTD, PORT_INT0LVL_LO_gc, 0x10);
	
	PMIC_EnableLowLevel();
}

void mergeSignalsToOutput() {

	int i;
	for (i = 0; i < NUMBER_OF_CHANNELS_OUT; i++) {

		outputChannels[i] = RCchannel[i]*2;		
	}
}

// PPM receiver
ISR(PORTD_INT0_vect) {
	
	// stores the current time as and END of the last PPM pulse
	uint16_t PPM_in_end = TCD1.CNT;
	uint16_t PPM_in_length = 0;
	
	// if the timer has not overflown
	if (PPM_in_end > PPM_in_start) {
		
		// computes the PPM pulse length
		PPM_in_length = PPM_in_end - PPM_in_start;
	} else { // if the time has overflown
		
		// computes the PPM pulse length
		PPM_in_length = 65535;
		PPM_in_length += PPM_in_end;
		PPM_in_length -= PPM_in_start;
	}
	
	// if the PPM is longer then treshold => synchronizing pulse
	if (PPM_in_length > PPM_IN_TRESHOLD) {
		
		PPM_in_current_channel = 0;
		
	// if it is within the boundaries of desired PPM pusle
	} else if ((PPM_in_length >= PPM_IN_MIN_LENGTH) && (PPM_in_length <= PPM_IN_MAX_LENGTH)) {
		
		RCchannel[PPM_in_current_channel] = PPM_in_length; // stores the value into the RCchannel array
		PPM_in_current_channel++;
	} else {
		
		PPM_in_current_channel++;
	}
	
	PPM_in_start = PPM_in_end;
}

ISR(TCD0_OVF_vect) {

	// starts the output PPM pulse
	pulse1_on();

	if (currentChannelOut < NUMBER_OF_CHANNELS_OUT) {
		
			TC_SetPeriod(&TCD0, outputChannels[currentChannelOut]);
			
			currentChannelOut++;

		} else {

			int i = 0;
			int outputSum = 0;
		
			for (i = 0; i < NUMBER_OF_CHANNELS_OUT; i++) {
				outputSum += outputChannels[i];
			}
			
			currentChannelOut = 0;

			// if the next space is the sync space, calculates it's length
			uint32_t finalOutLen = PPM_FRAME_LENGTH - outputSum;
			TC_SetPeriod(&TCD0, (uint16_t) finalOutLen);
			ioport_toggle_pin_level(BLUE);
	}
}

ISR(TCD0_CCA_vect) {
	
	// shut down the output PPM pulse
	pulse1_off();
}

void mainTask(void *p) {
	
	while (1) {

		mergeSignalsToOutput();
	}
}


int main(void)
{
	
	// prepare the i/o for LEDs
	ioport_init();
	ioport_set_pin_dir(RED, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(BLUE, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(ORANGE, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(GREEN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(YELLOW, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(OUT1, IOPORT_DIR_OUTPUT);
	
	ioport_set_pin_level(RED, true);
	ioport_set_pin_level(BLUE, true);
	ioport_set_pin_level(ORANGE, true);
	ioport_set_pin_level(GREEN, true);
	ioport_set_pin_level(YELLOW, true);
	
	// enable system clock to all peripheral modules
	sysclk_init();
	sysclk_enable_module(SYSCLK_PORT_GEN, 0xff);
	sysclk_enable_module(SYSCLK_PORT_A, 0xff);
	sysclk_enable_module(SYSCLK_PORT_B, 0xff);
	sysclk_enable_module(SYSCLK_PORT_C, 0xff);
	sysclk_enable_module(SYSCLK_PORT_D, 0xff);
	sysclk_enable_module(SYSCLK_PORT_E, 0xff);
	sysclk_enable_module(SYSCLK_PORT_F, 0xff);
	
	boardInit();
	
	xTaskCreate(blikej, (signed char*) "blikej", 1024, NULL, 2, NULL);
	// xTaskCreate(twi_loopback, (signed char*) "twi", 1024, NULL, 2, NULL);
	xTaskCreate(timerTest, (signed char*) "uartTest", 1024, NULL, 2, NULL);
	// xTaskCreate(stm, (signed char*) "stm", 1024, NULL, 2, NULL);
	// xTaskCreate(performanceTest, (signed char*) "perf", 1024, NULL, 2, NULL);
	xTaskCreate(mainTask, (signed char*) "mainTask", 1024, NULL, 2, NULL);
	
	vTaskStartScheduler();
	
	return 0;
}

