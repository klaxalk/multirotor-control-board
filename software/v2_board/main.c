/*
 * main.c
 *
 * Created: 24.8.2014 15:10:04
 *  Author: klaxalk
 */ 


#include <avr/io.h>
#include <util/delay.h>

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

#define PC_USART	USARTF0

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


void blikej(void *p) {
	
	while (1) {

		ioport_toggle_pin_level(YELLOW);
		vTaskDelay(100);
	}
}

void blikej2(void *p) {
	
	while (1) {

		ioport_toggle_pin_level(BLUE);
		vTaskDelay(1000);
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

int main(void)
{	
	
	// prepare the i/o for LEDs
	ioport_init();
	ioport_set_pin_dir(RED, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(BLUE, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(ORANGE, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(GREEN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(YELLOW, IOPORT_DIR_OUTPUT);
	
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
	
	xTaskCreate(blikej, (signed char*) "blikej", 1024, NULL, 2, NULL);
	xTaskCreate(blikej2, (signed char*) "blikej2", 1024, NULL, 2, NULL);
	xTaskCreate(twi_loopback, (signed char*) "twi", 1024, NULL, 2, NULL);
	// xTaskCreate(uartTest, (signed char*) "uartTest", 1024, NULL, 2, NULL);
		
	vTaskStartScheduler();
	
	return 0;
}

