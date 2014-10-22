/*
* main.c
*
* Created: 24.8.2014 15:10:04
*  Author: Tomas Baca
*/

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "ioport.h"
#include "twi_master_driver.h"
#include "twi_slave_driver.h"
#include "usart_driver_RTOS.h"

//xbee protocol
#include "packets.h"

// basic system functions
#include "system.h"

// the main task routine
#include "mainTask.h"

// the communication task
#include "commTask.h"

// the controllersTask
#include "controllersTask.h"

/*! Defining an example slave address. */
#define SLAVE_ADDRESS    0x55

/*! Defining number of bytes in buffer. */
#define NUM_BYTES        8

/*! CPU speed 2MHz, BAUDRATE 100kHz and Baudrate Register Settings */
#define CPU_SPEED	32000000UL
#define BAUDRATE	100000
#define TWI_BAUDSETTING TWI_BAUD(CPU_SPEED, BAUDRATE)

/* -------------------------------------------------------------------- */
/*	Contains signals from RC receiver									*/
/* -------------------------------------------------------------------- */
extern volatile uint16_t RCchannel[9];

/* -------------------------------------------------------------------- */
/*	Buffers for USARTs													*/
/* -------------------------------------------------------------------- */
extern UsartBuffer * usart_buffer_stm;
extern UsartBuffer * usart_buffer_xbee;
extern UsartBuffer * usart_buffer_log;
extern UsartBuffer * usart_buffer_1;
extern UsartBuffer * usart_buffer_2;
extern UsartBuffer * usart_buffer_3;
extern UsartBuffer * usart_buffer_4;

/* Global variables */
TWI_Master_t twiMaster;    /*!< TWI master module. */
TWI_Slave_t twiSlave;      /*!< TWI slave module. */

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
	
	taskENABLE_INTERRUPTS();

	while (1) {		
		// send the character to I2C Slave
		TWI_MasterWriteRead(&twiMaster, SLAVE_ADDRESS, &inChar, 1, 1);
		
		// Wait until transaction is complete
		while (twiMaster.status != TWIM_STATUS_READY) {}
		}
}


int main(void)
{
	//Control board initialize
	boardInit();
	
	//XBee protocol constants
	constInit();
	
	//Start the communication task routine
	xTaskCreate(commTask, (signed char*) "commTask", 1024, NULL, 2, NULL);
	
	//Start the main task routine
	xTaskCreate(mainTask, (signed char*) "mainTask", 1024, NULL, 2, NULL);

	//Start the main task routine
	xTaskCreate(controllersTask, (signed char*) "contTasks", 1024, NULL, 2, NULL);
	
	//Start the FreeRTOS scheduler
	vTaskStartScheduler();
	
	return 0;
}

