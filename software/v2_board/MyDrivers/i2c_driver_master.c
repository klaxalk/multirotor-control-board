/*
 * i2c_driver.c
 *
 * Created: 31.8.2014 22:29:50
 *  Author: klaxalk
 */ 

#include <avr/io.h>
#include "user_board.h"
#include "sysclk.h"

#define TWI_MASTER       TWIC
#define TWI_MASTER_PORT  PORTC
#define TWI_SLAVE        TWIE
#define TWI_SPEED        50000
#define TWI_MASTER_ADDR  0x50
#define TWI_SLAVE_ADDR   0x60
#define DATA_LENGTH     8
TWI_Slave_t slave;

#define	YELLOW	IOPORT_CREATE_PIN(PORTB, 7)

uint8_t data[DATA_LENGTH] = {
	0x0f, 0x1f, 0x2f, 0x3f, 0x4f, 0x5f, 0x6f, 0x7f
};

uint8_t recv_data[DATA_LENGTH] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

twi_options_t m_options = {
	.speed     = TWI_SPEED,
	.chip      = TWI_MASTER_ADDR,
	.speed_reg = TWI_BAUD(32000000UL, TWI_SPEED)
};

static void slave_process(void) {
	int i;
	for(i = 0; i < DATA_LENGTH; i++) {
		recv_data[i] = slave.receivedData[i];
	}
}

ISR(TWIF_TWIS_vect) {
	TWI_SlaveInterruptHandler(&slave);
}

void send_and_recv_twi()
{
	twi_package_t packet = {
		.addr_length = 0,
		.chip        = TWI_SLAVE_ADDR,
		.buffer      = (void *)data,
		.length      = DATA_LENGTH,
		.no_wait     = false
	};
	uint8_t i;
	TWI_MASTER_PORT.PIN0CTRL = PORT_OPC_WIREDANDPULL_gc;
	TWI_MASTER_PORT.PIN1CTRL = PORT_OPC_WIREDANDPULL_gc;
	
	irq_initialize_vectors();
	sysclk_enable_peripheral_clock(&TWI_MASTER);
	twi_master_init(&TWI_MASTER, &m_options);
	twi_master_enable(&TWI_MASTER);
	sysclk_enable_peripheral_clock(&TWI_SLAVE);
	TWI_SlaveInitializeDriver(&slave, &TWI_SLAVE, *slave_process);
	TWI_SlaveInitializeModule(&slave, TWI_SLAVE_ADDR,
	TWI_SLAVE_INTLVL_MED_gc);
	
	for (i = 0; i < TWIS_SEND_BUFFER_SIZE; i++) {
		slave.receivedData[i] = 0;
	}
	
	cpu_irq_enable();
	
	twi_master_write(&TWI_MASTER, &packet);
	
	do {
		// Nothing
	} while(slave.result != TWIS_RESULT_OK);
}