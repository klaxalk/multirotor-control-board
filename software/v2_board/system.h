/*
 * system.h
 *
 * Created: 10.9.2014 21:53:31
 *  Author: Tomas Baca
 */ 

#ifndef SYSTEM_H_
#define SYSTEM_H_

#include <stdint.h>
#include <stdio.h> // sprintf
#include <stdlib.h> // abs
#include "usart_driver_RTOS.h"
#include "ioport.h"
#include "config.h"

/* -------------------------------------------------------------------- */
/*	CPU clock															*/
/* -------------------------------------------------------------------- */
#define F_CPU 32000000UL

/* -------------------------------------------------------------------- */
/*	LED masks															*/
/* -------------------------------------------------------------------- */
#define	RED		IOPORT_CREATE_PIN(PORTB, 0)
#define	BLUE	IOPORT_CREATE_PIN(PORTB, 4)
#define	ORANGE	IOPORT_CREATE_PIN(PORTB, 2)
#define	GREEN	IOPORT_CREATE_PIN(PORTB, 6)
#define	YELLOW	IOPORT_CREATE_PIN(PORTB, 7)
#define	OUT1	IOPORT_CREATE_PIN(PORTD, 5)

/* -------------------------------------------------------------------- */
/*	LED macros															*/
/* -------------------------------------------------------------------- */
#define led_red_on()		ioport_set_pin_level(RED, false)
#define led_red_off()		ioport_set_pin_level(RED, true)
#define led_red_toggle()	ioport_toggle_pin_level(RED)
#define led_blue_on()		ioport_set_pin_level(BLUE, false)
#define led_blue_off()		ioport_set_pin_level(BLUE, true)
#define led_blue_toggle()	ioport_toggle_pin_level(BLUE)
#define led_orange_on()		ioport_set_pin_level(ORANGE, false)
#define led_orange_off()	ioport_set_pin_level(ORANGE, true)
#define led_orange_toggle()	ioport_toggle_pin_level(ORANGE)
#define led_green_on()		ioport_set_pin_level(GREEN, false)
#define led_green_off()	ioport_set_pin_level(GREEN, true)
#define led_green_toggle()	ioport_toggle_pin_level(GREEN)
#define led_yellow_on()		ioport_set_pin_level(YELLOW, false)
#define led_yellow_off()	ioport_set_pin_level(YELLOW, true)
#define led_yellow_toggle()	ioport_toggle_pin_level(YELLOW)

/* -------------------------------------------------------------------- */
/*	PPM output macro													*/
/* -------------------------------------------------------------------- */
#define ppm_out_on()		ioport_set_pin_level(OUT1, true)
#define ppm_out_off()		ioport_set_pin_level(OUT1, false)

/* -------------------------------------------------------------------- */
/*	Constants for PPM input timing										*/
/* -------------------------------------------------------------------- */
#define PPM_IN_MIN_LENGTH		4000
#define PPM_IN_MIDDLE_LENGTH	6000
#define PPM_IN_MAX_LENGTH		8000
#define PPM_IN_TRESHOLD			10000

/* -------------------------------------------------------------------- */
/*	Constants for PWM input timing										*/
/* -------------------------------------------------------------------- */
#define PWM_IN_MIN_LENGTH		4000
#define PWM_IN_MIDDLE_LENGTH	6000
#define PWM_IN_MAX_LENGTH		8000

/* -------------------------------------------------------------------- */
/*	Constants for PPM output timing										*/
/* -------------------------------------------------------------------- */
#define PULSE_OUT_MIN			4000
#define PULSE_OUT_MIDDLE		6000
#define PULSE_OUT_MAX			8000
#define NUMBER_OF_CHANNELS_OUT	6
#define PPM_FRAME_LENGTH		80000
#define PPM_PULSE				1600

/* -------------------------------------------------------------------- */
/*	Constants for USART aliases											*/
/* -------------------------------------------------------------------- */
#define USART_1			USARTD1
#define USART_2			USARTE0
#define USART_3			USARTE1
#define USART_4			USARTF0
#define USART_STM		USARTC1
#define USART_XBEE		USARTC0
#define USART_LOG		USARTD0

/* -------------------------------------------------------------------- */
/*	Constants to asociate the RC channels								*/
/* -------------------------------------------------------------------- */
#define THROTTLE		0
#define RUDDER			1
#define AILERON			2
#define ELEVATOR		3
#define AUX1			4
#define AUX2			5
#define AUX3			6
#define AUX4			7
#define AUX5			8

/* -------------------------------------------------------------------- */
/*	Contains signals from RC receiver									*/
/* -------------------------------------------------------------------- */
volatile uint16_t RCchannel[9];
volatile uint16_t outputChannels[6];

/* -------------------------------------------------------------------- */
/*	Buffers for USARTs													*/
/* -------------------------------------------------------------------- */
UsartBuffer * usart_buffer_stm;
UsartBuffer * usart_buffer_xbee;
UsartBuffer * usart_buffer_log;
UsartBuffer * usart_buffer_1;
UsartBuffer * usart_buffer_2;
UsartBuffer * usart_buffer_3;
UsartBuffer * usart_buffer_4;

/* -------------------------------------------------------------------- */
/*	USART baud rates													*/
/* -------------------------------------------------------------------- */
#define USART_STM_BAUDRATE		BAUD115200
#define USART_XBEE_BAUDRATE		BAUD115200
#define USART_LOG_BAUDRATE		BAUD115200
#define USART_1_BAUDRATE		BAUDPX4FLOW
#define USART_2_BAUDRATE		BAUD115200
#define USART_3_BAUDRATE		BAUD19200
#define USART_4_BAUDRATE		BAUD115200

/* -------------------------------------------------------------------- */
/*	Controllers output to mix with RC and send to FlightCTRL			*/
/* -------------------------------------------------------------------- */
volatile int16_t controllerElevatorOutput;
volatile int16_t controllerAileronOutput;
volatile int16_t controllerThrottleOutput;
volatile int16_t controllerRudderOutput;

/* -------------------------------------------------------------------- */
/*	RTC																	*/
/* -------------------------------------------------------------------- */
volatile int16_t milisecondsTimer;
volatile int16_t secondsTimer;
volatile int16_t hoursTimer;

/* -------------------------------------------------------------------- */
/*	Queue from main to comms											*/
/* -------------------------------------------------------------------- */
xQueueHandle main2commsQueue;

volatile int8_t auxSetpointFlag;

/* Basic initialization of the MCU, peripherals and i/o */
void boardInit();

/* Merge signals from RC Receiver with the controller outputs */
void mergeSignalsToOutput();

void disableController();

void enableController();

void disablePositionController();

void enablePositionController();

#endif /* SYSTEM_H_ */