/*
 * system.h
 *
 * Created: 10.9.2014 21:53:31
 *  Author: Tomas Baca
 */ 

#ifndef SYSTEM_H_
#define SYSTEM_H_

#include <stdint.h>
#include "usart_driver_RTOS.h"
#include "ioport.h"

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
#define PPM_IN_MIN_LENGTH		2000
#define PPM_IN_MIDDLE_LENGTH	3000
#define PPM_IN_MAX_LENGTH		4000
#define PPM_IN_TRESHOLD			5000

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
/*	Realtime clock														*/
/* -------------------------------------------------------------------- */
extern volatile int16_t milisecondsTimer;
extern volatile uint32_t secondsTimer;

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

//output signals
extern int16_t outputThrottle;
extern int16_t outputElevator;
extern int16_t outputAileron;
extern int16_t outputRudder;

int16_t saturationInt16(int16_t variable,int16_t maxValue);
float saturationFloat(float variable,float maxValue);

/* Basic initialization of the MCU, peripherals and i/o */
void boardInit();

/* Merge signals from RC Receiver with the controller outputs */
void mergeSignalsToOutput();
#endif /* SYSTEM_H_ */