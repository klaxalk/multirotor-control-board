/*
 * The configuration file for the main project
 */

#ifndef _CONFIG_H
#define _CONFIG_H

//~ --------------------------------------------------------------------
//~ PWM and PPM constants values configuration
//~ --------------------------------------------------------------------

// PWM constants
#define PULSE_MIN 2304		// PWM
#define PULSE_MAX 4608		// PWM
#define PULSE_MIDDLE 3456	// PWM
#define PPM_PULSE 461		// PWM
#define PPM_FRAME_LENGTH 46080	// PWM and PPM

#define	PULSE_OUT_MIN	1152
#define	PULSE_OUT_MIDDLE	1728

// Constants for 16 Mhz
/*
 * #define pulseMin 2000 		// PWM
 * #define pulseMax 4000 		// PWM
 * #define pulseMiddle 3000 	// PWM
 * #define ppmPulse 800 		// PPM
 * #define frameLength 40000 	// PWM and PPM
 */

// Constants for internal 8 Mhz
/*
 * #define pulseMin 1000
 * #define pulseMax 2000
 * #define pulseMiddle 1500
 * #define ppmPulse 400
 * #define frameLength 20000
 */

//~ --------------------------------------------------------------------
//~ RC channels configuration
//~ --------------------------------------------------------------------

// define RC channel order in
#define ELEVATOR 0		//PA0
#define AILERON 1		//PA1
#define RUDDER 2		//PA2
#define THROTTLE 3		//PA3

#define AUX1 4			//PB0
#define AUX2 5			//PB1
#define AUX3 6			//PB2
#define AUX4 7			//PB3
#define AUX5 8			//PB4

//~ --------------------------------------------------------------------
//~ Basic macros configuration
//~ --------------------------------------------------------------------

// define basic macros
#define led_R_on()  PORTC &= ~_BV(7)
#define led_R_off()  PORTC  |= _BV(7)
#define led_Y_on()  PORTC &= ~_BV(6)
#define led_Y_off()  PORTC |= _BV(6)
#define led_R_toggle() PORTC ^= _BV(7)
#define led_Y_toggle() PORTC ^= _BV(6)
#define led_control_on() PORTA |= _BV(5)
#define led_control_off() PORTA &= ~_BV(5)
#define led_control_toggle() PORTA ^= _BV(5)
#define pulse1_on()  PORTA |= _BV(4) // set output1 to 1
#define pulse1_off()  PORTA &= ~_BV(4) // set output1 to 0

#endif // _CONFIG_H

