/*
 * The configuration file for the main project
 */

#ifndef _CONFIG_H
#define _CONFIG_H

#include "defines.h"

//~ --------------------------------------------------------------------
//~ Choose a orientation of the quadcopter frame
//~ --------------------------------------------------------------------

// if the logging in on
#define LOGGING_ON ENABLED
/*
	ENABLED
	DISABLED

*/

// Choose the multirotor frame orientation
#define FRAME_ORIENTATION PLUS_COPTER
/*
	PLUS_COPTER
	X_COPTER

*/

//~ --------------------------------------------------------------------
//~ Choose a where is the Gumstix camera pointing
//~ --------------------------------------------------------------------

// Choose the multirotor frame orientation
#define GUMSTIX_CAMERA_POINTING FORWARD
/*
	FORWARD
	DOWNWARD

*/

//~ --------------------------------------------------------------------
//~ Receiving data from FlightCTRL configuration
//~ --------------------------------------------------------------------

// on/off the receiving of the Angles from the FlightCTRL
#define FLIGHTCTRL_DATA_RECEIVE DISABLED
/*
	ENABLED
	DISABLED
*/

// define the UART port for communication with the FlightCTRL
#define FLIGHTCTRL_RECEIVE_PORT UART1
/*
	UART0
	UART1
*/

//~ --------------------------------------------------------------------
//~ Receiving data from GumStix configuration
//~ --------------------------------------------------------------------

// on/off the receiving of data from the GumStix
#define GUMSTIX_DATA_RECEIVE ENABLED
/*
	ENABLED
	DISABLED
*/

// define the UART port for communication with the GumStix
#define GUMSTIX_RECEIVE_PORT UART0
/*
	UART0
	UART1
*/

//~ --------------------------------------------------------------------
//~ Receiving data from px4flow configuration
//~ --------------------------------------------------------------------

// on/off the receiving of data from the px4flow
#define PX4FLOW_DATA_RECEIVE ENABLED
/*
	ENABLED
	DISABLED
*/

// define the UART port for communication with the px4flow
#define PX4FLOW_RECEIVE_PORT UART1
/*
	UART0
	UART1
*/

//~ --------------------------------------------------------------------
//~ Receiving data from Atom computer (surfnav) configuration
//~ --------------------------------------------------------------------

// on/off the receiving of data from the atom computer
#define ATOM_DATA_RECEIVE DISABLED
/*
	ENABLED
	DISABLED
*/

// define the UART port for communication with the atom
#define ATOM_RECEIVE_PORT UART0
/*
	UART0
	UART1
*/

//~ --------------------------------------------------------------------
//~ Config baud rates for UARTs
//~ --------------------------------------------------------------------

#if FLIGHTCTRL_DATA_RECEIVE == ENABLED

#if FLIGHTCTRL_RECEIVE_PORT == UART0
#define BAUD0 57600
#endif

#if FLIGHTCTRL_RECEIVE_PORT == UART1
#define BAUD1 57600
#endif

#endif

#if GUMSTIX_DATA_RECEIVE == ENABLED

#if GUMSTIX_RECEIVE_PORT == UART0
#define BAUD0 57600
#endif

#if GUMSTIX_RECEIVE_PORT == UART1
#define BAUD1 57600
#endif

#endif

#if PX4FLOW_DATA_RECEIVE == ENABLED

#if PX4FLOW_RECEIVE_PORT == UART0
#define BAUD0 115200
#endif

#if PX4FLOW_RECEIVE_PORT == UART1
#define BAUD1 115200
#endif

#endif

#if ATOM_DATA_RECEIVE == ENABLED

#if ATOM_RECEIVE_PORT == UART0
#define BAUD0 115200
#endif

#if ATOM_RECEIVE_PORT == UART1
#define BAUD1 115200
#endif

#endif

// default BAUDs
#ifndef BAUD0
#define BAUD0 57600
#endif

#ifndef BAUD1
#define BAUD1 57600
#endif

//~ --------------------------------------------------------------------
//~ PWM and PPM constants values configuration
//~ --------------------------------------------------------------------

// PWM constants
#define PULSE_MIN 2304		// PWM
#define PULSE_MAX 4608		// PWM
#define PULSE_MIDDLE 3456	// PWM
#define PPM_PULSE 922		// PWM
#define PPM_FRAME_LENGTH 46080	// PWM and PPM

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

#define PITCH_BUFFER_SIZE 8
#define ROLL_BUFFER_SIZE 8

#endif // _CONFIG_H

