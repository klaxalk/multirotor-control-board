/*
 * config.h
 *
 * Created: 11.9.2014 13:25:39
 *  Author: Tomas Baca
 */ 

#ifndef _CONFIG_H
#define _CONFIG_H

#include "defines.h"

// Choose if the logging is on
#define LOGGING_ON ENABLED
/*
	ENABLED
	DISABLED

*/

// Choose if trajectory following is on
#define TRAJECTORY_FOLLOWING ENABLED
/*
	ENABLED
	DISABLED

*/

//~ --------------------------------------------------------------------
//~ Choose a orientation of the quadcopter frame
//~ --------------------------------------------------------------------

// Choose the multirotor frame orientation
#define FRAME_ORIENTATION X_COPTER
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
//~ Choose the PX4Flow camera orientation
//~ --------------------------------------------------------------------

// Choose where the XP4Flow's X axis is pointing
#define PX4_CAMERA_ORIENTATION BACKWARD
/*
	FORWARD
	BACKWARD

*/

//~ --------------------------------------------------------------------
//~ Receiving data from FlightCTRL configuration
//~ --------------------------------------------------------------------


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

// default BAUDs
#ifndef BAUD0
#define BAUD0 57600
#endif

#ifndef BAUD1
#define BAUD1 57600
#endif

#define PITCH_BUFFER_SIZE 8
#define ROLL_BUFFER_SIZE 8

#endif // _CONFIG_H