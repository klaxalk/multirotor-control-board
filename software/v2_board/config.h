/*
 * config.h
 *
 * Created: 11.9.2014 13:25:39
 *  Author: Tomas Baca
 */ 

#ifndef _CONFIG_H
#define _CONFIG_H

#include "defines.h"

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
//~ Receiving data from GumStix configuration
//~ --------------------------------------------------------------------

// on/off the receiving of data from the GumStix
#define GUMSTIX_DATA_RECEIVE ENABLED
/*
	ENABLED
	DISABLED
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


//~ --------------------------------------------------------------------
//~ communication with onboard PC
//~ --------------------------------------------------------------------

#ifndef PC_COMMUNICATION
#define PC_COMMUNICATION DISABLED
#endif
/*
	ENABLED
	DISABLED
*/

#if PC_COMMUNICATION == ENABLED
	#ifndef PC_USART_BUFFER
	#define PC_USART_BUFFER usart_buffer_1 // TODO: set properly to used port
	#endif
#endif

// disable the trajectory following IF there are no data for control
#if (PX4FLOW_DATA_RECEIVE == DISABLED && GUMSTIX_DATA_RECEIVE == DISABLED)

	#undef	TRAJECTORY_FOLLOWING
	#define	TRAJECTORY_FOLLOWING	DISABLED

	#warning TRAJECTORY_FOLLOWING DISABLED because PX4FLOW_DATA_RECEIVE and GUMSTIX_DATA_RECEIVE is disabled

#endif 

#endif // _CONFIG_H
