/*
 * config.h
 *
 * Created: 11.9.2014 13:25:39
 *  Author: Tomas Baca
 */ 

#ifndef _CONFIG_H
#define _CONFIG_H

#include "defines.h"


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

// on/off the receiving of data from the XBee
#define XBEE_DATA_RECEIVE ENABLED
/*
	ENABLED
	DISABLED
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


#endif // _CONFIG_H