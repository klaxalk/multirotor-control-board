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
//~ What MCU is this code compiled for?
//~ ... Used for some platform specific code in matrixLib
//~ --------------------------------------------------------------------

#define USED_MCU XMEGA
/*
	XMEGA
	ARM
*/

#endif // _CONFIG_H