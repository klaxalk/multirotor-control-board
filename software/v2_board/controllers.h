/*
 * controllers.h
 *
 * Created: 11.9.2014 13:24:27
 *  Author: Tomas Baca
 */ 

#ifndef _CONTROLLERS_H
#define _CONTROLLERS_H

#include "system.h"

/* -------------------------------------------------------------------- */
/*	variables that supports controllers in general						*/
/* -------------------------------------------------------------------- */

// controllers period (do not change!)
#define DT	0.0142222

volatile bool altitudeControllerEnabled;
volatile bool mpcControllerEnabled;

/* -------------------------------------------------------------------- */
/*	variables that support altitude controller and estimator			*/
/* -------------------------------------------------------------------- */

#define CONTROLLER_THROTTLE_SATURATION 300

#define ALTITUDE_KP 150
#define ALTITUDE_KI 70
#define ALTITUDE_KV 180

#define THROTTLE_SP_HIGH  1.5
#define THROTTLE_SP_LOW   0.5

// constants for altitude and landing controllers
#define ALTITUDE_MAXIMUM	3.00 // used to crop values from PX4Flow
#define ALTITUDE_MINIMUM	0.35 // used for landing (must be > 0.3)
#define ALTITUDE_SPEED_MAX	0.8 // in m/s, must be positive!

// for altitude estimator
volatile float estimatedThrottlePos;

// for altitude controller
volatile float throttleSetpoint;

/* -------------------------------------------------------------------- */
/*	functions for turning controllers on/off							*/
/* -------------------------------------------------------------------- */

void enableAltitudeController();
void disableAltitudeController();

void enableMpcController();
void disableMpcController();

/* -------------------------------------------------------------------- */
/*	functions used by controllerTask									*/
/* -------------------------------------------------------------------- */

void altitudeEstimator();
void altitudeController();

#endif // _CONTROLLERS_H
