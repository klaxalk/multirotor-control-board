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

#define CONTROLLER_THROTTLE_SATURATION 600

#ifdef MIKROKOPTER_KK2

#define ALTITUDE_KP 200
#define ALTITUDE_KI 100
#define ALTITUDE_KV 1200

#endif

#ifdef TRICOPTER

#define ALTITUDE_KP 250
#define ALTITUDE_KI 70
#define ALTITUDE_KV 2000

#endif

#ifdef PRASE

#define ALTITUDE_KP 180
#define ALTITUDE_KI 70
#define ALTITUDE_KV 2000

#endif

// constants for altitude and landing controllers
#define ALTITUDE_MAXIMUM	2.00 // used to crop values from PX4Flow
#define ALTITUDE_MINIMUM	0.35 // used for landing (must be > 0.3)
#define ALTITUDE_SPEED_MAX	0.8 // in m/s, must be positive!

// for altitude estimator
volatile float estimatedThrottlePos;
volatile float estimatedThrottleVel;
volatile float estimatedThrottlePos_prev;

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
