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
volatile bool positionControllerEnabled;

/* -------------------------------------------------------------------- */
/*	variables that support altitude controller and estimator			*/
/* -------------------------------------------------------------------- */

#define CONTROLLER_THROTTLE_SATURATION 600

#define ALTITUDE_OUTPUT_FILTER_K	(float) 0.1

#ifdef MIKROKOPTER_KK2

#define ALTITUDE_KP 150
#define ALTITUDE_KI 70
#define ALTITUDE_KV 200 //1200

#endif

#ifdef TRICOPTER

#define ALTITUDE_KP 250
#define ALTITUDE_KI 70
#define ALTITUDE_KV 2000

#endif

#ifdef PRASE

#define ALTITUDE_KP 150
#define ALTITUDE_KI 70
#define ALTITUDE_KV 200

#endif

// constants for altitude and landing controllers
#define ALTITUDE_MAXIMUM	3.00 // used to crop values from PX4Flow
#define ALTITUDE_MINIMUM	0.35 // used for landing (must be > 0.3)
#define ALTITUDE_SPEED_MAX	0.8 // in m/s, must be positive!

// for altitude estimator
volatile float estimatedThrottlePos;
volatile float estimatedThrottleVel;
volatile float estimatedThrottlePos_prev;

// for altitude controller
volatile float throttleSetpoint;
volatile float throttleIntegration;

/* -------------------------------------------------------------------- */
/*	functions for turning controllers on/off							*/
/* -------------------------------------------------------------------- */

void enableAltitudeController(void);
void disableAltitudeController(void);

void enablePositionController(void);
void disableMpcController(void);

/* -------------------------------------------------------------------- */
/*	functions used by controllerTask									*/
/* -------------------------------------------------------------------- */

void altitudeEstimator(void);
void altitudeController(void);

#ifdef PID_POSITION_CONTROLLER

/* -------------------------------------------------------------------- */
/*	For PID position controller											*/
/* -------------------------------------------------------------------- */

volatile float elevator_prev_error;
volatile float aileron_prev_error;
volatile float elevator_integration;
volatile float aileron_integration;
volatile float elevator_reference;
volatile float aileron_reference;

int16_t calculatePID(const float reference, float * prev_error, float * integration, const float position, const float KP, const float KD, const float KI, const float dt, const int16_t saturation);

#endif

#endif // _CONTROLLERS_H
