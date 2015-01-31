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
/*	variables that supports MPC											*/
/* -------------------------------------------------------------------- */

#define MPC_SATURATION	350

volatile int16_t mpcElevatorOutput;
volatile int16_t mpcAileronOutput;

// this structure hold setpoints for elevator and aileron position
typedef struct {
	
	float elevatorSetpoint;
	float aileronSetpoint;
} mpcSetpoints_t;

volatile mpcSetpoints_t mpcSetpoints;

/* -------------------------------------------------------------------- */
/*	variables that support kalman filter (running on STM)				*/
/* -------------------------------------------------------------------- */

// this struct contains variables for elevator&aileron axis
typedef struct {
	
	volatile float position;
	volatile float velocity;
	volatile float acceleration;
	
} oneAxisStates_t;

// this struct contains states computed by kalman filter in STM MCU
typedef struct {

	oneAxisStates_t elevator;
	oneAxisStates_t aileron;
	
} kalmanStates_t;

volatile kalmanStates_t kalmanStates;

/* -------------------------------------------------------------------- */
/*	variables that support altitude controller and estimator			*/
/* -------------------------------------------------------------------- */

#define CONTROLLER_THROTTLE_SATURATION 300

#define ALTITUDE_KP 180
#define ALTITUDE_KI 120
#define ALTITUDE_KV 200

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
