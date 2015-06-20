/*
 * controllers.h
 *
 * Created: 11.9.2014 13:24:27
 *  Author: Tomas Baca
 */

#ifndef _CONTROLLERS_H
#define _CONTROLLERS_H

/* -------------------------------------------------------------------- */
/*	variables that supports controllers in general						*/
/* -------------------------------------------------------------------- */

// controllers period (do not change!)
#define DT	0.0142222

/* -------------------------------------------------------------------- */
/*	variables that support altitude controller and estimator			*/
/* -------------------------------------------------------------------- */

#define CONTROLLER_THROTTLE_SATURATION 300

#define ALTITUDE_KP 100
#define ALTITUDE_KI 100
#define ALTITUDE_KV 200

// constants for altitude and landing controllers
#define ALTITUDE_MAXIMUM	3.00 // used to crop values from PX4Flow
#define ALTITUDE_MINIMUM	0.35 // used for landing (must be > 0.3)
#define ALTITUDE_SPEED_MAX	0.8 // in m/s, must be positive!

/* -------------------------------------------------------------------- */
/*	functions used by controllerTask									*/
/* -------------------------------------------------------------------- */

int altitudeController(float setpoint,float velocity, float position);

#endif // _CONTROLLERS_H
