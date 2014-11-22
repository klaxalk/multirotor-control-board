/*
 * controllers.h
 *
 * Created: 11.9.2014 13:24:27
 *  Author: Tomas Baca
 */ 

#ifndef _CONTROLLERS_H
#define _CONTROLLERS_H

#include "system.h"
#include <stdlib.h>

// controllers period (do not change!)
#define DT	0.0142222

// constants for altitude and landing controllers
#define ALTITUDE_MAXIMUM  3.00 // used to crop values from PX4Flow
#define ALTITUDE_MINIMUM  0.35 // used for landing (must be > 0.3)
#define ALTITUDE_SPEED_MAX 0.8 // in m/s, must be positive!

#define THROTTLE_SP_HIGH  1.5
#define THROTTLE_SP_LOW   0.5

#define ALTITUDE_KP 180
#define ALTITUDE_KI 120
#define ALTITUDE_KV 200

// controller on/off
extern volatile bool altitudeControllerEnabled;
extern volatile bool mpcControllerEnabled;

// controllers saturations
#define CONTROLLER_ELEVATOR_SATURATION 100
#define CONTROLLER_AILERON_SATURATION  100
#define CONTROLLER_THROTTLE_SATURATION 300

extern volatile float estimatedThrottlePos;
extern volatile float estimatedThrottleVel;

// vars for controllers
extern volatile float throttleIntegration;
extern volatile float throttleSetpoint;

void enableAltitudeController();
void disableAltitudeController();
void enableMpcController();
void disableMpcController();

// altitude estimator and controllers
void altitudeEstimator();
void altitudeController();

#endif // _CONTROLLERS_H
