/*
 * This file contains sources of system controllers
 */

#ifndef _CONTROLLERS_H
#define _CONTROLLERS_H

#include "mavlink/v1.0/common/mavlink.h"
#include "defines.h"
#include "config.h"
#include <stdlib.h>

// controllers period (do not change!)
#define DT	0.0142222

// time constants for data filters (must be >= DT)
#define GUMSTIX_FILTER_CONST    0.05
#define PX4FLOW_FILTER_CONST    0.05
#define SETPOINT_FILTER_CONST   0.10

// constants for position and velocity controllers
#define ELEVATOR_SP_HIGH  -1.0
#define ELEVATOR_SP_LOW   -2.0

#define AILERON_SP_HIGH   +0.5
#define AILERON_SP_LOW    -0.5

#define POSITION_SPEED_MAX 0.6 //in m/s, must be positive!

#define VELOCITY_KV 250
#define VELOCITY_KI 10
#define VELOCITY_KA 30

#define POSITION_KP 110
#define POSITION_KI 5
#define POSITION_KV 212
#define POSITION_KA 20

// constants for altitude and landing controllers
#define ALTITUDE_MAXIMUM  3.00 //used to crop values from PX4Flow
#define ALTITUDE_MINIMUM  0.35 //used for landing (must be > 0.3)
#define ALTITUDE_SPEED_MAX 0.8 //in m/s, must be positive!
#define LANDING_SPEED     -0.4 //in m/s, must be negative!

#define THROTTLE_SP_HIGH  1.5
#define THROTTLE_SP_LOW   0.5

#define ALTITUDE_KP 180
#define ALTITUDE_KI 120
#define ALTITUDE_KV 200

#define LANDING_KV 180
#define LANDING_KI 120

// common global variables
// controllers output variables
extern volatile int16_t controllerElevatorOutput;
extern volatile int16_t controllerAileronOutput;
extern volatile int16_t controllerThrottleOutput;

// controller on/off
extern volatile unsigned char controllerEnabled;
extern volatile unsigned char positionControllerEnabled;
extern volatile unsigned char landingMode;

// constants from RC transmitter
extern volatile float constant1;
extern volatile float constant2;
extern volatile float constant3;
extern volatile float constant4;
extern volatile float constant5;

// controllers saturations
#define CONTROLLER_ELEVATOR_SATURATION 100
#define CONTROLLER_AILERON_SATURATION  100
#define CONTROLLER_THROTTLE_SATURATION 300

#if PX4FLOW_DATA_RECEIVE == ENABLED

//px4flow values
extern volatile float groundDistance;
extern volatile float elevatorSpeed;
extern volatile float aileronSpeed;

#if GUMSTIX_DATA_RECEIVE == ENABLED

//gumstix values
extern volatile float elevatorGumstix;
extern volatile float aileronGumstix;
extern volatile float throttleGumstix;
extern volatile int8_t validGumstix;

#endif

//vars for estimators
extern volatile float estimatedElevatorPos;
extern volatile float estimatedAileronPos;
extern volatile float estimatedThrottlePos;
extern volatile float estimatedElevatorVel;
extern volatile float estimatedAileronVel;
extern volatile float estimatedThrottleVel;

//vars for controllers
extern volatile float elevatorIntegration;
extern volatile float aileronIntegration;
extern volatile float throttleIntegration;
extern volatile float elevatorSetpoint;
extern volatile float aileronSetpoint;
extern volatile float throttleSetpoint;

//auto-landing variables and state defines
extern volatile unsigned char landingRequest;
extern volatile unsigned char landingState;
extern volatile uint8_t landingCounter;
#define LS_ON_GROUND          0
#define LS_STABILIZATION      1
#define LS_LANDING            2
#define LS_TAKEOFF            3
#define LS_FLIGHT             4

//auto-trajectory variables

//setpoint and trajectory handling
void setpoints();

//position estimator and controllers
void positionEstimator();
void velocityController();
void positionController();

//altitude estimator and controllers
void altitudeEstimator();
void altitudeController();
void landingStateAutomat();

#endif // PX4FLOW_DATA_RECEIVE == ENABLED

#endif // _CONTROLLERS_H
