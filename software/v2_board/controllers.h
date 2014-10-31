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

// time constants for data filters (must be >= DT)
#define GUMSTIX_FILTER_CONST    0.05
#define PX4FLOW_FILTER_CONST    0.05
#define SETPOINT_FILTER_CONST   0.10

//constants for setpoints
#define DEFAULT_AILERON_POSITION_SETPOINT 0
#define DEFAULT_ELEVATOR_POSITION_SETPOINT -1.5
#define DEFAULT_THROTTLE_SETPOINT 1
#define DEFAULT_AILERON_VELOCITY_SETPOINT 0
#define DEFAULT_ELEVATOR_VELOCITY_SETPOINT 0

// constants for position and velocity controllers
#define POSITION_MAXIMUM   2000 //in mm, must be positive! crops Gumstix values
#define SPEED_MAX 0.4 // in m/s, must be positive!

#define VELOCITY_KV 250
#define VELOCITY_KI 10
#define VELOCITY_KA 30

#define POSITION_KP 85
#define POSITION_KI 5
#define POSITION_KV 180
#define POSITION_KA 9

// constants for altitude and landing controllers
#define ALTITUDE_MAXIMUM  3.00 //used to crop values from PX4Flow
#define ALTITUDE_MINIMUM  0.35 //used for landing (must be > 0.3)
#define ALTITUDE_SPEED_MAX 0.8 //in m/s, must be positive!
#define LANDING_SPEED     -0.4 //in m/s, must be negative!

#define THROTTLE_SP_HIGH  2.0
#define THROTTLE_SP_LOW   0.5

#define ALTITUDE_KP 180
#define ALTITUDE_KI 120
#define ALTITUDE_KV 200

#define LANDING_KV 180
#define LANDING_KI 120

// controllers saturations
#define CONTROLLER_ELEVATOR_SATURATION 100
#define CONTROLLER_AILERON_SATURATION  100
#define CONTROLLER_THROTTLE_SATURATION 300

// common global variables
// controllers output variables
extern volatile int16_t velocityControllerElevatorOutput;
extern volatile int16_t velocityControllerAileronOutput;
extern volatile int16_t positionControllerElevatorOutput;
extern volatile int16_t positionControllerAileronOutput;
extern volatile int16_t controllerThrottleOutput;

// controller on/off
extern volatile unsigned char velocityControllerEnabled;
extern volatile unsigned char positionControllerEnabled;

//vars for estimators
extern volatile float estimatedElevatorPos;
extern volatile float estimatedAileronPos;
extern volatile float estimatedThrottlePos;
extern volatile float estimatedElevatorVel;
extern volatile float estimatedAileronVel;
extern volatile float estimatedThrottleVel;
extern volatile float estimatedElevatorAcc;
extern volatile float estimatedAileronAcc;

//vars for controllers - Setpoints
extern volatile float elevatorPositionSetpoint;
extern volatile float aileronPositionSetpoint;
extern volatile float throttleSetpoint;
extern volatile float elevatorVelocitySetpoint;
extern volatile float aileronVelocitySetpoint;

extern volatile float elevatorDesiredPositionSetpoint;
extern volatile float aileronDesiredPositionSetpoint;
extern volatile float throttleDesiredSetpoint;
extern volatile float elevatorDesiredVelocitySetpoint;
extern volatile float aileronDesiredVelocitySetpoint;

//Gumstix on/off
extern volatile unsigned char gumstixEnabled;

//auto-landing variables and state defines
extern volatile float landingThrottleSetpoint;
extern volatile int16_t landingThrottleOutput;
extern volatile unsigned char landingRequest;
extern volatile unsigned char landingState;
#define LS_STABILIZATION      2
#define LS_LANDING            1
#define LS_ON_GROUND          0
#define LS_TAKEOFF            3
#define LS_FLIGHT             4

//auto-trajectory variables and types
extern volatile unsigned char trajectoryEnabled;
extern volatile int8_t trajMaxIndex;
typedef struct {
	float time;
	float elevatorPos; //position in m
	float aileronPos;  //position in m
	float throttlePos; //position in m
} trajectoryPoint_t;
extern volatile trajectoryPoint_t trajectory[];
#define TRAJECTORY_LENGTH	10
#define TRAJ_POINT(i,t,e,a,th) \
	trajectory[i].time = t; \
	trajectory[i].elevatorPos = e; \
	trajectory[i].aileronPos = a; \
	trajectory[i].throttlePos = th

//set default trajectory
void initTrajectory();

//enables - disables
void disableVelocityController();
void enableVelocityController();
void disablePositionController();
void enablePositionController();
void enableGumstix();
void disableGumstix();
void enableLanding();
void disableLanding();

//setpoint and trajectory handling
void setpointsFilter(float throttleDSP,float aileronPosDSP,float elevatorPosDSP,float aileronVelDSP,float elevatorVelDSP);

//position estimator and controllers
void positionEstimator();
void velocityController();
void positionController();

//altitude estimator and controllers
void altitudeEstimator();
void altitudeController();
void landingStateAutomat();

#endif // _CONTROLLERS_H
