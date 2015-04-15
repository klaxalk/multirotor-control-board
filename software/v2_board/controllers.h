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
#define DEFAULT_ELEVATOR_POSITION_SETPOINT 0
#define DEFAULT_THROTTLE_POSITION_SETPOINT 1
#define DEFAULT_AILERON_VELOCITY_SETPOINT 0
#define DEFAULT_ELEVATOR_VELOCITY_SETPOINT 0
#define DEFAULT_THROTTLE_VELOCITY_SETPOINT 0

// constants for position and velocity controllers
#define SPEED_MAX 0.4 // in m/s, must be positive!

// constants for altitude and landing controllers
#define ALTITUDE_MAXIMUM  3.00 //used to crop values from PX4Flow
#define ALTITUDE_MINIMUM  0.35 //used for landing (must be > 0.3)
#define ALTITUDE_SPEED_MAX 0.8 //in m/s, must be positive!

// controllers saturations
#define CONTROLLER_ELEVATOR_SATURATION 300
#define CONTROLLER_AILERON_SATURATION  300
#define CONTROLLER_THROTTLE_SATURATION 300

//Elevator Aileron struct
typedef struct{
	float elevator;
	float aileron;
	float altitude;
}state_t;

typedef struct{
	float position;
	float speed;
	float acceleration;
}state2_t;

// controllers output variables
extern volatile int16_t controllerElevatorOutput;
extern volatile int16_t controllerAileronOutput;
extern volatile int16_t controllerRudderOutput;
extern volatile int16_t controllerThrottleOutput;

// controller select
volatile unsigned char controllerActive;

//vars for estimators
extern volatile state_t positionShift;
extern volatile state2_t altitude;

//vars for controllers
extern volatile state_t positionSetpoint;

//landing variables
extern volatile unsigned char landingState;

//trajectory variables and types
typedef struct {
	uint32_t time;
	float elevatorPos; //position in m
	float aileronPos;  //position in m
	float throttlePos; //position in m
} trajectoryPoint_t;

extern volatile int8_t trajMaxIndex;
extern volatile trajectoryPoint_t trajectory[];
extern volatile float MPCElevatorTrajectory[];
extern volatile float MPCAileronTrajectory[];
extern volatile float altitudeTrajectory[];
extern volatile char trajSend;

#define TRAJECTORY_LENGTH	10
#define TRAJ_POINT(i,t,e,a,th) \
trajectory[i].time = t; \
trajectory[i].elevatorPos = e; \
trajectory[i].aileronPos = a; \
trajectory[i].throttlePos = th

//Trajectory
void initTrajectory();
void setpointsCalculate();

//enables - disables
void enableLanding();
void disableLanding();
void controllerSet(unsigned char controllerDesired);

//controllers
void velocityController();
void positionController(float elevatorSetpoint, float aileronSetpoint);

//altitude estimator and controllers
void altitudeEstimator();
void altitudeController(float setpoint);
void landingController();

#endif // _CONTROLLERS_H
