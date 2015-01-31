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
#define DEFAULT_THROTTLE_SETPOINT 1
#define DEFAULT_AILERON_VELOCITY_SETPOINT 0
#define DEFAULT_ELEVATOR_VELOCITY_SETPOINT 0

// constants for position and velocity controllers
#define SPEED_MAX 0.4 // in m/s, must be positive!


// constants for altitude and landing controllers
#define ALTITUDE_MAXIMUM  3.00 //used to crop values from PX4Flow
#define ALTITUDE_MINIMUM  0.35 //used for landing (must be > 0.3)
#define ALTITUDE_SPEED_MAX 0.8 //in m/s, must be positive!

#define THROTTLE_SP_HIGH  2.5
#define THROTTLE_SP_LOW   ALTITUDE_MINIMUM

// controllers saturations
#define CONTROLLER_ELEVATOR_SATURATION 100
#define CONTROLLER_AILERON_SATURATION  100
#define CONTROLLER_THROTTLE_SATURATION 300

// leading data TTL (sec)
#define LEADING_DATA_TTL 0.5

// controllers output variables
extern volatile int16_t controllerElevatorOutput;
extern volatile int16_t controllerAileronOutput;
extern volatile int16_t controllerRudderOutput;
extern volatile int16_t controllerThrottleOutput;

// controller select
volatile unsigned char controllerActive;

//vars for estimators

extern volatile float estimatedElevatorPos;
extern volatile float estimatedAileronPos;
extern volatile float estimatedThrottlePos;
extern volatile float estimatedElevatorVel;
extern volatile float estimatedAileronVel;
extern volatile float estimatedThrottleVel;
extern volatile float estimatedElevatorAcc;
extern volatile float estimatedAileronAcc;

extern volatile float estimatedBlobDistance;
extern volatile float estimatedBlobHorizontal;
extern volatile float estimatedBlobVertical;

//vars for controllers
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

//auto-landing variables and state defines
extern volatile unsigned char landingState;

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

//trajectory Follow
void initTrajectory();
void trajectorySetpoints();

//enables - disables
void enableLanding();
void disableLanding();
void enableTrajectoryFollow();
void disableTrajectoryFollow();
void controllerSet(unsigned char controllerDesired);

//position estimator and controllers
void positionEstimator();
void velocityController(int16_t *elevator, int16_t *aileron, float elevatorSetpoint, float aileronSetpoint);
void positionController(int16_t *elevator, int16_t *aileron, float elevatorSetpoint, float aileronSetpoint);

//altitude estimator and controllers
void altitudeEstimator();
void altitudeController(int16_t *throttle, float setpoint);
void landingController();

//leading data age checker
void leadingDataActualCheck();

#endif // _CONTROLLERS_H
