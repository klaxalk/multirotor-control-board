/*
 * This file contains sources of system controllers
 */

#ifndef _CONTROLLERS_H
#define _CONTROLLERS_H

#include "mavlink/v1.0/common/mavlink.h"
#include "defines.h"

// constants for surfnav position controllers
#define ELEVATOR_POSITION_KP_SURFNAV 0.001
#define AILERON_POSITION_KP_SURFNAV 0.001
#define SURFNAV_CONTROLLER_SATURATION 0.2
#define SURFNAV_FILTER_WEIGHT 0.3

// constants for px4flow speed controllers
#define AILERON_SPEED_KP 190
#define AILERON_SPEED_KD 5
#define ELEVATOR_SPEED_KP 190
#define ELEVATOR_SPEED_KD 5
#define PX4FLOW_FILTER_WEIGHT 0.3

// constants for altitude controller
#define ALTITUDE_KP 0.1
#define ALTITUDE_KD 110
#define ALTITUDE_KI 0.0002
#define ALTITUDE_SETPOINT 1500

// variables for altitude controller
extern volatile int16_t throttlePreviousError;
extern volatile float throttleIntegration;

// common global variables
// controllers output variables
extern volatile int16_t controllerElevatorOutput;
extern volatile int16_t controllerAileronOutput;
extern volatile int16_t controllerThrottleOutput;

// constants from RC transmitter
extern volatile float constant1;
extern volatile float constant2;
extern volatile float constant3;

// controllers saturations
#define CONTROLLER_ELEVATOR_SATURATION 150
#define CONTROLLER_AILERON_SATURATION 150
#define CONTROLLER_THROTTLE_SATURATION 150

// variables for px4flow speed controller
extern volatile float aileronSpeedSetPoint;
extern volatile float elevatorSpeedSetpoint;
extern mavlink_optical_flow_t opticalFlowData;
extern volatile float elevatorSpeedPreviousError;
extern volatile float aileronSpeedPreviousError;
extern volatile float elevatorSpeed;
extern volatile float aileronSpeed;
extern volatile float groundDistance;

// variables for surfnav position controller
extern volatile int16_t xPosSurf;
extern volatile int16_t yPosSurf;
extern volatile int16_t headingSurf;

void controllerAileronSpeed();

void controllerElevatorSpeed();

void controllerAileron_surfnav();

void controllerElevator_surfnav();

void controllerThrottle();

#endif // _CONTROLLERS_H

