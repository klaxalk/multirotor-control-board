/*
 * This file contains sources of system controllers
 */

#include "controllers.h"

// aileron position controller using surfnav
void controllerAileron_surfnav() {

	float KP = AILERON_POSITION_KP_SURFNAV;

	float error = xPosSurf;

	aileronSpeedSetPoint = KP*error;

	if (aileronSpeedSetPoint > SURFNAV_CONTROLLER_SATURATION) {
		aileronSpeedSetPoint = SURFNAV_CONTROLLER_SATURATION;
	} else if (aileronSpeedSetPoint < -SURFNAV_CONTROLLER_SATURATION) {
		aileronSpeedSetPoint = -SURFNAV_CONTROLLER_SATURATION;
	}
}

// elevator position controller using surfnav
void controllerElevator_surfnav() {

	float KP = ELEVATOR_POSITION_KP_SURFNAV;

	float error = yPosSurf;

	elevatorSpeedSetpoint = KP*error;

	if (elevatorSpeedSetpoint > SURFNAV_CONTROLLER_SATURATION) {
		elevatorSpeedSetpoint = SURFNAV_CONTROLLER_SATURATION;
	} else if (elevatorSpeedSetpoint < -SURFNAV_CONTROLLER_SATURATION) {
		elevatorSpeedSetpoint = -SURFNAV_CONTROLLER_SATURATION;
	}
}

// aileron speed controller using px4flow data
void controllerAileronSpeed() {

	float KP = AILERON_SPEED_KP;
	float KD = AILERON_SPEED_KD;

	float error = aileronSpeedSetPoint-aileronSpeed;

	// calculate P
	float proportional = KP*error;

	//~ // calculate angular
	//~ float angular = -rollAngle;

	// calculate D
	float derivative = KD*(error-aileronSpeedPreviousError);

	aileronSpeedPreviousError = error;

	controllerAileronOutput = proportional + derivative;

	// controller saturation
	if (controllerAileronOutput > CONTROLLER_AILERON_SATURATION) {
		controllerAileronOutput = CONTROLLER_AILERON_SATURATION;
	} else if (controllerAileronOutput < -CONTROLLER_AILERON_SATURATION) {
		controllerAileronOutput = -CONTROLLER_AILERON_SATURATION;
	}
}

// elevator speed controller using px4flow data
void controllerElevatorSpeed() {

	float KP = ELEVATOR_SPEED_KP;
	float KD = ELEVATOR_SPEED_KD;

	float error = (elevatorSpeedSetpoint+elevatorSpeed);

	// calculate P
	float proportional = KP*error;

	//~ // calculate angular
	//~ float angular = -pitchAngle;

	// calulate D
	float derivative = KD*(error-elevatorSpeedPreviousError);

	elevatorSpeedPreviousError = error;

	controllerElevatorOutput = proportional + derivative;

	// controller saturation
	if (controllerElevatorOutput > CONTROLLER_ELEVATOR_SATURATION) {
		controllerElevatorOutput = CONTROLLER_ELEVATOR_SATURATION;
	} else if (controllerElevatorOutput < -CONTROLLER_ELEVATOR_SATURATION) {
		controllerElevatorOutput = -CONTROLLER_ELEVATOR_SATURATION;
	}
}

// altitude controller
void controllerThrottle() {

	float KP = ALTITUDE_KP;
	float KD = ALTITUDE_KD;
	float KI = ALTITUDE_KI;
	float error;

	//~ if (cameraConfiguration == 0) // dolu
	//~ error = -(cameraZ-controllerThrottleSetPoint); // pro sledovani dolu
	//~ else // dopredu
	//~ error = -cameraX+altitudeSetPoint; // pro sledovani dopredu

	error = (ALTITUDE_SETPOINT - ((groundDistance)*((float) 1000)));

	// calculate proportional

	float proportional = KP*error;

	// calculate derivative

	float derivative = KD*(error-throttlePreviousError);

	throttlePreviousError = error;

	// calculate integrational

	float integrational = throttleIntegration;

	controllerThrottleOutput = proportional + derivative + integrational;

	// integrate
	throttleIntegration += KI*error;

	if (throttleIntegration > CONTROLLER_THROTTLE_SATURATION) {
		throttleIntegration = CONTROLLER_THROTTLE_SATURATION;
	}
	if (throttleIntegration <  -CONTROLLER_THROTTLE_SATURATION) {
		throttleIntegration = -CONTROLLER_THROTTLE_SATURATION;
	}

	// saturace regulatoru
	if (controllerThrottleOutput > 3*CONTROLLER_THROTTLE_SATURATION) {
		controllerThrottleOutput = 3*CONTROLLER_THROTTLE_SATURATION;
	}

	// nelinearita
	if (controllerThrottleOutput < 0) {
		controllerThrottleOutput = (int16_t)((float) controllerThrottleOutput)*0.1;  //0.2
	} else {
		controllerThrottleOutput = (int16_t)((float) controllerThrottleOutput)*1.5;  // 1.5
	}
}
