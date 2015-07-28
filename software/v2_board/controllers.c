/*
 * controllers.c
 *
 * Created: 11.9.2014 13:24:16
 *  Author: Tomas Baca
 */ 

#include "controllers.h"
#include "communication.h"

/* -------------------------------------------------------------------- */
/*	variables that supports controllers in general						*/
/* -------------------------------------------------------------------- */

volatile bool altitudeControllerEnabled;
volatile bool mpcControllerEnabled;

/* -------------------------------------------------------------------- */
/*	variables that support altitude controller and estimator			*/
/* -------------------------------------------------------------------- */

// for altitude estimator
volatile float estimatedThrottlePos = 0;
volatile float estimatedThrottleVel = 0;
volatile uint8_t estimator_cycle = 0;
volatile float   estimatedThrottlePos_prev = 0;

// for altitude controller
volatile float throttleIntegration = 0;
volatile float throttleSetpoint = 0.75;

/* -------------------------------------------------------------------- */
/*	Altitude Estimator - interpolates the data from PX4Flow				*/
/* -------------------------------------------------------------------- */
void altitudeEstimator() {
	//new cycle
	estimator_cycle++;
	
	// extreme filter
	if(fabs(groundDistance - estimatedThrottlePos_prev) <= 0.1) {//limitation cca 3m/s
		// compute new values
		estimatedThrottleVel = (groundDistance - estimatedThrottlePos_prev) / (DT);
		estimatedThrottlePos      = groundDistance;
		estimatedThrottlePos_prev = groundDistance;
		estimator_cycle = 0;
	}
	
	if (estimator_cycle >= 20) { //safety reset
		estimatedThrottleVel = 0;
		estimatedThrottlePos = groundDistance;
		estimatedThrottlePos_prev = groundDistance;
		estimator_cycle = 0;
		
		} else { //estimate position
		estimatedThrottlePos += estimatedThrottleVel * DT;
	}
}

/* -------------------------------------------------------------------- */
/*	Altitude Controller - stabilizes throttle							*/
/* -------------------------------------------------------------------- */
void altitudeController() {
	
	float error;
	float vd; //desired velocity
	float KX, KI, KV;

	KX = ((float)ALTITUDE_KP / ALTITUDE_KV);
	KI = ALTITUDE_KI;
	KV = ALTITUDE_KV;

	error =(throttleSetpoint - estimatedThrottlePos);
	vd = KX * error;
	if(vd > +ALTITUDE_SPEED_MAX) vd = +ALTITUDE_SPEED_MAX;
	if(vd < -ALTITUDE_SPEED_MAX) vd = -ALTITUDE_SPEED_MAX;
	
	// calculate integrational
	throttleIntegration += KI * error * DT;
	if (throttleIntegration > CONTROLLER_THROTTLE_SATURATION*2/3) {
		throttleIntegration = CONTROLLER_THROTTLE_SATURATION*2/3;
	}
	if (throttleIntegration <  -CONTROLLER_THROTTLE_SATURATION*2/3) {
		throttleIntegration = -CONTROLLER_THROTTLE_SATURATION*2/3;
	}

	//total output
	
	portENTER_CRITICAL();
	controllerThrottleOutput =
	KV * (vd - estimatedThrottleVel) + throttleIntegration;
	if (controllerThrottleOutput > CONTROLLER_THROTTLE_SATURATION) {
		controllerThrottleOutput = CONTROLLER_THROTTLE_SATURATION;
	}
	if (controllerThrottleOutput < -CONTROLLER_THROTTLE_SATURATION) {
		controllerThrottleOutput = -CONTROLLER_THROTTLE_SATURATION;
	}
	portEXIT_CRITICAL();
}

void enableAltitudeController() {
	
	if (altitudeControllerEnabled == false) {
		
		throttleIntegration = 0;
	}
	
	altitudeControllerEnabled = true;
}

void disableAltitudeController() {
		
	if (altitudeControllerEnabled == true) {
		
	}
	
	altitudeControllerEnabled = false;
}

void enableMpcController() {
	
	if (mpcControllerEnabled == false) {
		
	}
	
	mpcControllerEnabled = true;
}

void disableMpcController() {
	
	if (mpcControllerEnabled == true) {
		
	}
	
	mpcControllerEnabled = false;
}