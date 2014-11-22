/*
 * controllers.c
 *
 * Created: 11.9.2014 13:24:16
 *  Author: Tomas Baca
 */ 

#include "controllers.h"
#include "communication.h"

// controller on/off
volatile bool altitudeControllerEnabled;
volatile bool mpcControllerEnabled;

static uint8_t estimator_cycle = 0;
static float   estimatedThrottlePos_prev = 0;

// vars for altitude estimators
volatile float estimatedThrottlePos = 0;
volatile float estimatedThrottleVel = 0;

volatile float throttleIntegration = 0;
volatile float throttleSetpoint = 0.75;

//~ ------------------------------------------------------------------------ ~//
//~ Altitude Estimator - interpolates the data from PX4Flow sonar sensor     ~//
//~ ------------------------------------------------------------------------ ~//
void altitudeEstimator() {

	//new cycle 
	estimator_cycle++;

	//input data changed
    if(groundDistance != estimatedThrottlePos_prev) {

        // extreme filter
		if(fabs(groundDistance - estimatedThrottlePos_prev) <= 0.5) {//limitation cca 3m/s

           // compute new values 
           estimatedThrottleVel = (groundDistance - estimatedThrottlePos_prev) / (7*DT);
           estimatedThrottlePos      = groundDistance;
           estimatedThrottlePos_prev = groundDistance;
           estimator_cycle = 0;

        }

    } else {

        if (estimator_cycle >= 8) { //safety reset

             estimatedThrottleVel = 0;
             estimatedThrottlePos = groundDistance;
             estimatedThrottlePos_prev = groundDistance;
             estimator_cycle = 0;

        } else { //estimate position

            estimatedThrottlePos += estimatedThrottleVel * DT;

        }
    }
}

//~ ------------------------------------------------------------------------ ~//
//~ Altitude Controller - stabilizes throttle                                ~//
//~ ------------------------------------------------------------------------ ~//
void altitudeController() {

	//desired velocity
	float error;

	// control constants
	float KX = (float) (ALTITUDE_KP / ALTITUDE_KV);
	float KI = ALTITUDE_KI;
	float KV = ALTITUDE_KV;
	
	// altitude error
	error = throttleSetpoint - estimatedThrottlePos;
	
	// compute the desired velocity
	float vd = KX * error;
	
	// desired velocity saturation
	if (vd > +ALTITUDE_SPEED_MAX)
		vd = +ALTITUDE_SPEED_MAX;
	
	if (vd < -ALTITUDE_SPEED_MAX)
		vd = -ALTITUDE_SPEED_MAX;

	// calculate integration
	throttleIntegration += KI * error * DT;
	if (throttleIntegration > +CONTROLLER_THROTTLE_SATURATION*2/3) {
		throttleIntegration = +CONTROLLER_THROTTLE_SATURATION*2/3;
	}
	if (throttleIntegration < -CONTROLLER_THROTTLE_SATURATION*2/3) {
		throttleIntegration = -CONTROLLER_THROTTLE_SATURATION*2/3;
	}
	
	// forbid context switch
	// prevents unsaturated values being passed from this function
	portENTER_CRITICAL();

	controllerThrottleOutput = KV * (vd - estimatedThrottleVel) + throttleIntegration;
	
	if (controllerThrottleOutput > +CONTROLLER_THROTTLE_SATURATION) {
		controllerThrottleOutput = +CONTROLLER_THROTTLE_SATURATION;
	}
	if (controllerThrottleOutput < -CONTROLLER_THROTTLE_SATURATION) {
		controllerThrottleOutput = -CONTROLLER_THROTTLE_SATURATION;
	}

	// allow context switch
	portEXIT_CRITICAL();
}

void enableAltitudeController() {
	
	if (altitudeControllerEnabled == false) {
		
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