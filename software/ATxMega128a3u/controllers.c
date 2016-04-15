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
volatile bool positionControllerEnabled;

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
volatile float throttleSetpoint = 1.0;

#ifdef PID_POSITION_CONTROLLER

/* -------------------------------------------------------------------- */
/*	For PID position controller											*/
/* -------------------------------------------------------------------- */

volatile float elevator_prev_error = 0;
volatile float aileron_prev_error = 0;
volatile float elevator_integration = 0;
volatile float aileron_integration = 0;

volatile float elevator_reference = 0;
volatile float aileron_reference = 0;

#endif

/* -------------------------------------------------------------------- */
/*	Altitude Estimator - interpolates the data from PX4Flow				*/
/* -------------------------------------------------------------------- */
void altitudeEstimator(void) {
	//new cycle
	estimator_cycle++;
	
		// extreme filter 
		if(fabs(groundDistance - estimatedThrottlePos_prev) <= 0.2) { // limitation cca 3m/s
			
			// compute new values
			estimatedThrottleVel = (1-ALTITUDE_OUTPUT_FILTER_K)*estimatedThrottleVel + (ALTITUDE_OUTPUT_FILTER_K)*((groundDistance - estimatedThrottlePos_prev) / (1*DT));
			estimatedThrottlePos      = groundDistance;
			estimatedThrottlePos_prev = groundDistance;
			estimator_cycle = 0;
		}
		
		if (estimator_cycle >= 71) { //safety reset
			
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
void altitudeController(void) {
	
	float error;
	float vd; //desired velocity
	float KX, KI, KV;
	float unfilteredOutput;

	KX = ((float) ALTITUDE_KP / (float) ALTITUDE_KV);
	KI = ALTITUDE_KI;
	KV = ALTITUDE_KV;

	error = (throttleSetpoint - estimatedThrottlePos);
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
	
	unfilteredOutput = (KV * (vd - estimatedThrottleVel) + throttleIntegration);

	//total output
	portENTER_CRITICAL();
	
	controllerThrottleOutput = (int16_t) (unfilteredOutput);
	
	// saturate
	if (controllerThrottleOutput > CONTROLLER_THROTTLE_SATURATION) {
		controllerThrottleOutput = CONTROLLER_THROTTLE_SATURATION;
	}
	
	if (controllerThrottleOutput < -CONTROLLER_THROTTLE_SATURATION) {
		controllerThrottleOutput = -CONTROLLER_THROTTLE_SATURATION;
	}
	
	portEXIT_CRITICAL();
}

void enableAltitudeController(void) {
	
	if (altitudeControllerEnabled == false) {
		
		throttleIntegration = 0;
	}
	
	altitudeControllerEnabled = true;
}

void disableAltitudeController(void) {
		
	if (altitudeControllerEnabled == true) {
		
	}
	
	altitudeControllerEnabled = false;
}

void enablePositionController(void) {
	
	if (positionControllerEnabled == false) {
		
		#ifdef PID_POSITION_CONTROLLER
		
		elevator_integration = 0;
		aileron_integration = 0;
		elevator_prev_error = 0;
		aileron_prev_error = 0;
		
		#endif
	}
	
	positionControllerEnabled = true;
}

void disableMpcController(void) {
	
	if (positionControllerEnabled == true) {
		
	}
	
	positionControllerEnabled = false;
}

#ifdef PID_POSITION_CONTROLLER

int16_t calculatePID(const float reference, float * prev_error, float * integration, const float position, const float KP, const float KD, const float KI, const float dt, const int16_t saturation) {
	
	// temp variables
	float error;
	int16_t output;
	
	// calculate tha actual control error and filter it
	error = (reference - position)*0.1 + (*prev_error)*0.9;
	
	// calculate the controllers output
	output = (int16_t) (KP*error + KD*((error - *prev_error)/dt) + KI*(*integration));
	
	// saturate the controllers output
	if (output > saturation)
		output = saturation;
	// saturate the other side
	else if (output < -saturation)
		output = -saturation;
	// integrate the adaptive offset
	else {
		if (error > 0)
			*integration += 1;
		else
			*integration += -1;	
	}

	// save the actual error to the previous error
	*prev_error = error;
	
	return output;
}

#endif