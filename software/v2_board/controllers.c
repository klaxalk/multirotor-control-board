/*
* controllers.c
*
* Created: 11.9.2014 13:24:16
*  Author: Tomas Baca
*/

#include "controllers.h"
#include "communication.h"
#include "battery.h"
#include "commTask.h"
#include "mpcHandler.h"

/* -------------------------------------------------------------------- */
/*	variables that supports controllers in general						*/
/* -------------------------------------------------------------------- */

volatile bool altitudeControllerEnabled;
volatile bool mpcControllerEnabled;
volatile float lastGoodGroundDistance;
volatile float groundDistanceConfidence;
uint8_t cyclesSinceGoodDistance = 0;
uint8_t position = 0;
volatile char kalmanStarted = 0;
volatile float kalmanFit = 20;
float differences[20] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

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
volatile float throttleSetpoint = 1;

/* -------------------------------------------------------------------- */
/*	Altitude Estimator - interpolates the data from PX4Flow				*/
/* -------------------------------------------------------------------- */

void altitudeEstimator() {
	//new cycle
	estimator_cycle++;
	
	// extreme filter
	if(fabs(groundDistance - estimatedThrottlePos_prev) <= 0.2) {//limitation cca 3m/s
		// compute new values
		estimatedThrottleVel = ((groundDistance - estimatedThrottlePos_prev) / (7*DT));
		estimatedThrottlePos      = groundDistance;
		estimatedThrottlePos_prev = groundDistance;
		estimator_cycle = 0;
	}
	
	if (estimator_cycle >= 30) { //safety reset
		
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

	KX = ((float) ALTITUDE_KP / (float) ALTITUDE_KV);
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
	controllerThrottleOutput = (int16_t) (KV * (vd - estimatedThrottleVel) + throttleIntegration);
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

/* -------------------------------------------------------------------- */
/*	Altitude Evaluator - computes confidence and sends data to kalman	*/
/* -------------------------------------------------------------------- */

void altitudeEvaluateAndSendToKalman() {
	controller2commMessage_t message;
	message.messageType = SEND_INPUTS;
	message.data.groundDistance = groundDistance;
	message.data.batteryLevel = batteryLevel;
	message.data.throttleInput = outputChannels[0];	
	
	if (lastGoodGroundDistance == groundDistance)  { //rate unchanged value
		
		groundDistanceConfidence = ((float) 1) - ((float) (cyclesSinceGoodDistance * cyclesSinceGoodDistance)) / 256;
		
		if (cyclesSinceGoodDistance++ >= 16) { //reject long unchanged value
			
			groundDistanceConfidence = (float) 0;
			
		}
		
		} else {
		
		if (fabs(lastGoodGroundDistance - groundDistance) > GRND_DIST_DIFF_MAX) { //reject likely erroneous reading based on difference
			
			groundDistanceConfidence = ((float) (((uint16_t) 1) << cyclesSinceGoodDistance)) / 65536; //full confidence after 16 cycles (0.8 s)
			
			if (cyclesSinceGoodDistance++ >= 16) { //accept steady yet differentiated distance reading
				
				lastGoodGroundDistance = groundDistance;
				groundDistanceConfidence = (float) 1;
				cyclesSinceGoodDistance = 0;
				
			}
			
			} else { //accept good distance reading
			
			lastGoodGroundDistance = groundDistance;
			groundDistanceConfidence = (float) 1;
			cyclesSinceGoodDistance = 0;
			
		}
	}
	message.data.groundDistanceConfidence = groundDistanceConfidence;
	
	xQueueSend(controller2commsQueue, &message, 0);
}

void altitudeSendToKalman() {
	controller2commMessage_t message;
	message.messageType = SEND_INPUTS;
	message.data.groundDistance = groundDistance;
	message.data.batteryLevel = batteryLevel;
	message.data.throttleInput = outputChannels[0];
	lastGoodGroundDistance = groundDistance;
	groundDistanceConfidence = (float) 1;
	message.data.groundDistanceConfidence = groundDistanceConfidence;
	
	xQueueSend(controller2commsQueue, &message, 0);
}

void kalmanStep() {
	
	kalmanFit -= differences[position];
	differences[position] = fabs(kalmanStates.throttle.position - groundDistance);
	kalmanFit += differences[position];
	position = ((position < 19) ? (position + 1) : 0);
	
	if(kalmanStarted) {
		altitudeEvaluateAndSendToKalman();
		led_green_on();
	} else {
		altitudeSendToKalman();
		kalmanStarted = (kalmanFit < KALMAN_FIT_THRESHOLD);
		led_green_toggle();
	}
}

void calculateNextThrottle() {
	float error;
	float temp = 0;
	
	temp += (ALTITUDE_K1 * kalmanStates.throttle.position);
	temp += (ALTITUDE_K2 * kalmanStates.throttle.velocity);
	temp += (ALTITUDE_K3 * kalmanStates.throttle.acceleration);
	temp += (ALTITUDE_K4 * kalmanStates.throttle.omega);
	
	error = throttleSetpoint - kalmanStates.throttle.position;
	throttleIntegration += error * DT;
	if (throttleIntegration > CONTROLLER_THROTTLE_SATURATION*2/3) {
		throttleIntegration = CONTROLLER_THROTTLE_SATURATION*2/3;
	}
	if (throttleIntegration <  -CONTROLLER_THROTTLE_SATURATION*2/3) {
		throttleIntegration = -CONTROLLER_THROTTLE_SATURATION*2/3;
	}
	temp += (ALTITUDE_K5 * throttleIntegration);
	
	
	if (temp > CONTROLLER_THROTTLE_SATURATION) {
		temp = CONTROLLER_THROTTLE_SATURATION;
	}
	
	if (temp < -CONTROLLER_THROTTLE_SATURATION) {
		temp = -CONTROLLER_THROTTLE_SATURATION;
	}
	
	portENTER_CRITICAL();
	controllerThrottleOutput = (int16_t) temp;
	portEXIT_CRITICAL();
}

void resetThrottleKalman() {
	controller2commMessage_t message;
	message.messageType = CLEAR_STATE;
	message.data.groundDistance = groundDistance;
	xQueueSend(controller2commsQueue, &message, 0);
}