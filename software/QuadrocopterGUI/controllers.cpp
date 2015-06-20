/*
 * controllers.c
 *
 * Created: 11.9.2014 13:24:16
 *  Author: Tomas Baca
 */

#include "controllers.h"
//#include "communication.h"

/* -------------------------------------------------------------------- */
/*	variables that supports controllers in general						*/
/* -------------------------------------------------------------------- */


/* -------------------------------------------------------------------- */
/*	variables that support altitude controller and estimator			*/
/* -------------------------------------------------------------------- */

volatile float   estimatedThrottlePos_prev = 0;

// for altitude controller
volatile float throttleIntegration = 0;


/* -------------------------------------------------------------------- */
/*	Altitude Controller - stabilizes throttle							*/
/* -------------------------------------------------------------------- */
int altitudeController(float throttleSetpoint,float estimatedThrottleVel, float estimatedThrottlePos) {

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

    int controllerThrottleOutput =
            KV * (vd - estimatedThrottleVel) + throttleIntegration;
    if (controllerThrottleOutput > CONTROLLER_THROTTLE_SATURATION) {
        controllerThrottleOutput = CONTROLLER_THROTTLE_SATURATION;
    }
    if (controllerThrottleOutput < -CONTROLLER_THROTTLE_SATURATION) {
        controllerThrottleOutput = -CONTROLLER_THROTTLE_SATURATION;
    }

    return controllerThrottleOutput;
}

