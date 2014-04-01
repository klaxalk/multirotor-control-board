/*
 * This file contains sources of system controllers
 */

#include "controllers.h"

#if PX4FLOW_DATA_RECEIVE == ENABLED

//~ ------------------------------------------------------------------------ ~//
//~ Setpoints - assign setpoint values                                       ~//
//~ ------------------------------------------------------------------------ ~//
void setpoints() {

	float sp_new;

	//setpoint setting from RC transmitter
	//sp_new = ELEVATOR_SP_HIGH * constant2 + ELEVATOR_SP_LOW * (1-constant2);
	//elevatorSetpoint += (sp_new-elevatorSetpoint) * (DT/SETPOINT_FILTER_CONST);

	sp_new = AILERON_SP_HIGH * constant2 + AILERON_SP_LOW * (1-constant2);
	aileronSetpoint += (sp_new-aileronSetpoint) * (DT/SETPOINT_FILTER_CONST);

	sp_new = THROTTLE_SP_HIGH * constant1 + THROTTLE_SP_LOW * (1-constant1);
	throttleSetpoint += (sp_new-throttleSetpoint) * (DT/SETPOINT_FILTER_CONST);

}

//~ ------------------------------------------------------------------------ ~//
//~ Position Estimator - estimates Elevator and Aileron position and         ~//
//~                      velocity from PX4flow and Gumstix data              ~//
//~ ------------------------------------------------------------------------ ~//
void positionEstimator() {

#if GUMSTIX_DATA_RECEIVE == ENABLED

	//gustix valid delay - filters faulty values
	static uint8_t gumstix_counter = 0;
	uint8_t gumstix_delay = 7;

	if(validGumstix) {
		if(gumstix_counter < gumstix_delay) gumstix_counter++;
	} else {
		gumstix_counter = 0;
	}

#endif

	//elevator velocity
	estimatedElevatorVel += (elevatorSpeed-estimatedElevatorVel) * (DT/PX4FLOW_FILTER_CONST);

	//elevator position
#if GUMSTIX_DATA_RECEIVE == ENABLED
	if(gumstix_counter == gumstix_delay) {
		estimatedElevatorPos += (elevatorGumstix-estimatedElevatorPos) * (DT/GUMSTIX_FILTER_CONST);
	}else{
#endif
		estimatedElevatorPos += estimatedElevatorVel * DT;
#if GUMSTIX_DATA_RECEIVE == ENABLED
	}
#endif

	//aileron velocity
	estimatedAileronVel += (aileronSpeed-estimatedAileronVel) * (DT/PX4FLOW_FILTER_CONST);

	//aileron position
#if GUMSTIX_DATA_RECEIVE == ENABLED
	if(gumstix_counter == gumstix_delay) {
		estimatedAileronPos += (aileronGumstix-estimatedAileronPos) * (DT/GUMSTIX_FILTER_CONST);
	}else{
#endif
            estimatedAileronPos += estimatedAileronVel * DT;
#if GUMSTIX_DATA_RECEIVE == ENABLED
	}
#endif

}

//~ ------------------------------------------------------------------------ ~//
//~ Position Controller - stabilizes Elevator and Aileron                    ~//
//~ ------------------------------------------------------------------------ ~//
void positionController() {

	static float elevatorSpeed_prev = 0;
	static float elevatorAcc_filt  = 0;
	static float aileronSpeed_prev = 0;
	static float aileronAcc_filt  = 0;
	float acc_new;

	float error;
	float proportional;
	float derivative1;
	float derivative2;
	//float integrational;

	float KP, KI, KV, KA;

	if(positionControllerEnabled) {
		KP = POSITION_KP;
		KI = POSITION_KI;
		KV = POSITION_KV;
		KA = POSITION_KA;

	} else { //velocity controller
		KP = VELOCITY_KP;
		KI = VELOCITY_KI;
		KV = 0;
		KA = VELOCITY_KD;
	}

	//elevator controller
	if(positionControllerEnabled) {
		error = elevatorSetpoint - estimatedElevatorPos;
	} else {
		error = - estimatedElevatorVel;
	}
	proportional = KP * error;

	elevatorIntegration += KI * error * DT;
	if (elevatorIntegration > CONTROLLER_ELEVATOR_SATURATION/2) {
		elevatorIntegration = CONTROLLER_ELEVATOR_SATURATION/2;
	} else if (elevatorIntegration < -CONTROLLER_ELEVATOR_SATURATION/2) {
		elevatorIntegration = -CONTROLLER_ELEVATOR_SATURATION/2;
	}

	derivative1 = -1 * KV * estimatedElevatorVel;

	acc_new = (estimatedElevatorVel - elevatorSpeed_prev) / DT;
	elevatorSpeed_prev = estimatedElevatorVel;
	elevatorAcc_filt += (acc_new - elevatorAcc_filt) * (DT/PX4FLOW_FILTER_CONST);
	derivative2 = -1 * KA * elevatorAcc_filt;

	controllerElevatorOutput = proportional + elevatorIntegration + derivative1 + derivative2;
	if (controllerElevatorOutput > CONTROLLER_ELEVATOR_SATURATION) {
		controllerElevatorOutput = CONTROLLER_ELEVATOR_SATURATION;
	} else if (controllerElevatorOutput < -CONTROLLER_ELEVATOR_SATURATION) {
		controllerElevatorOutput = -CONTROLLER_ELEVATOR_SATURATION;
	}


	//aileron controller
	if(positionControllerEnabled) {
		error = aileronSetpoint - estimatedAileronPos;
	} else {
		error = - estimatedAileronVel;
	}
	proportional = KP * error;
	
	aileronIntegration += KI * error * DT;
	if (aileronIntegration > CONTROLLER_AILERON_SATURATION/2) {
		aileronIntegration = CONTROLLER_AILERON_SATURATION/2;
	} else if (aileronIntegration < -CONTROLLER_AILERON_SATURATION/2) {
		aileronIntegration = -CONTROLLER_AILERON_SATURATION/2;
	} 

	derivative1 = -1 * KV * estimatedAileronVel;

	acc_new = (estimatedAileronVel - aileronSpeed_prev) / DT;
	aileronSpeed_prev = estimatedAileronVel;
	aileronAcc_filt += (acc_new - aileronAcc_filt) * (DT/PX4FLOW_FILTER_CONST);
	derivative2 = -1 * KA * aileronAcc_filt;

	controllerAileronOutput = proportional + aileronIntegration + derivative1 + derivative2;
	if (controllerAileronOutput > CONTROLLER_AILERON_SATURATION) {
		controllerAileronOutput = CONTROLLER_AILERON_SATURATION;
	} else if (controllerAileronOutput < -CONTROLLER_AILERON_SATURATION) {
		controllerAileronOutput = -CONTROLLER_AILERON_SATURATION;
	}

}

//~ ------------------------------------------------------------------------ ~//
//~ Altitude Estimator - interpolates the data fron PX4Flow sonar sensor     ~//
//~ ------------------------------------------------------------------------ ~//
void altitudeEstimator() {

   static uint8_t estimator_cycle = 0;
   static float   estimatedThrottlePos_prev = 0;

   //new cycle 
   estimator_cycle++;

    if(groundDistance != estimatedThrottlePos_prev) {//input data changed

        // extreme filter
        if(abs(groundDistance - estimatedThrottlePos_prev) <= 0.3) {//limitation cca 3m/s

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
//~ Altitude Controller - stabilizes throttle to a fixed altitude            ~//
//~ ------------------------------------------------------------------------ ~//
void altitudeController() {

	float KP = ALTITUDE_KP;
	float KD = ALTITUDE_KD;
	float KI = ALTITUDE_KI;	

	float error = throttleSetpoint - estimatedThrottlePos;

	// calculate proportional
	float proportional = KP * error;

	// calculate integrational
	throttleIntegration += KI * error * DT;
	if (throttleIntegration > CONTROLLER_THROTTLE_SATURATION*2/3) {
		throttleIntegration = CONTROLLER_THROTTLE_SATURATION*2/3;
	}
	if (throttleIntegration <  -CONTROLLER_THROTTLE_SATURATION*2/3) {
		throttleIntegration = -CONTROLLER_THROTTLE_SATURATION*2/3;
	}

	// calculate derivative
	float derivative = -1 * KD * estimatedThrottleVel;
	
	//total output
	controllerThrottleOutput = proportional + throttleIntegration + derivative;
	if (controllerThrottleOutput > CONTROLLER_THROTTLE_SATURATION) {
		controllerThrottleOutput = CONTROLLER_THROTTLE_SATURATION;
	}
	if (controllerThrottleOutput < -CONTROLLER_THROTTLE_SATURATION) {
		controllerThrottleOutput = -CONTROLLER_THROTTLE_SATURATION;
	}

}

//~ ------------------------------------------------------------------------ ~//
//~ Landing Controller - performs autonomous landing                         ~//
//~ ------------------------------------------------------------------------ ~//
void landingController() {

	float KP = ALTITUDE_KP;
	float KI = ALTITUDE_KI;

	//float landingSpeed = - 0.4; //m/s
	//float landingDecrease = 25; //cmd/s

	static float landing_counter = 0;

	if(estimatedThrottlePos > ALTITUDE_MINIMUM)
	{
		landing_counter = 0;

		float error = (LANDING_SPEED - estimatedThrottleVel);

		// calculate proportional
		float proportional = KP * error;

		// calculate integrational
		throttleIntegration += KI * error * DT;
		if (throttleIntegration > CONTROLLER_THROTTLE_SATURATION*2/3) {
			throttleIntegration = CONTROLLER_THROTTLE_SATURATION*2/3;
		}
		if (throttleIntegration <  -CONTROLLER_THROTTLE_SATURATION*2/3) {
			throttleIntegration = -CONTROLLER_THROTTLE_SATURATION*2/3;
		}

		//total output
		controllerThrottleOutput = proportional + throttleIntegration;
	}
	else if(controllerThrottleOutput > -CONTROLLER_THROTTLE_SATURATION)
	{
		//final landing stage
		if(++landing_counter >= 8){
			controllerThrottleOutput = -CONTROLLER_THROTTLE_SATURATION;
		}
	}

	//output saturation
	if (controllerThrottleOutput > CONTROLLER_THROTTLE_SATURATION) {
		controllerThrottleOutput = CONTROLLER_THROTTLE_SATURATION;
	}
	if (controllerThrottleOutput < -CONTROLLER_THROTTLE_SATURATION) {
		controllerThrottleOutput = -CONTROLLER_THROTTLE_SATURATION;
	}

}

#endif // PX4FLOW_DATA_RECEIVE == ENABLED

