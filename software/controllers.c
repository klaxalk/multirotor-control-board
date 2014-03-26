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
//TODO remove
//~ ------------------------------------------------------------------------ ~//
// elevator speed controller using px4flow data
#define ELEVATOR_SPEED_KP 235
#define ELEVATOR_SPEED_KD 2
#define POSITION_KP_GUMSTIX 0.05
#define GUMSTIX_CONTROLLER_SATURATION 0.2
#define POSITION_KI_GUMSTIX 0.02
void controllerElevatorSpeed() {

	float KP = ELEVATOR_SPEED_KP;
	float KD = ELEVATOR_SPEED_KD;
	//~ float KI = ELEVATOR_SPEED_KI;

	//~ float error = elevatorSpeedSetpoint+elevatorSpeed;
	float error = - elevatorSpeed;

#if GUMSTIX_DATA_RECEIVE == ENABLED

	float positionError = elevatorGumstix - elevatorSetpoint;

	if (positionError > 2000) {
		positionError = 2000;
	} else if (positionError < -2000) {
		positionError = -2000;
	}

	// calculate position P
	float positionProportional = POSITION_KP_GUMSTIX*positionError;

	// calculate adaptive offset
	int8_t smer;
	if (positionError > 0) {
		smer = 1;
	} else {
		smer = -1;
	}

	// calculate position I
	gumstixElevatorIntegral += POSITION_KI_GUMSTIX*smer;

#endif // GUMSTIX_DATA_RECEIVE == ENABLED

	// calculate P
	float proportional = KP*error;

	//~ // calculate angular
	//~ float angular = -pitchAngle;

	// calulate D
	float derivative = KD*(error-elevatorSpeedPreviousError);

	//~ // calculate I
	//~ float integrational = KI*elevatorSpeedIntegration;
//~
	//~ if (constant2 < 1) {
		//~ integrational = 0;
	//~ }

	elevatorSpeedPreviousError = error;

#if GUMSTIX_DATA_RECEIVE == ENABLED

	if (validGumstix == 1 && (abs(elevatorSpeed) < GUMSTIX_CONTROLLER_SATURATION)) {
		controllerElevatorOutput = proportional + derivative + positionProportional + gumstixElevatorIntegral;
	} else {
		controllerElevatorOutput = proportional + derivative;
	}

#elif ATOM_DATA_RECEIVE == ENABLED

	controllerElevatorOutput = proportional + derivative - pitchAngle;

#else

	controllerElevatorOutput = proportional + derivative;

#endif

	// controller saturation
	if (controllerElevatorOutput > CONTROLLER_ELEVATOR_SATURATION) {
		controllerElevatorOutput = CONTROLLER_ELEVATOR_SATURATION;
	} else if (controllerElevatorOutput < -CONTROLLER_ELEVATOR_SATURATION) {
		controllerElevatorOutput = -CONTROLLER_ELEVATOR_SATURATION;
	}
}

//~ ------------------------------------------------------------------------ ~//
//~ Velocity Controller - stabilizes Elevator and Aileron to zero speed      ~//
//~ ------------------------------------------------------------------------ ~//
void velocityController() {

	static float elevatorSpeed_prev = 0;
      static float elevatorAcc_filt  = 0;
      static float aileronSpeed_prev = 0;
      static float aileronAcc_filt  = 0;
	float acc_new;

	float error;
	float proportional;
	float derivative;
	//float integrational;

	float KP = VELOCITY_KP;
	float KI = VELOCITY_KI;
	float KD = VELOCITY_KD;

	//TODO elevator controller

      //TODO remove
	controllerElevatorSpeed();

	//aileron controller
	error = - estimatedAileronVel;
	proportional = KP * error;
	
	aileronIntegration += KI * error * DT;
	if (aileronIntegration > CONTROLLER_AILERON_SATURATION/2) {
		aileronIntegration = CONTROLLER_AILERON_SATURATION/2;
	} else if (aileronIntegration < -CONTROLLER_AILERON_SATURATION/2) {
		aileronIntegration = -CONTROLLER_AILERON_SATURATION/2;
	} 

      acc_new = (estimatedAileronVel - aileronSpeed_prev) / DT;
      aileronSpeed_prev = estimatedAileronVel;
	aileronAcc_filt += (acc_new - aileronAcc_filt) * (DT/PX4FLOW_FILTER_CONST);
	derivative = -1 * KD * aileronAcc_filt;

	controllerAileronOutput = proportional + aileronIntegration + derivative;
	if (controllerAileronOutput > CONTROLLER_AILERON_SATURATION) {
		controllerAileronOutput = CONTROLLER_AILERON_SATURATION;
	} else if (controllerAileronOutput < -CONTROLLER_AILERON_SATURATION) {
		controllerAileronOutput = -CONTROLLER_AILERON_SATURATION;
	}

}

//~ ------------------------------------------------------------------------ ~//
//~ Position Controller - stabilizes Elevator and Aileron to a fixed point   ~//
//~ ------------------------------------------------------------------------ ~//
void positionController() {

	float error;
	float proportional;
	float derivative;
	//float integrational;

	float KP = POSITION_KP;
	float KI = POSITION_KI;
	float KD = POSITION_KD;

	//TODO elevator controller

      //TODO remove
	controllerElevatorSpeed();

	//aileron controller
	error = aileronSetpoint - estimatedAileronPos;
	proportional = KP * error;

	aileronIntegration += KI * error * DT;
	if (aileronIntegration > CONTROLLER_AILERON_SATURATION/2) {
		aileronIntegration = CONTROLLER_AILERON_SATURATION/2;
	} else if (aileronIntegration < -CONTROLLER_AILERON_SATURATION/2) {
		aileronIntegration = -CONTROLLER_AILERON_SATURATION/2;
	}

	derivative = -1 * KD * estimatedAileronVel;

	controllerAileronOutput = proportional + aileronIntegration + derivative;
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
	if (throttleIntegration > CONTROLLER_THROTTLE_SATURATION/2) {
		throttleIntegration = CONTROLLER_THROTTLE_SATURATION/2;
	}
	if (throttleIntegration <  -CONTROLLER_THROTTLE_SATURATION/2) {
		throttleIntegration = -CONTROLLER_THROTTLE_SATURATION/2;
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
		if (throttleIntegration > CONTROLLER_THROTTLE_SATURATION/2) {
			throttleIntegration = CONTROLLER_THROTTLE_SATURATION/2;
		}
		if (throttleIntegration <  -CONTROLLER_THROTTLE_SATURATION/2) {
			throttleIntegration = -CONTROLLER_THROTTLE_SATURATION/2;
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

