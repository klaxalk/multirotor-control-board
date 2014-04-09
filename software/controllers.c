/*
 * This file contains sources of system controllers
 */

#include "controllers.h"

#if PX4FLOW_DATA_RECEIVE == ENABLED

//~ ------------------------------------------------------------------------ ~//
//~ Setpoints - assign setpoint values                                       ~//
//~ ------------------------------------------------------------------------ ~//
void setpoints() {

	float sp_new, dTime;
	int16_t dValue;
	static float aileronIncrement;
	static float elevatorIncrement;

      //manual setpoints
	if(!trajectoryEnabled){

		//setpoint setting from RC transmitter
		//sp_new = ELEVATOR_SP_HIGH * constant2 + ELEVATOR_SP_LOW * (1-constant2);
		//elevatorSetpoint += (sp_new-elevatorSetpoint) * (DT/SETPOINT_FILTER_CONST);

		sp_new = AILERON_SP_HIGH * constant2 + AILERON_SP_LOW * (1-constant2);
		aileronSetpoint += (sp_new-aileronSetpoint) * (DT/SETPOINT_FILTER_CONST);

		sp_new = THROTTLE_SP_HIGH * constant1 + THROTTLE_SP_LOW * (1-constant1);
		throttleSetpoint += (sp_new-throttleSetpoint) * (DT/SETPOINT_FILTER_CONST);

      //trajectory following
	}else if(trajIndex < TRAJECTORY_LENGTH){

		if(trajIndex < 0 || trajTimer >= trajectory[trajIndex].time) {
			trajIndex++;
			dTime  = (trajectory[trajIndex].time - trajTimer);
			dValue = (trajectory[trajIndex].elevatorPos - elevatorSetpoint);
			elevatorIncrement = ((float)dValue) / 1000 / dTime;
			dValue = (trajectory[trajIndex].aileronPos - aileronSetpoint);
			aileronIncrement = ((float)dValue) / 1000 / dTime;
		}

		trajTimer  += DT;
		elevatorSetpoint += elevatorIncrement;
		aileronSetpoint  += aileronIncrement;

	}

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
	float derivative2;
	float vd; //desired velocity

	float KX, KI, KV, KA;

	//set controller constants
	if(positionControllerEnabled) {
		KX = (POSITION_KP / POSITION_KV);
		KI = POSITION_KI;
		KV = POSITION_KV;
		KA = POSITION_KA;

	} else { //velocity controller
		KI = VELOCITY_KI;
		KV = VELOCITY_KV;
		KA = VELOCITY_KA;
 
	}

	//elevator controller
	if(positionControllerEnabled) {
		error = elevatorSetpoint - estimatedElevatorPos;
		vd = KX * error;
		if(vd > +POSITION_SPEED_MAX) vd = +POSITION_SPEED_MAX;
		if(vd < -POSITION_SPEED_MAX) vd = -POSITION_SPEED_MAX;
	} else { //velocity controller
		vd = 0;
		error = - estimatedElevatorVel;
	}

	elevatorIntegration += KI * error * DT;
	if (elevatorIntegration > CONTROLLER_ELEVATOR_SATURATION/2) {
		elevatorIntegration = CONTROLLER_ELEVATOR_SATURATION/2;
	} else if (elevatorIntegration < -CONTROLLER_ELEVATOR_SATURATION/2) {
		elevatorIntegration = -CONTROLLER_ELEVATOR_SATURATION/2;
	}

	acc_new = (estimatedElevatorVel - elevatorSpeed_prev) / DT;
	elevatorSpeed_prev = estimatedElevatorVel;
	elevatorAcc_filt += (acc_new - elevatorAcc_filt) * (DT/PX4FLOW_FILTER_CONST);
	derivative2 = -1 * KA * elevatorAcc_filt;

	controllerElevatorOutput =
		KV * (vd - estimatedElevatorVel) + elevatorIntegration + derivative2;
	if (controllerElevatorOutput > CONTROLLER_ELEVATOR_SATURATION) {
		controllerElevatorOutput = CONTROLLER_ELEVATOR_SATURATION;
	} else if (controllerElevatorOutput < -CONTROLLER_ELEVATOR_SATURATION) {
		controllerElevatorOutput = -CONTROLLER_ELEVATOR_SATURATION;
	}


	//aileron controller
	if(positionControllerEnabled) {
		error = aileronSetpoint - estimatedAileronPos;
		vd = KX * error;
		if(vd > +POSITION_SPEED_MAX) vd = +POSITION_SPEED_MAX;
		if(vd < -POSITION_SPEED_MAX) vd = -POSITION_SPEED_MAX;
	} else { //velocity controller
		vd = 0;
		error = - estimatedAileronVel;
	}

	aileronIntegration += KI * error * DT;
	if (aileronIntegration > CONTROLLER_AILERON_SATURATION/2) {
		aileronIntegration = CONTROLLER_AILERON_SATURATION/2;
	} else if (aileronIntegration < -CONTROLLER_AILERON_SATURATION/2) {
		aileronIntegration = -CONTROLLER_AILERON_SATURATION/2;
	} 

	acc_new = (estimatedAileronVel - aileronSpeed_prev) / DT;
	aileronSpeed_prev = estimatedAileronVel;
	aileronAcc_filt += (acc_new - aileronAcc_filt) * (DT/PX4FLOW_FILTER_CONST);
	derivative2 = -1 * KA * aileronAcc_filt;

	controllerAileronOutput =
		KV * (vd - estimatedAileronVel) + aileronIntegration + derivative2;
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
//~ Altitude Controller - stabilizes throttle                                ~//
//~ ------------------------------------------------------------------------ ~//
void altitudeController() {

	float error;
	float vd; //desired velocity

	float KX, KI, KV;

	if(landingMode) {
		KI = LANDING_KI;
		KV = LANDING_KV;

		error = LANDING_SPEED - estimatedThrottleVel;
		vd = LANDING_SPEED;

	} else { //altitude controller
		KX = (ALTITUDE_KP / ALTITUDE_KV);
		KI = ALTITUDE_KI;
		KV = ALTITUDE_KV;

		error =(throttleSetpoint - estimatedThrottlePos);
		vd = KX * error;
		if(vd > +ALTITUDE_SPEED_MAX) vd = +ALTITUDE_SPEED_MAX;
		if(vd < -ALTITUDE_SPEED_MAX) vd = -ALTITUDE_SPEED_MAX;

	}

	// calculate integrational
	throttleIntegration += KI * error * DT;
	if (throttleIntegration > CONTROLLER_THROTTLE_SATURATION*2/3) {
		throttleIntegration = CONTROLLER_THROTTLE_SATURATION*2/3;
	}
	if (throttleIntegration <  -CONTROLLER_THROTTLE_SATURATION*2/3) {
		throttleIntegration = -CONTROLLER_THROTTLE_SATURATION*2/3;
	}

	//total output
	controllerThrottleOutput =
		KV * (vd - estimatedThrottleVel) + throttleIntegration;
	if (controllerThrottleOutput > CONTROLLER_THROTTLE_SATURATION) {
		controllerThrottleOutput = CONTROLLER_THROTTLE_SATURATION;
	}
	if (controllerThrottleOutput < -CONTROLLER_THROTTLE_SATURATION) {
		controllerThrottleOutput = -CONTROLLER_THROTTLE_SATURATION;
	}

}

#endif // PX4FLOW_DATA_RECEIVE == ENABLED
