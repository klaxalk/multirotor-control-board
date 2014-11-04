/*
 * controllers.c
 *
 * Created: 11.9.2014 13:24:16
 *  Author: Tomas Baca
 */ 

#include "controllers.h"
#include "communication.h"

// controller on/off
volatile unsigned char controllerEnabled = 0;
volatile unsigned char positionControllerEnabled = 0;

#if PX4FLOW_DATA_RECEIVE == ENABLED

//auto-trajectory variables
volatile unsigned char trajectoryEnabled = 0;
volatile float trajTimer = 0;
volatile int trajIndex = -1;
volatile trajectoryPoint_t trajectory[TRAJECTORY_LENGTH];

static uint8_t estimator_cycle = 0;
static float   estimatedThrottlePos_prev = 0;

//vars for estimators
volatile float estimatedElevatorPos = 0;
volatile float estimatedAileronPos  = 0;
volatile float estimatedThrottlePos = 0;
volatile float estimatedElevatorVel = 0;
volatile float estimatedAileronVel  = 0;
volatile float estimatedThrottleVel = 0;


volatile float elevatorIntegration = 0;
volatile float aileronIntegration  = 0;
volatile float throttleIntegration = 0;
//~ volatile float elevatorSetpoint = (ELEVATOR_SP_LOW + ELEVATOR_SP_HIGH)/2;
volatile float elevatorSetpoint = -1.5;
volatile float aileronSetpoint  = (AILERON_SP_LOW  + AILERON_SP_HIGH )/2;
//~ volatile float throttleSetpoint = (THROTTLE_SP_LOW + THROTTLE_SP_HIGH)/2;
volatile float throttleSetpoint = 0.75;
volatile float elevatorVelocitySetpoint = 0;
volatile float aileronVelocitySetpoint = 0;
volatile float throttleVelocitySetpoint = 0;

//auto-landing variables
volatile unsigned char landingRequest = 0;
volatile unsigned char landingState = LS_ON_GROUND;
volatile uint8_t landingCounter = 0;

//~ ------------------------------------------------------------------------ ~//
//~ Setpoints - assign setpoint values                                       ~//
//~ ------------------------------------------------------------------------ ~//
void setpoints() {

	float sp_new;

#if TRAJECTORY_FOLLOWING == ENABLED

	float dTime;
	float dValue;
	static float aileronIncrement;
	static float elevatorIncrement;
	static float throttleIncrement;

	//trajectory following
	if(trajectoryEnabled && positionControllerEnabled){

		if(trajIndex < TRAJECTORY_LENGTH){

			if(trajIndex < 0 || trajTimer >= trajectory[trajIndex].time) {
				trajIndex++;
				dTime  = (trajectory[trajIndex].time - trajTimer);
				dValue = ((float)trajectory[trajIndex].elevatorPos/1000 - elevatorSetpoint);
				elevatorIncrement = dValue / dTime * DT;
				dValue = ((float)trajectory[trajIndex].aileronPos/1000 - aileronSetpoint);
				aileronIncrement = dValue / dTime * DT;
				dValue = ((float)trajectory[trajIndex].throttlePos/1000 - throttleSetpoint);
				throttleIncrement = dValue / dTime * DT;
			}

			trajTimer  += DT;
			elevatorSetpoint += elevatorIncrement;
			aileronSetpoint  += aileronIncrement;
			throttleSetpoint  += throttleIncrement;


		} //else End of Trajectory - do nothing

		//manual setpoints from RC transmitter
	} else {

#endif //TRAJECTORY_FOLLOWING == ENABLED

		// TODO: ???
		//sp_new = ELEVATOR_SP_HIGH * constant2 + ELEVATOR_SP_LOW * (1-constant2);
		sp_new = (ELEVATOR_SP_LOW + ELEVATOR_SP_HIGH)/2;
		elevatorSetpoint += (sp_new-elevatorSetpoint) * (DT/SETPOINT_FILTER_CONST);

		//sp_new = AILERON_SP_HIGH * constant2 + AILERON_SP_LOW * (1-constant2);
		sp_new = (AILERON_SP_LOW  + AILERON_SP_HIGH )/2;
		aileronSetpoint += (sp_new-aileronSetpoint) * (DT/SETPOINT_FILTER_CONST);

		//sp_new = THROTTLE_SP_HIGH * constant1 + THROTTLE_SP_LOW * (1-constant1);
		
		// zakomentovano, Tom·ö B·Ëa, 24.6.2014
		// sp_new = (THROTTLE_SP_LOW + THROTTLE_SP_HIGH)/2;
		
		sp_new = 1; // TODO: why 1 ?
		
		throttleSetpoint += (sp_new-throttleSetpoint) * (DT/SETPOINT_FILTER_CONST);

#if TRAJECTORY_FOLLOWING == ENABLED

		//reset trajectory vars
		trajIndex = -1;
		trajTimer = 0;
	}

#endif //TRAJECTORY_FOLLOWING == ENABLED

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

	if(validGumstix == 1) {
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

	float KX, KI, KP, KV, KA;

	//set controller constants
	if(positionControllerEnabled && landingState == LS_FLIGHT) {
		KI = POSITION_KI;
		KP = POSITION_KP; // * (0.5 + constant1);
		KV = POSITION_KV; // * (0.5 + constant2);
		KA = POSITION_KA; // * (0.5 + constant5);
		//KP = POSITION_KP;
		//KV = POSITION_KV;
		//KA = POSITION_KA;
		KX = KP / KV;


	} else { //velocity controller
		KI = VELOCITY_KI;
		KV = VELOCITY_KV;
		KA = VELOCITY_KA;
		KX = 0;
 
	}

	//elevator controller
	if(positionControllerEnabled && landingState == LS_FLIGHT) {
		error = elevatorSetpoint - estimatedElevatorPos;
		vd = KX * error;
		if(vd > +POSITION_SPEED_MAX) vd = +POSITION_SPEED_MAX;
		if(vd < -POSITION_SPEED_MAX) vd = -POSITION_SPEED_MAX;
	} else { //velocity controller
		vd = elevatorVelocitySetpoint;
		error = elevatorVelocitySetpoint - estimatedElevatorVel;
	}

	elevatorIntegration += KI * error * DT;
	if (elevatorIntegration > CONTROLLER_ELEVATOR_SATURATION/4) {
		elevatorIntegration = CONTROLLER_ELEVATOR_SATURATION/4;
	} else if (elevatorIntegration < -CONTROLLER_ELEVATOR_SATURATION/4) {
		elevatorIntegration = -CONTROLLER_ELEVATOR_SATURATION/4;
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
	if(positionControllerEnabled && landingState == LS_FLIGHT) {
		error = aileronSetpoint - estimatedAileronPos;
		vd = KX * error;
		if(vd > +POSITION_SPEED_MAX) vd = +POSITION_SPEED_MAX;
		if(vd < -POSITION_SPEED_MAX) vd = -POSITION_SPEED_MAX;
	} else { //velocity controller
		vd = aileronVelocitySetpoint;
		error = aileronVelocitySetpoint - estimatedAileronVel;
	}

	aileronIntegration += KI * error * DT;
	if (aileronIntegration > CONTROLLER_AILERON_SATURATION/4) {
		aileronIntegration = CONTROLLER_AILERON_SATURATION/4;
	} else if (aileronIntegration < -CONTROLLER_AILERON_SATURATION/4) {
		aileronIntegration = -CONTROLLER_AILERON_SATURATION/4;
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

   //new cycle 
   estimator_cycle++;

    if(groundDistance != estimatedThrottlePos_prev) {//input data changed

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

	float error;
	float vd; //desired velocity

	float KX, KI, KV;

	if(landingState == LS_ON_GROUND){
		controllerThrottleOutput = -CONTROLLER_THROTTLE_SATURATION;
		return;

	}else if(landingState == LS_LANDING) {
		KI = LANDING_KI;
		KV = LANDING_KV;

		error = LANDING_SPEED - estimatedThrottleVel;
		vd = LANDING_SPEED;

	} else { //altitude controller
		KX = ((float)ALTITUDE_KP / ALTITUDE_KV);
		KI = ALTITUDE_KI;
		KV = ALTITUDE_KV;

		error = (throttleSetpoint - estimatedThrottlePos);
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

//~ ------------------------------------------------------------------------ ~//
//~ LandingStateAutomat - handles stabilized landing and takeoff             ~//
//~ ------------------------------------------------------------------------ ~//
void landingStateAutomat(){

	switch(landingState){

		case LS_ON_GROUND:
			if(!landingRequest){
				landingCounter = 0;
				landingState = LS_TAKEOFF;
			} break;

		case LS_TAKEOFF:
			if(landingRequest){
				landingCounter = 0;
				landingState = LS_LANDING;
			}else{
				//stabilize altitude for 0.5s
				if(fabs(throttleSetpoint - estimatedThrottlePos) < 0.1
				&& fabs(estimatedThrottleVel ) < 0.2){
					landingCounter++;
				}else{
					landingCounter = 0;
				}
				if(landingCounter >= 35){
					landingState = LS_FLIGHT;
				}
			} break;

		case LS_FLIGHT:
			if(landingRequest){
				landingCounter = 0;
				landingState = LS_STABILIZATION;
			} break;

		case LS_STABILIZATION:
			if(!landingRequest){
				landingState = LS_FLIGHT;
			}else{
				//stabilize position for 1s
				if(++landingCounter >= 70){
					landingCounter = 0;
					landingState = LS_LANDING;
				}
			} break;

		default: //LS_LANDING
			if(!landingRequest){
				landingCounter = 0;
				landingState = LS_TAKEOFF;
			}else{
				//filter wrong sonar readings
				if(estimatedThrottlePos < ALTITUDE_MINIMUM){
					landingCounter++;
				}else{
					landingCounter = 0;
				}
				if(landingCounter >= 7){
					landingState = LS_ON_GROUND;
				}
			}

	} //endswitch

}

#endif // PX4FLOW_DATA_RECEIVE == ENABLED
