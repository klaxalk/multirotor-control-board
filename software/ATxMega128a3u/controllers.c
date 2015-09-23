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
<<<<<<< HEAD
//~ volatile float elevatorSetpoint = (ELEVATOR_SP_LOW + ELEVATOR_SP_HIGH)/2;
volatile float elevatorSetpoint = -1.5;
volatile float aileronSetpoint  = (AILERON_SP_LOW  + AILERON_SP_HIGH )/2;
//~ volatile float throttleSetpoint = (THROTTLE_SP_LOW + THROTTLE_SP_HIGH)/2;
volatile float throttleSetpoint = 0.75;

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
	}else{

#endif //TRAJECTORY_FOLLOWING == ENABLED

		//sp_new = ELEVATOR_SP_HIGH * constant2 + ELEVATOR_SP_LOW * (1-constant2);
		sp_new = (ELEVATOR_SP_LOW + ELEVATOR_SP_HIGH)/2;
		elevatorSetpoint += (sp_new-elevatorSetpoint) * (DT/SETPOINT_FILTER_CONST);

		//sp_new = AILERON_SP_HIGH * constant2 + AILERON_SP_LOW * (1-constant2);
		sp_new = (AILERON_SP_LOW  + AILERON_SP_HIGH )/2;
		aileronSetpoint += (sp_new-aileronSetpoint) * (DT/SETPOINT_FILTER_CONST);

		//sp_new = THROTTLE_SP_HIGH * constant1 + THROTTLE_SP_LOW * (1-constant1);
		
		// zakomentovano, Tom·ö B·Ëa, 24.6.2014
		// sp_new = (THROTTLE_SP_LOW + THROTTLE_SP_HIGH)/2;
		
		sp_new = 1;
		
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
		// TODO [HCH] add requested velocity setpoint
		vd = 0;
		error = - estimatedElevatorVel;
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

	portENTER_CRITICAL();
	
	controllerElevatorOutput =
		KV * (vd - estimatedElevatorVel) + elevatorIntegration + derivative2;
	if (controllerElevatorOutput > CONTROLLER_ELEVATOR_SATURATION) {
		controllerElevatorOutput = CONTROLLER_ELEVATOR_SATURATION;
	} else if (controllerElevatorOutput < -CONTROLLER_ELEVATOR_SATURATION) {
		controllerElevatorOutput = -CONTROLLER_ELEVATOR_SATURATION;
	}
	
	portEXIT_CRITICAL();

	//aileron controller
	if(positionControllerEnabled && landingState == LS_FLIGHT) {
		error = aileronSetpoint - estimatedAileronPos;
		vd = KX * error;
		if(vd > +POSITION_SPEED_MAX) vd = +POSITION_SPEED_MAX;
		if(vd < -POSITION_SPEED_MAX) vd = -POSITION_SPEED_MAX;
	} else { //velocity controller
		// TODO [HCH] add requested velocity setpoint
		vd = 0;
		error = - estimatedAileronVel;
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

	portENTER_CRITICAL();

	controllerAileronOutput =
		KV * (vd - estimatedAileronVel) + aileronIntegration + derivative2;
	if (controllerAileronOutput > CONTROLLER_AILERON_SATURATION) {
		controllerAileronOutput = CONTROLLER_AILERON_SATURATION;
	} else if (controllerAileronOutput < -CONTROLLER_AILERON_SATURATION) {
		controllerAileronOutput = -CONTROLLER_AILERON_SATURATION;
	}

	portEXIT_CRITICAL();

}

//~ ------------------------------------------------------------------------ ~//
//~ Altitude Estimator - interpolates the data fron PX4Flow sonar sensor     ~//
//~ ------------------------------------------------------------------------ ~//
=======
volatile float throttleSetpoint = 1;

/* -------------------------------------------------------------------- */
/*	Altitude Estimator - interpolates the data from PX4Flow				*/
/* -------------------------------------------------------------------- */
>>>>>>> TomasBaca
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
	
	portENTER_CRITICAL();

	//total output
	portENTER_CRITICAL();
	controllerThrottleOutput = (int16_t) (KV * (vd - estimatedThrottleVel) + throttleIntegration);
	if (controllerThrottleOutput > CONTROLLER_THROTTLE_SATURATION) {
		controllerThrottleOutput = CONTROLLER_THROTTLE_SATURATION;
	}
	if (controllerThrottleOutput < -CONTROLLER_THROTTLE_SATURATION) {
		controllerThrottleOutput = -CONTROLLER_THROTTLE_SATURATION;
	}
<<<<<<< HEAD

	portEXIT_CRITICAL();

=======
	portEXIT_CRITICAL();
>>>>>>> TomasBaca
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
