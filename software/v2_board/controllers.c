/*
 * controllers.c
 *
 * Created: 11.9.2014 13:24:16
 *  Author: Tomas Baca
 */ 

#include "controllers.h"
#include "communication.h"

// controllers output variables
volatile int16_t velocityControllerElevatorOutput;
volatile int16_t velocityControllerAileronOutput;
volatile int16_t positionControllerElevatorOutput;
volatile int16_t positionControllerAileronOutput;

volatile int16_t controllerThrottleOutput;
volatile int16_t controllerRudderOutput;

//vars for estimators
volatile float estimatedElevatorPos = 0;
volatile float estimatedAileronPos  = 0;
volatile float estimatedThrottlePos = 0;

volatile float estimatedElevatorVel = 0;
volatile float estimatedAileronVel  = 0;
volatile float estimatedThrottleVel = 0;
volatile float estimatedElevatorVel_Prev = 0;
volatile float estimatedAileronVel_Prev  = 0;

volatile float estimatedElevatorAcc = 0;
volatile float estimatedAileronAcc = 0;

//vars for controllers
volatile float elevatorPositionIntegration = 0;
volatile float aileronPositionIntegration  = 0;
volatile float elevatorVelocityIntegration = 0;
volatile float aileronVelocityIntegration  = 0;
volatile float throttleIntegration = 0;

volatile float elevatorPositionSetpoint =  DEFAULT_ELEVATOR_POSITION_SETPOINT;
volatile float aileronPositionSetpoint  = DEFAULT_AILERON_POSITION_SETPOINT;
volatile float throttleSetpoint = DEFAULT_THROTTLE_SETPOINT;
volatile float elevatorVelocitySetpoint =  DEFAULT_ELEVATOR_VELOCITY_SETPOINT;
volatile float aileronVelocitySetpoint  = DEFAULT_AILERON_VELOCITY_SETPOINT;

volatile float elevatorDesiredPositionSetpoint = DEFAULT_ELEVATOR_POSITION_SETPOINT;
volatile float aileronDesiredPositionSetpoint  = DEFAULT_AILERON_POSITION_SETPOINT;
volatile float throttleDesiredSetpoint = DEFAULT_THROTTLE_SETPOINT;
volatile float elevatorDesiredVelocitySetpoint = DEFAULT_ELEVATOR_VELOCITY_SETPOINT;
volatile float aileronDesiredVelocitySetpoint  = DEFAULT_AILERON_VELOCITY_SETPOINT;

//auto-landing variables
volatile unsigned char landingRequest = 0;
volatile unsigned char landingState = LS_ON_GROUND;
volatile uint8_t landingCounter = 0;

// controller on/off
volatile unsigned char velocityControllerEnabled = 0;
volatile unsigned char positionControllerEnabled = 0;

//Gumstix on/off
volatile unsigned char gumstixEnabled = 0;

//auto-trajectory variables
volatile unsigned char trajectoryEnabled = 0;
volatile float trajTimer = 0;
volatile int8_t trajIndex = -1;
volatile int8_t trajMaxIndex = -1;
volatile trajectoryPoint_t trajectory[TRAJECTORY_LENGTH];

static uint8_t estimator_cycle = 0;
static float   estimatedThrottlePos_prev = 0;


void initTrajectory(){
	int8_t i=0;
	// (i, time (s), x (+ forward), y (+ leftward), z (altitude))
	for(i=0;i<TRAJECTORY_LENGTH;i++){
		TRAJ_POINT(i,i+1,elevatorPositionSetpoint,aileronPositionSetpoint,throttleSetpoint);
	}
	trajMaxIndex=-1;
}

void enableGumstix(){
	if (gumstixEnabled ==0 ){		
		elevatorPositionSetpoint = DEFAULT_ELEVATOR_POSITION_SETPOINT;
		aileronPositionSetpoint = DEFAULT_AILERON_POSITION_SETPOINT;
		elevatorDesiredPositionSetpoint = DEFAULT_ELEVATOR_POSITION_SETPOINT;
		aileronDesiredPositionSetpoint = DEFAULT_AILERON_POSITION_SETPOINT;	
		
		estimatedElevatorPos = elevatorPositionSetpoint;
		estimatedAileronPos  = aileronPositionSetpoint;
		
		aileronVelocitySetpoint = DEFAULT_AILERON_VELOCITY_SETPOINT;
		elevatorVelocitySetpoint = DEFAULT_ELEVATOR_VELOCITY_SETPOINT;
		aileronDesiredVelocitySetpoint = DEFAULT_AILERON_VELOCITY_SETPOINT;
		elevatorDesiredVelocitySetpoint = DEFAULT_ELEVATOR_VELOCITY_SETPOINT;			
	}
	gumstixEnabled = 1;
}

void disableGumstix(){	
	gumstixEnabled = 0;
}

void enableVelocityController() {
	if(velocityControllerEnabled==0 && positionControllerEnabled==0){
		throttleIntegration = 0;
		throttleSetpoint = DEFAULT_THROTTLE_SETPOINT;
		throttleDesiredSetpoint = DEFAULT_THROTTLE_SETPOINT;
	}
	if (velocityControllerEnabled == 0) {
		elevatorVelocityIntegration = 0;
		aileronVelocityIntegration = 0;		
						
		aileronVelocitySetpoint = DEFAULT_AILERON_VELOCITY_SETPOINT;		
		elevatorVelocitySetpoint = DEFAULT_ELEVATOR_VELOCITY_SETPOINT;
		aileronDesiredVelocitySetpoint = DEFAULT_AILERON_VELOCITY_SETPOINT;
		elevatorDesiredVelocitySetpoint = DEFAULT_ELEVATOR_VELOCITY_SETPOINT;
	}
	velocityControllerEnabled = 1;
}

void disableVelocityController() {
	velocityControllerEnabled = 0;
}

void enablePositionController() {
	if(velocityControllerEnabled==0 && positionControllerEnabled==0){
		throttleIntegration = 0;
		throttleSetpoint = DEFAULT_THROTTLE_SETPOINT;
		throttleDesiredSetpoint = DEFAULT_THROTTLE_SETPOINT;
	}	
	if (positionControllerEnabled == 0) {
		elevatorPositionIntegration = 0;
		aileronPositionIntegration = 0;
		
		elevatorPositionSetpoint = DEFAULT_ELEVATOR_POSITION_SETPOINT;
		aileronPositionSetpoint = DEFAULT_AILERON_POSITION_SETPOINT;
		elevatorDesiredPositionSetpoint = DEFAULT_ELEVATOR_POSITION_SETPOINT;
		aileronDesiredPositionSetpoint = DEFAULT_AILERON_POSITION_SETPOINT;
		
		estimatedElevatorPos = elevatorPositionSetpoint;
		estimatedAileronPos  = aileronPositionSetpoint;
	}
	positionControllerEnabled = 1;
}

void disablePositionController() {
	positionControllerEnabled = 0;
}

//~ ------------------------------------------------------------------------ ~//
//~ Setpoints - assign setpoint values                                       ~//
//~ ------------------------------------------------------------------------ ~//
void setpoints() {

	float dTime;
	float dValue;
	static float aileronIncrement;
	static float elevatorIncrement;
	static float throttleIncrement;


	//trajectory following
	if(trajectoryEnabled && positionControllerEnabled){

		if(trajIndex < trajMaxIndex){

			if(trajIndex < 0 || trajTimer >= trajectory[trajIndex].time) {
				trajIndex++;
				dTime  = (trajectory[trajIndex].time - trajTimer);
				dValue = (trajectory[trajIndex].elevatorPos - elevatorPositionSetpoint);
				elevatorIncrement = dValue / dTime * DT;
				dValue = (trajectory[trajIndex].aileronPos - aileronPositionSetpoint);
				aileronIncrement = dValue / dTime * DT;
				dValue = (trajectory[trajIndex].throttlePos - throttleSetpoint);
				throttleIncrement = dValue / dTime * DT;
			}

			trajTimer  += DT;
			elevatorPositionSetpoint += elevatorIncrement;
			aileronPositionSetpoint  += aileronIncrement;
			throttleSetpoint  += throttleIncrement;


		} //else End of Trajectory - do nothing

	//manual setpoints from RC transmitter
	}else{
	
		if(throttleDesiredSetpoint<THROTTLE_SP_LOW){throttleDesiredSetpoint=THROTTLE_SP_LOW;}
		if(throttleDesiredSetpoint>THROTTLE_SP_HIGH){throttleDesiredSetpoint=THROTTLE_SP_HIGH;}
			
		if(elevatorDesiredVelocitySetpoint<-SPEED_MAX){elevatorDesiredVelocitySetpoint=-SPEED_MAX;}
		if(elevatorDesiredVelocitySetpoint>SPEED_MAX){elevatorDesiredVelocitySetpoint=SPEED_MAX;}
			
		if(aileronDesiredVelocitySetpoint<-SPEED_MAX){aileronDesiredVelocitySetpoint=-SPEED_MAX;}
		if(aileronDesiredVelocitySetpoint>SPEED_MAX){aileronDesiredVelocitySetpoint=SPEED_MAX;}
				
			
		elevatorPositionSetpoint += (elevatorDesiredPositionSetpoint-elevatorPositionSetpoint) * (DT/SETPOINT_FILTER_CONST);
		aileronPositionSetpoint += (aileronDesiredPositionSetpoint-aileronPositionSetpoint) * (DT/SETPOINT_FILTER_CONST);
		throttleSetpoint += (throttleDesiredSetpoint-throttleSetpoint) * (DT/SETPOINT_FILTER_CONST);
		
		elevatorVelocitySetpoint += (elevatorDesiredVelocitySetpoint-elevatorVelocitySetpoint) * (DT/SETPOINT_FILTER_CONST);
		aileronVelocitySetpoint += (aileronDesiredVelocitySetpoint-aileronVelocitySetpoint) * (DT/SETPOINT_FILTER_CONST);		

		//reset trajectory vars
		trajIndex = -1;
		trajTimer = 0;
	}
}

//~ ------------------------------------------------------------------------ ~//
//~ Position Estimator - estimates Elevator and Aileron position and         ~//
//~                      velocity from PX4flow and Gumstix data              ~//
//~ ------------------------------------------------------------------------ ~//
void positionEstimator() {
	//help variable for acceleration
	float acc_new;
	
	//gustix valid delay - filters faulty values
	static uint8_t gumstix_counter = 0;
	uint8_t gumstix_delay = 7;

	if(validGumstix == 1) {
		if(gumstix_counter < gumstix_delay) gumstix_counter++;
	} else {
		gumstix_counter = 0;
	}

	//elevator velocity
	estimatedElevatorVel += (elevatorSpeed-estimatedElevatorVel) * (DT/PX4FLOW_FILTER_CONST);
	
	//elevator acceleration
	acc_new = (estimatedElevatorVel - estimatedElevatorVel_Prev) / DT;
	estimatedElevatorVel_Prev = estimatedElevatorVel;
	estimatedElevatorAcc += (acc_new - estimatedElevatorAcc) * (DT/PX4FLOW_FILTER_CONST);

	//elevator position
	if(gumstix_counter == gumstix_delay) {
		estimatedElevatorPos += (elevatorGumstix-estimatedElevatorPos) * (DT/GUMSTIX_FILTER_CONST);
	}else{
		estimatedElevatorPos += estimatedElevatorVel * DT;
	}

	//aileron velocity
	estimatedAileronVel += (aileronSpeed-estimatedAileronVel) * (DT/PX4FLOW_FILTER_CONST);
	
	//aileron acceleration
	acc_new = (estimatedAileronVel - estimatedAileronVel_Prev) / DT;
	estimatedAileronVel_Prev = estimatedAileronVel;
	estimatedAileronAcc += (acc_new - estimatedAileronAcc) * (DT/PX4FLOW_FILTER_CONST);

	//aileron position
	if(gumstix_counter == gumstix_delay) {
		estimatedAileronPos += (aileronGumstix-estimatedAileronPos) * (DT/GUMSTIX_FILTER_CONST);
	}else{
            estimatedAileronPos += estimatedAileronVel * DT;
	}
}

void velocityController() {
	float error;
	float KI, KV, KA;

	//set controller constants
	KI = VELOCITY_KI;
	KV = VELOCITY_KV;
	KA = VELOCITY_KA;

	//velocity controller
	error = elevatorVelocitySetpoint - estimatedElevatorVel;

	elevatorVelocityIntegration += KI * error * DT;
	if (elevatorVelocityIntegration > CONTROLLER_ELEVATOR_SATURATION/4) {elevatorVelocityIntegration = CONTROLLER_ELEVATOR_SATURATION/4;} else
	if (elevatorVelocityIntegration < -CONTROLLER_ELEVATOR_SATURATION/4) {elevatorVelocityIntegration = -CONTROLLER_ELEVATOR_SATURATION/4;}

	velocityControllerElevatorOutput = (KV * error) + elevatorPositionIntegration - (KA * estimatedElevatorAcc);
	if (velocityControllerElevatorOutput > CONTROLLER_ELEVATOR_SATURATION) {velocityControllerElevatorOutput = CONTROLLER_ELEVATOR_SATURATION;} else 
	if (velocityControllerElevatorOutput < -CONTROLLER_ELEVATOR_SATURATION) {velocityControllerElevatorOutput = -CONTROLLER_ELEVATOR_SATURATION;}


	//aileron controller
	error = aileronVelocitySetpoint - estimatedAileronVel;	

	aileronVelocityIntegration += KI * error * DT;
	if (aileronVelocityIntegration > CONTROLLER_AILERON_SATURATION/4) {aileronVelocityIntegration = CONTROLLER_AILERON_SATURATION/4;} else 
	if (aileronVelocityIntegration < -CONTROLLER_AILERON_SATURATION/4) {aileronVelocityIntegration = -CONTROLLER_AILERON_SATURATION/4;}

	velocityControllerAileronOutput = (KV * error) + aileronPositionIntegration - (KA * estimatedAileronAcc);
	if (velocityControllerAileronOutput > CONTROLLER_AILERON_SATURATION) {velocityControllerAileronOutput = CONTROLLER_AILERON_SATURATION;} else 
	if (velocityControllerAileronOutput < -CONTROLLER_AILERON_SATURATION) {velocityControllerAileronOutput = -CONTROLLER_AILERON_SATURATION;}	
}

void positionController() {
	float error;
	float vd; //desired velocity

	float KX, KI, KP, KV, KA;

	//set controller constants
	if(positionControllerEnabled && landingState == LS_FLIGHT) {
		KI = POSITION_KI;
		KP = POSITION_KP;
		KV = POSITION_KV;
		KA = POSITION_KA;
		KX = KP / KV;


	} else { //velocity controller
		KI = VELOCITY_KI;
		KV = VELOCITY_KV;
		KA = VELOCITY_KA;
		KX = 0;
	}

	//elevator controller
	if(positionControllerEnabled && landingState == LS_FLIGHT) {
		error = elevatorPositionSetpoint - estimatedElevatorPos;
		vd = KX * error;
		if(vd > +SPEED_MAX) vd = +SPEED_MAX;
		if(vd < -SPEED_MAX) vd = -SPEED_MAX;
	} else { //velocity controller
		vd = 0;
		error = - estimatedElevatorVel;
	}

	elevatorPositionIntegration += KI * error * DT;
	if (elevatorPositionIntegration > CONTROLLER_ELEVATOR_SATURATION/4) {elevatorPositionIntegration = CONTROLLER_ELEVATOR_SATURATION/4;} else 
	if (elevatorPositionIntegration < -CONTROLLER_ELEVATOR_SATURATION/4) {elevatorPositionIntegration = -CONTROLLER_ELEVATOR_SATURATION/4;}

	positionControllerElevatorOutput = KV * (vd - estimatedElevatorVel) + elevatorPositionIntegration - (KA * estimatedElevatorAcc);
	if (positionControllerElevatorOutput > CONTROLLER_ELEVATOR_SATURATION) {positionControllerElevatorOutput = CONTROLLER_ELEVATOR_SATURATION;} else 
	if (positionControllerElevatorOutput < -CONTROLLER_ELEVATOR_SATURATION) {positionControllerElevatorOutput = -CONTROLLER_ELEVATOR_SATURATION;}


	//aileron controller
	if(positionControllerEnabled && landingState == LS_FLIGHT) {
		error = aileronPositionSetpoint - estimatedAileronPos;
		vd = KX * error;
		if(vd > +SPEED_MAX) vd = +SPEED_MAX;
		if(vd < -SPEED_MAX) vd = -SPEED_MAX;
	} else { //velocity controller
		vd = 0;
		error = - estimatedAileronVel;
	}

	aileronPositionIntegration += KI * error * DT;
	if (aileronPositionIntegration > CONTROLLER_AILERON_SATURATION/4) {aileronPositionIntegration = CONTROLLER_AILERON_SATURATION/4;} else 
	if (aileronPositionIntegration < -CONTROLLER_AILERON_SATURATION/4) {aileronPositionIntegration = -CONTROLLER_AILERON_SATURATION/4;} 

	positionControllerAileronOutput = KV * (vd - estimatedAileronVel) + aileronPositionIntegration - (KA * estimatedAileronAcc);
	if (positionControllerAileronOutput > CONTROLLER_AILERON_SATURATION) {positionControllerAileronOutput = CONTROLLER_AILERON_SATURATION;} else 
	if (positionControllerAileronOutput < -CONTROLLER_AILERON_SATURATION) {positionControllerAileronOutput = -CONTROLLER_AILERON_SATURATION;}

}

//~ ------------------------------------------------------------------------ ~//
//~ Altitude Estimator - interpolates the data from PX4Flow sonar sensor     ~//
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

