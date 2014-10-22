/*
 * mainTask.c
 *
 * Created: 11.9.2014 11:05:32
 *  Author: Tomas Baca
 */ 

#include "mainTask.h"
#include "system.h"
#include "controllers.h"
#include <stdio.h> // sprintf
#include <stdlib.h> // abs

extern volatile uint16_t RCchannel[9];

// flag to run the controllers
volatile int8_t controllersFlag = 0;

// constants from RC transmitter
volatile float constant1 = 0;
volatile float constant2 = 0;
volatile float constant5 = 0;

// timestamp for debug and logging
volatile double timeStamp = 0;
volatile uint16_t main_cycle = 0;

// for on-off by AUX channels
unsigned char previous_AUX3 = 0;
unsigned char previous_AUX4 = 2;


#if PX4FLOW_DATA_RECEIVE == ENABLED

//~ --------------------------------------------------------------------
//~ Variables used with the px4flow sensor
//~ --------------------------------------------------------------------

//vars for estimators
volatile float estimatedElevatorPos = 0;
volatile float estimatedAileronPos  = 0;
volatile float estimatedThrottlePos = 0;
volatile float estimatedElevatorVel = 0;
volatile float estimatedAileronVel  = 0;
volatile float estimatedThrottleVel = 0;

//vars for controllers
volatile float elevatorIntegration = 0;
volatile float aileronIntegration  = 0;
volatile float throttleIntegration = 0;
volatile float elevatorSetpoint = (ELEVATOR_SP_LOW + ELEVATOR_SP_HIGH)/2;
volatile float aileronSetpoint  = (AILERON_SP_LOW  + AILERON_SP_HIGH )/2;
volatile float throttleSetpoint = 1;

volatile float elevatorDesiredSetpoint;
volatile float aileronDesiredSetpoint;
volatile float throttleDesiredSetpoint;

//auto-landing variables
volatile unsigned char landingRequest = 0;
volatile unsigned char landingState = LS_ON_GROUND;
volatile uint8_t landingCounter = 0;

//auto-trajectory variables
volatile unsigned char trajMaxIndex=0;
volatile unsigned char trajectoryEnabled = 0;
volatile float trajTimer = 0;
volatile unsigned char trajIndex = 0;
volatile trajectoryPoint_t trajectory[TRAJECTORY_LENGTH];

#endif // PX4FLOW_DATA_RECEIVE == ENABLED

#if GUMSTIX_DATA_RECEIVE == ENABLED

/* -------------------------------------------------------------------- */
/*	Variables for gumstix												*/
/* -------------------------------------------------------------------- */
volatile unsigned char gumstixParseCharState = 0;
volatile unsigned char gumstixParseCharByte = 0;
volatile int16_t gumstixParseTempInt;
volatile int16_t xPosGumstixNew = 0;
volatile int16_t yPosGumstixNew = 0;
volatile int16_t zPosGumstixNew = 0;
volatile float elevatorGumstix = 0;
volatile float aileronGumstix = 0;
volatile float throttleGumstix = 0;
volatile int8_t validGumstix = 0;
volatile int8_t gumstixDataFlag = 0;
volatile unsigned char gumstixParseCharCrc = 0;

#endif



void writeTrajectory1(){
	int i=0;
	for(i=0;i<TRAJECTORY_LENGTH;i++){
		trajectory[i].time=0;
		trajectory[i].throttlePos=throttleSetpoint;
		trajectory[i].aileronPos=aileronSetpoint;
		trajectory[i].elevatorPos=elevatorSetpoint;
	}	
	// TRAJ_POINT(i, time (s), x (+ forward), y (+ leftward), z (altitude))
}

void enableTrajectory(){
	if(trajectoryEnabled==0){
		trajIndex=0;
		trajTimer=0;
		trajectoryEnabled=1;
	}	
}

void disableTrajectory(){
	if(trajectoryEnabled==1){		
		elevatorDesiredSetpoint=elevatorSetpoint;
		aileronDesiredSetpoint=aileronSetpoint;
		throttleDesiredSetpoint=throttleSetpoint;
		trajectoryEnabled=0;	
	}
}

void mainTask(void *p) {	
		
//INIT desired setpoints	
elevatorDesiredSetpoint=elevatorSetpoint;
aileronDesiredSetpoint=aileronSetpoint;
throttleDesiredSetpoint=throttleSetpoint;
	
writeTrajectory1();
	
while (1) {		
		main_cycle++;
		
		// controller on/off
		if (abs(RCchannel[AUX3] - PPM_IN_MIDDLE_LENGTH) < 200) {
			if (previous_AUX3!=1){
				disablePositionController();
				enableController();				
				previous_AUX3 = 1;
			}			
		} else if (RCchannel[AUX3] > (PPM_IN_MIDDLE_LENGTH + 200)) {
			if(previous_AUX3 != 2){			
				enableController();	
				enablePositionController();
				previous_AUX3 = 2;		
			}			
		} else {
			if(previous_AUX3 != 0){
				disableController();
				disablePositionController();			
				previous_AUX3 = 0;
			}
		}
		
#if PX4FLOW_DATA_RECEIVE == ENABLED
	// landing on/off, trajectory on/off
	if (RCchannel[AUX4] < (PPM_IN_MIDDLE_LENGTH - 200)) {
			if(!previous_AUX4==0){
				landingRequest = 1;
				disableTrajectory();
				previous_AUX4 = 0;
			}
		} else if(RCchannel[AUX4] > (PPM_IN_MIDDLE_LENGTH + 200)) {
			if(!previous_AUX4==1){
				landingRequest = 0;
				enableTrajectory();
				previous_AUX4=1;
			}
		}else{
			if(!previous_AUX4==2){
				landingRequest = 0;
				disableTrajectory();
				previous_AUX4=2;
			}
	}
	
#endif // PX4FLOW_DATA_RECEIVE == ENABLED

		// load the constant values from the RC
		// <0; 1>
		constant1 = ((float)(RCchannel[AUX1] - PPM_IN_MIN_LENGTH))/(PPM_IN_MAX_LENGTH - PPM_IN_MIN_LENGTH);
		if(constant1 > 1) constant1 = 1;
		if(constant1 < 0) constant1 = 0;

		constant2 = ((float)(RCchannel[AUX2] - PPM_IN_MIN_LENGTH))/(PPM_IN_MAX_LENGTH - PPM_IN_MIN_LENGTH);
		if(constant2 > 1) constant2 = 1;
		if(constant2 < 0) constant2 = 0;

		constant5 = ((float)(RCchannel[AUX5] - PPM_IN_MIN_LENGTH))/(PPM_IN_MAX_LENGTH - PPM_IN_MIN_LENGTH);
		if(constant5 > 1) constant5 = 1;
		if(constant5 < 0) constant5 = 0;
		
#if GUMSTIX_DATA_RECEIVE == ENABLED

	//receive gumstix data
	if (gumstixDataFlag == 1) {

		if (validGumstix == 1) {

			//Gumstix returns position of the blob relative to camera
			//in milimeters, we want position of the drone relative
			//to the blob in meters.
			// +elevator = front
			// +aileron  = left
			// +throttle = up

			//saturation
			if(xPosGumstixNew > 2*POSITION_MAXIMUM) xPosGumstixNew = 2*POSITION_MAXIMUM;
			if(xPosGumstixNew < 0) xPosGumstixNew = 0; //distance from the blob (positive)

			if(yPosGumstixNew > +POSITION_MAXIMUM) yPosGumstixNew = +POSITION_MAXIMUM;
			if(yPosGumstixNew < -POSITION_MAXIMUM) yPosGumstixNew = -POSITION_MAXIMUM;

			if(zPosGumstixNew > +POSITION_MAXIMUM) zPosGumstixNew = +POSITION_MAXIMUM;
			if(zPosGumstixNew < -POSITION_MAXIMUM) zPosGumstixNew = -POSITION_MAXIMUM;

			#if GUMSTIX_CAMERA_POINTING == FORWARD //camera led on up side

			//~ Camera pointing forward and being PORTRAIT oriented
			//~ elevatorGumstix = - (float)xPosGumstixNew / 1000;
			//~ aileronGumstix  = - (float)zPosGumstixNew / 1000;
			//~ throttleGumstix = + (float)yPosGumstixNew / 1000;

			//~ Camera pointing forward and being LANDSCAPE oriented
			elevatorGumstix = - (float) xPosGumstixNew / 1000;
			aileronGumstix  = - (float) yPosGumstixNew / 1000;
			throttleGumstix = - (float) zPosGumstixNew / 1000;

			#elif GUMSTIX_CAMERA_POINTING == DOWNWARD //camera led on front side

			elevatorGumstix = + (float) yPosGumstixNew / 1000;
			aileronGumstix  = - (float) zPosGumstixNew / 1000;
			throttleGumstix = + (float) xPosGumstixNew / 1000;

			#endif

		}

		gumstixDataFlag = 0;
	}

#endif
		
		mergeSignalsToOutput();
	}
}