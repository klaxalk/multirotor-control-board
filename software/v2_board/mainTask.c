/*
 * mainTask.c
 *
 * Created: 11.9.2014 11:05:32
 *  Author: Tomas Baca
 */ 

#include "mainTask.h"
#include "system.h"
#include "controllers.h"
#include "communication.h"
#include <stdio.h> // sprintf
#include <stdlib.h> // abs

// constants from RC transmitter
volatile float constant1 = 0;
volatile float constant2 = 0;
volatile float constant5 = 0;

// timestamp for debug and logging
volatile double timeStamp = 0;
volatile uint16_t main_cycle = 0;

// for on-off by AUX channels
unsigned char previous_AUX3 = 5;
unsigned char previous_AUX4 = 5;

// controller on/off
volatile unsigned char controllerEnabled = 0;
volatile unsigned char positionControllerEnabled = 0;

#if TRAJECTORY_FOLLOWING == ENABLED

void writeTrajectory1(){
	int8_t i=0;

	// (i, time (s), x (+ forward), y (+ leftward), z (altitude))
	
	for(i=0;i<TRAJECTORY_LENGTH;i++){
		TRAJ_POINT(i,i+1,elevatorSetpoint,aileronSetpoint,throttleSetpoint);
	}
	trajMaxIndex=-1;
}

#endif //TRAJECTORY_FOLLOWING == ENABLED

void mainTask(void *p) {
	
#if TRAJECTORY_FOLLOWING == ENABLED
	writeTrajectory1();
#endif
	
	while (1) {		
		main_cycle++;		
		// controller on/off
		if (abs(RCchannel[AUX3] - PPM_IN_MIDDLE_LENGTH) < 200) {
			if (previous_AUX3 != 1) {
				enableController();
				disablePositionController();
				previous_AUX3 = 1;
			}
		} else if (RCchannel[AUX3] > (PPM_IN_MIDDLE_LENGTH + 200)) {
			if (previous_AUX3 != 2) {
				enablePositionController();
				previous_AUX3 = 2;
			}			
		} else {
			if (previous_AUX3 != 0) {
				disableController();
				disablePositionController();
				previous_AUX3 = 0;
			}
		}
		
#if PX4FLOW_DATA_RECEIVE == ENABLED
	// landing on/off, trajectory on/off
	if (RCchannel[AUX4] < (PPM_IN_MIDDLE_LENGTH - 200)) {
		if(previous_AUX4!=0){
			landingRequest = 1;
			trajectoryEnabled = 0;
			previous_AUX4 = 0;
		}
	} else if(RCchannel[AUX4] > (PPM_IN_MIDDLE_LENGTH + 200)) {
		if(previous_AUX4!=1){
			landingRequest = 0;
			trajectoryEnabled = 1;
			previous_AUX4=1;
		}
	}else{
		if(previous_AUX4!=2){
			landingRequest = 0;
			trajectoryEnabled = 0;
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
		
		mergeSignalsToOutput();
	}
}