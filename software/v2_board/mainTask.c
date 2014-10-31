#include "mainTask.h"
#include "controllers.h"
#include "system.h"
#include <stdio.h> // sprintf
#include <stdlib.h> // abs

// constants from RC transmitter
volatile float constant1 = 0;
volatile float constant5 = 0;

//AUX channels switchers reacts just for changeS
unsigned char previous_AUX2 = 5;
unsigned char previous_AUX3 = 5;
unsigned char previous_AUX4 = 5;

void mainTask(void *p) {		
	while (1) {					
		// controllers on/off
		if (abs(RCchannel[AUX3] - PPM_IN_MIDDLE_LENGTH) < 200) {
			if (previous_AUX3 != 1) {
				disableVelocityController();
				disablePositionController();
				previous_AUX3 = 1;
			}
		} else if (RCchannel[AUX3] > (PPM_IN_MIDDLE_LENGTH + 200)) {
			if (previous_AUX3 != 2) {
				enablePositionController();
				disableVelocityController();
				previous_AUX3 = 2;
			}			
		} else {
			if (previous_AUX3 != 0) {
				enableVelocityController();
				disablePositionController();
				previous_AUX3 = 0;
			}
		}

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

	if(RCchannel[AUX1]<PPM_IN_MIDDLE_LENGTH){
		constant1=0;
	}else{
		constant1=1;
		aileronDesiredVelocitySetpoint = DEFAULT_AILERON_VELOCITY_SETPOINT;			
		elevatorDesiredVelocitySetpoint = DEFAULT_ELEVATOR_VELOCITY_SETPOINT;		
	}

	if(RCchannel[AUX2]<PPM_IN_MIDDLE_LENGTH){
		if(previous_AUX2!=0){
			disableGumstix();
			previous_AUX2=0;
		}
	}else{
		if(previous_AUX2!=1){
			enableGumstix();
			previous_AUX2=1;
		}
	}


		constant5 = ((float)(RCchannel[AUX5] - PPM_IN_MIN_LENGTH))/(PPM_IN_MAX_LENGTH - PPM_IN_MIN_LENGTH);
		if(constant5 > 1) constant5 = 1;
		if(constant5 < 0) constant5 = 0;	
		mergeSignalsToOutput();
	}
}