#include "mainTask.h"
#include "controllers.h"
#include "communication.h"
#include "system.h"
#include "packets.h"
#include "commands.h"
#include <stdio.h> // sprintf
#include <stdlib.h> // abs


//AUX channels switchers reacts just for changeS
unsigned char previous_AUX1 = 5;
unsigned char previous_AUX2 = 5;
unsigned char previous_AUX3 = 5;
unsigned char previous_AUX4 = 5;
unsigned char previous_AUX5 = 5;

void mainTask(void *p) {	
	while (1) {					
		// controllers on/off
		portENTER_CRITICAL();
		if (       RCchannel[AUX3] < (PPM_IN_MIDDLE_LENGTH - 400)) {
			if (previous_AUX3 != 1) {			
				disableVelocityController();
				disablePositionController();						
				previous_AUX3 = 1;
			}
		} else if (RCchannel[AUX3] > (PPM_IN_MIDDLE_LENGTH + 400)) {
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
		portEXIT_CRITICAL();

		// landing on/off, trajectory on/off
		if (RCchannel[AUX4] < (PPM_IN_MIDDLE_LENGTH - 400)) {
			if(previous_AUX4!=0){				
				enableLanding();
				trajectoryEnabled = 0;		
				previous_AUX4 = 0;
			}
		} else if(RCchannel[AUX4] > (PPM_IN_MIDDLE_LENGTH + 400)) {
			if(previous_AUX4!=1){
				disableLanding();
				trajectoryEnabled = 1;					
				previous_AUX4=1;
			}
		}else{
			if(previous_AUX4!=2){
				disableLanding();
				trajectoryEnabled = 0;				
				previous_AUX4=2;
			}
		}

		//velocity null setpoints
		if(RCchannel[AUX2]<(PPM_IN_MIDDLE_LENGTH)){
			if(previous_AUX2!=0){	
						
			previous_AUX2=0;
			}
		}else{
			if(previous_AUX2!=1){	
				aileronDesiredVelocitySetpoint = DEFAULT_AILERON_VELOCITY_SETPOINT;			
				elevatorDesiredVelocitySetpoint = DEFAULT_ELEVATOR_VELOCITY_SETPOINT;					
			previous_AUX2=1;
			}
		}


		//gumstix enable
		if(RCchannel[AUX1]<PPM_IN_MIDDLE_LENGTH){
			if(previous_AUX1!=0){
	
				previous_AUX1=0;
			}
		}else{
			if(previous_AUX1!=1){

				previous_AUX1=1;
			}
		}
	
		//leading enable
		if(RCchannel[AUX5]<PPM_IN_MIDDLE_LENGTH){
			if(previous_AUX5!=0){
				adr64Setter2(leadKopter,ADDRESS.COORDINATOR);
				previous_AUX5=0;
			}
		}else{
			if(previous_AUX5!=1){
				adr64Setter2(leadKopter,ADDRESS.K1);
				previous_AUX5=1;
			}
		}
		mergeSignalsToOutput();
	}
}