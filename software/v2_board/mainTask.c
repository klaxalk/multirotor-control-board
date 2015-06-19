#include "mainTask.h"
#include "controllers.h"
#include "communication.h"
#include "system.h"
#include "packets.h"
#include "commands.h"
#include <stdlib.h> // abs


//AUX channels switchers reacts just for changeS
unsigned char previous_AUX1 = 5;
unsigned char previous_AUX2 = 5;
unsigned char previous_AUX3 = 5;
unsigned char previous_AUX4 = 5;
unsigned char previous_AUX5 = 5;

void mainTask(void *p) {	
	while (1) {		
		
		if(RCchannel[AUX1]<PPM_IN_MIDDLE_LENGTH){
			if(previous_AUX1!=0){				
				previous_AUX1=0;
			}
		}else{
			if(previous_AUX1!=1){							
				previous_AUX1=1;
			}
		}
		
				
		if(RCchannel[AUX2]<(PPM_IN_MIDDLE_LENGTH)){
			if(previous_AUX2!=0){			
				previous_AUX2=0;
			}
		}else{
			if(previous_AUX2!=1){				
				previous_AUX2=1;
			}
		}
					
		// controllers on/off
		if (RCchannel[AUX3] < (PPM_IN_MIDDLE_LENGTH - 400)) {
			if (previous_AUX3 != 1) {			
				controllerSet(CONTROLLERS.OFF);						
				previous_AUX3 = 1;
			}
		} else if (RCchannel[AUX3] > (PPM_IN_MIDDLE_LENGTH + 400)) {
			if (previous_AUX3 != 2) {			
				TRAJ_POINT(0,0,2,0,1);
				trajMaxIndex=0;				
				previous_AUX3 = 2;
			}			
		} else {
			if (previous_AUX3 != 0) {			
				controllerSet(CONTROLLERS.MPC);	
				TRAJ_POINT(0,0,0,0,1);
				trajMaxIndex=0;	
				previous_AUX3 = 0;
			}
		}

		if(RCchannel[AUX4]<PPM_IN_MIDDLE_LENGTH){
			if(previous_AUX4!=0){
				previous_AUX4=0;
			}
			}else{
			if(previous_AUX4!=1){
				previous_AUX4=1;
			}
		}
	

		if(RCchannel[AUX5]<PPM_IN_MIDDLE_LENGTH){
			if(previous_AUX5!=0){
				//kopterPositionSlaveSet(ADDRESS.COORDINATOR,ADDRESS.UNKNOWN16,ADDRESS.COORDINATOR);					
				previous_AUX5=0;
			}
		}else{
			if(previous_AUX5!=1){
				//kopterPositionSlaveSet(ADDRESS.COORDINATOR,ADDRESS.UNKNOWN16,ADDRESS.K3);			
				previous_AUX5=1;
			}
		}
		mergeSignalsToOutput();
	}
}