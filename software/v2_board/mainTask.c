#include "mainTask.h"
#include "controllers.h"
#include "communication.h"
#include "system.h"
#include "packets.h"
#include "commands.h"
#include <stdio.h> // sprintf
#include <stdlib.h> // abs

// constants from RC transmitter
volatile float constant1 = 0;
volatile float constant5 = 0;

//AUX channels switchers reacts just for changeS
unsigned char previous_AUX1 = 5;
unsigned char previous_AUX3 = 5;
unsigned char previous_AUX4 = 5;
unsigned char previous_AUX5 = 5;

void mainTask(void *p) {		
	while (1) {					
		// controllers on/off
		if (abs(RCchannel[AUX3] - PPM_IN_MIDDLE_LENGTH) < 200) {
			if (previous_AUX3 != 1) {
				#if (COOR_REPORTS==ENABLE)
				sendXBeeMessage(ADDRESS.COORDINATOR,ADDRESS.UNKNOWN16,"MANUAL VEL CONT",0x00);
				#endif
				enableVelocityController();
				disablePositionController();								
				previous_AUX3 = 1;
			}
		} else if (RCchannel[AUX3] > (PPM_IN_MIDDLE_LENGTH + 200)) {
			if (previous_AUX3 != 2) {
				#if (COOR_REPORTS==ENABLE)
				sendXBeeMessage(ADDRESS.COORDINATOR,ADDRESS.UNKNOWN16,"MANUAL POS CONT",0x00);
				#endif				
				enablePositionController();
				disableVelocityController();
				previous_AUX3 = 2;
			}			
		} else {
			if (previous_AUX3 != 0) {
				#if (COOR_REPORTS==ENABLE)
				sendXBeeMessage(ADDRESS.COORDINATOR,ADDRESS.UNKNOWN16,"MANUAL OFF CONT",0x00);
				#endif				
				disableVelocityController();
				disablePositionController();
				previous_AUX3 = 0;
			}
		}

		// landing on/off, trajectory on/off
		if (RCchannel[AUX4] < (PPM_IN_MIDDLE_LENGTH - 200)) {
			if(previous_AUX4!=0){
				#if (COOR_REPORTS==ENABLE)
				sendXBeeMessage(ADDRESS.COORDINATOR,ADDRESS.UNKNOWN16,"MANUAL LANDING ON",0x00);
				#endif				
				enableLanding();
				disableTrajectoryFollow();			
				previous_AUX4 = 0;
			}
		} else if(RCchannel[AUX4] > (PPM_IN_MIDDLE_LENGTH + 200)) {
			if(previous_AUX4!=1){
				#if (COOR_REPORTS==ENABLE)
				sendXBeeMessage(ADDRESS.COORDINATOR,ADDRESS.UNKNOWN16,"MANUAL TRAJ FOLLOW",0x00);
				#endif				
				disableLanding();
				enableTrajectoryFollow();
				previous_AUX4=1;
			}
		}else{
			if(previous_AUX4!=2){
				#if (COOR_REPORTS==ENABLE)
				sendXBeeMessage(ADDRESS.COORDINATOR,ADDRESS.UNKNOWN16,"MANUAL LAND TRAJ OFF",0x00);
				#endif				
				disableLanding();
				disableTrajectoryFollow();
				previous_AUX4=2;
			}
		}

		//velocity null setpoints
		if(RCchannel[AUX2]<(PPM_IN_MIDDLE_LENGTH)){
		}else{
			aileronDesiredVelocitySetpoint = DEFAULT_AILERON_VELOCITY_SETPOINT;			
			elevatorDesiredVelocitySetpoint = DEFAULT_ELEVATOR_VELOCITY_SETPOINT;	
		}


		//gumstix enable
		if(RCchannel[AUX1]<PPM_IN_MIDDLE_LENGTH){
			if(previous_AUX1!=0){
				#if (COOR_REPORTS==ENABLE)
				sendXBeeMessage(ADDRESS.COORDINATOR,ADDRESS.UNKNOWN16,"MANUAL GUMSTIX DISABLE",0x00);
				#endif				
				disableGumstix();
				previous_AUX1=0;
			}
		}else{
			if(previous_AUX1!=1){
				#if (COOR_REPORTS==ENABLE)
				sendXBeeMessage(ADDRESS.COORDINATOR,ADDRESS.UNKNOWN16,"MANUAL GUMSTIX ENABLE",0x00);
				#endif				
				enableGumstix();
				previous_AUX1=1;
			}
		}
	
		//leading enable
		if(RCchannel[AUX5]<PPM_IN_MIDDLE_LENGTH){
			if(previous_AUX5!=0){
				//leadKopter=ADDRESS.COORDINATOR;
				previous_AUX5=0;
			}
		}else{
			if(previous_AUX5!=1){
				//leadKopter=ADDRESS.K1;
				previous_AUX5=1;
			}
		}
		mergeSignalsToOutput();
	}
}