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

// constants = AUX channels from the RC transmitter
volatile float constant1 = 0;
volatile float constant2 = 0;
volatile float constant5 = 0;

// timestamp for debug and logging
volatile double timeStamp = 0;
volatile uint16_t main_cycle = 0;

// for on-off by AUX3 channel
int8_t previous_AUX3 = 0;

#if TRAJECTORY_FOLLOWING == ENABLED

void writeTrajectory1(){

	//TRAJ_POINT(0,  0, -1500,     0);

	// (i, time (s), x (+ forward), y (+ leftward), z (altitude))

	//~ Preplanovani LEADER Experiment Telocvicna
	//~  Aileron: Pùvodní*2
	TRAJ_POINT(0,  3,  -900,  0, 1000);
	TRAJ_POINT(1,  6,  -300,  0, 1000);
	TRAJ_POINT(2,  9,  +300,  0, 1000);
	TRAJ_POINT(3,  12,  +900,  0, 1000);
	TRAJ_POINT(4,  15,  +1500,  0, 1000);
	TRAJ_POINT(5,  18,  +2100,  0, 1000);
	TRAJ_POINT(6,  21,  +2700,  0, 1000);
	TRAJ_POINT(7,  24,  +3300,  0, 1000);
	TRAJ_POINT(8,  27,  +3900,  0, 1000);
	TRAJ_POINT(9,  30,  -1500,  0, 1000);
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
			if (previous_AUX3 == 0) {
				enableController();
			}
			disablePositionController();
			previous_AUX3 = 1;
			} else if (RCchannel[AUX3] > (PPM_IN_MIDDLE_LENGTH + 200)) {
			if (previous_AUX3 == 1) {
				enablePositionController();
			}
			previous_AUX3 = 2;
			} else {
			disableController();
			disablePositionController();
			previous_AUX3 = 0;
		}
		
#if PX4FLOW_DATA_RECEIVE == ENABLED

	// landing on/off, trajectory on/off
	if (RCchannel[AUX4] < (PPM_IN_MIDDLE_LENGTH - 200)) {
		landingRequest = 1;
		trajectoryEnabled = 0;
		} else if(RCchannel[AUX4] > (PPM_IN_MIDDLE_LENGTH + 200)) {
		landingRequest = 0;
		trajectoryEnabled = 1;
		}else{
		landingRequest = 0;
		trajectoryEnabled = 0;
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