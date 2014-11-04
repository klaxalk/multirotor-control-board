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

// for controller on-off
int8_t previous_RADIO_CONTROLLER_ON_OFF_SWITCH = 0;

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

// TODO: support for different RC transmitters

#define RADIO_AURORA9
// #define RADIO_HCH

#define NOT_AVAILABLE (-1)

#ifdef RADIO_AURORA9
	#define RADIO_CONTROLLER_ON_OFF_SWITCH AUX3
	#define RADIO_LANDING_SWITCH AUX4
	#define RADIO_CONSTANT1 AUX1
	#define RADIO_CONSTANT2 AUX2
	#define RADIO_CONSTANT5 AUX5

	#ifdef RADIO_HCH
		#error Only one radio transmitter may be defined
	#endif
#endif // RADIO_AURORA9

#ifdef RADIO_HCH
	#define RADIO_CONTROLLER_ON_OFF_SWITCH AUX3 // TODO
	#define RADIO_LANDING_SWITCH AUX4 // TODO
	#define RADIO_CONSTANT1 NOT_AVAILABLE
	#define RADIO_CONSTANT2 NOT_AVAILABLE
	#define RADIO_CONSTANT5 NOT_AVAILABLE

	#ifdef RADIO_AURORA9
		#error Only one radio transmitter may be defined
	#endif
#endif // RADIO_HCH

#if ! defined(RADIO_AURORA9) && ! defined(RADIO_HCH)
	#error No radio transmitter defined
#endif

void mainTask(void *p)
{

#if TRAJECTORY_FOLLOWING == ENABLED
	writeTrajectory1();
#endif

	while (1) {

		main_cycle++;

		// controller on/off
		if (abs(RCchannel[RADIO_CONTROLLER_ON_OFF_SWITCH] - PPM_IN_MIDDLE_LENGTH) < 200) {
			if (previous_RADIO_CONTROLLER_ON_OFF_SWITCH == 0) {
				enableController();
			}
			disablePositionController();
			previous_RADIO_CONTROLLER_ON_OFF_SWITCH = 1;
		} else if (RCchannel[RADIO_CONTROLLER_ON_OFF_SWITCH] > (PPM_IN_MIDDLE_LENGTH + 200)) {
			if (previous_RADIO_CONTROLLER_ON_OFF_SWITCH == 1) {
				enablePositionController();
			}
			previous_RADIO_CONTROLLER_ON_OFF_SWITCH = 2;
		} else {
			disableController();
			disablePositionController();
			previous_RADIO_CONTROLLER_ON_OFF_SWITCH = 0;
		}

#if PX4FLOW_DATA_RECEIVE == ENABLED

		// landing on/off, trajectory on/off
		if (RCchannel[RADIO_LANDING_SWITCH] < (PPM_IN_MIDDLE_LENGTH - 200)) {
			landingRequest = 1;
			trajectoryEnabled = 0;
		} else if(RCchannel[RADIO_LANDING_SWITCH] > (PPM_IN_MIDDLE_LENGTH + 200)) {
			landingRequest = 0;
			trajectoryEnabled = 1;
		}else{
			landingRequest = 0;
			trajectoryEnabled = 0;
		}

#endif // PX4FLOW_DATA_RECEIVE == ENABLED

		// load the constant values from the RC
		// <0; 1>
#if RADIO_CONSTANT1 != NOT_AVAILABLE
		constant1 = ((float)(RCchannel[RADIO_CONSTANT1] - PPM_IN_MIN_LENGTH))/(PPM_IN_MAX_LENGTH - PPM_IN_MIN_LENGTH);
		if(constant1 > 1) constant1 = 1;
		if(constant1 < 0) constant1 = 0;
#endif

#if RADIO_CONSTANT2 != NOT_AVAILABLE
		constant2 = ((float)(RCchannel[RADIO_CONSTANT2] - PPM_IN_MIN_LENGTH))/(PPM_IN_MAX_LENGTH - PPM_IN_MIN_LENGTH);
		if(constant2 > 1) constant2 = 1;
		if(constant2 < 0) constant2 = 0;
#endif

#if RADIO_CONSTANT5 != NOT_AVAILABLE
		constant5 = ((float)(RCchannel[RADIO_CONSTANT5] - PPM_IN_MIN_LENGTH))/(PPM_IN_MAX_LENGTH - PPM_IN_MIN_LENGTH);
		if(constant5 > 1) constant5 = 1;
		if(constant5 < 0) constant5 = 0;
#endif

		mergeSignalsToOutput();
	}
}
