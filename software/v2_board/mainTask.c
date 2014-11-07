/*
 * mainTask.c
 *
 * Created: 11.9.2014 11:05:32
 *  Author: Tomas Baca
 */ 

#ifdef WITH_PRINTF
#include <stdio.h> // sprintf
#endif
#include <stdlib.h> // abs

#include "mainTask.h"
#include "system.h"
#include "controllers.h"
#include "communication.h"
#include "debugcomm.h"
#if PC_COMMUNICATION == ENABLED
#include "pc_communication.h"
#endif

// constants = AUX channels from the RC transmitter
volatile float constant1 = 0;
volatile float constant2 = 0;
volatile float constant5 = 0;

// timestamp for debug and logging
volatile double timeStamp = 0;
volatile uint16_t main_cycle = 0;
volatile uint8_t dbg1 = 0;

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

#define RC_SWITCH_COUNT 5
#define PPM_IN_1_3 (PPM_IN_MIN_LENGTH + 0.365 * (PPM_IN_MAX_LENGTH - PPM_IN_MIN_LENGTH))
#define PPM_IN_2_3 (PPM_IN_MIN_LENGTH + 0.750 * (PPM_IN_MAX_LENGTH - PPM_IN_MIN_LENGTH))
#define RC_SWITCH_CHANNELS { AUX1, AUX2, AUX3, AUX4, AUX5 }
#define RC_SWITCH_THRESHOLDS_LOW { PPM_IN_MIDDLE_LENGTH, PPM_IN_MIDDLE_LENGTH, PPM_IN_1_3, PPM_IN_1_3, PPM_IN_MIDDLE_LENGTH }
#define RC_SWITCH_THRESHOLDS_HIGH { PPM_IN_TRESHOLD, PPM_IN_TRESHOLD, PPM_IN_2_3, PPM_IN_2_3, PPM_IN_TRESHOLD }

#define RC_SWITCH_STATE_UNDEFINED 0xFF

#define PPM_IN_DEADZONE 200

#define RADIO_CONTROLLER_ON_OFF_SWITCH 2
#define RADIO_LANDING_SWITCH 3

static const uint8_t rcSwitchChannel[RC_SWITCH_COUNT] = RC_SWITCH_CHANNELS;
static const uint16_t rcSwitchThreshold[RC_SWITCH_COUNT] = RC_SWITCH_THRESHOLDS_LOW;
static const uint16_t rcSwitchThreshold2[RC_SWITCH_COUNT] = RC_SWITCH_THRESHOLDS_HIGH;

#define RADIO_AURORA9
// #define RADIO_HCH

#define NOT_AVAILABLE (-1)

#ifdef RADIO_AURORA9
	#define RADIO_CONTROLLER_ON_OFF_CHANNEL AUX3
	#define RADIO_LANDING_CHANNEL AUX4
	#define RADIO_CONSTANT1 AUX1
	#define RADIO_CONSTANT2 AUX2
	#define RADIO_CONSTANT5 AUX5

	#ifdef RADIO_HCH
		#error Only one radio transmitter may be defined
	#endif
#endif // RADIO_AURORA9

#ifdef RADIO_HCH
	#define RADIO_CONTROLLER_ON_OFF_CHANNEL AUX3 // TODO
	#define RADIO_LANDING_CHANNEL AUX4 // TODO
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

#if PC_COMMUNICATION == ENABLED
static uint8_t in_trajectory_mode = 0;
#endif

void mainTask(void *p)
{

#if TRAJECTORY_FOLLOWING == ENABLED
	writeTrajectory1();
#endif

	uint8_t old_switches[RC_SWITCH_COUNT];
	for (uint8_t i = 0; i < RC_SWITCH_COUNT; i++) old_switches[i] = RC_SWITCH_STATE_UNDEFINED;

	while (1) {

		main_cycle++;

		// read switch values
		uint8_t switches[RC_SWITCH_COUNT];
		for (uint8_t i = 0; i < RC_SWITCH_COUNT; i++) {
			uint16_t inValue = RCchannel[rcSwitchChannel[i]];
			uint8_t inState;
			if (inValue <= rcSwitchThreshold[i] - PPM_IN_DEADZONE) {
				inState = 0;
			} else if (inValue > rcSwitchThreshold[i] + PPM_IN_DEADZONE) {
				if (inValue <= rcSwitchThreshold2[i] - PPM_IN_DEADZONE) {
					inState = 1;
				} else if (inValue > rcSwitchThreshold2[i] + PPM_IN_DEADZONE) {
					inState = 2;
				} else {
					inState = RC_SWITCH_STATE_UNDEFINED;
				}
			} else {
				inState = RC_SWITCH_STATE_UNDEFINED;
			}
			switches[i] = inState;
			if (inState != old_switches[i]) {
				if (inState != RC_SWITCH_STATE_UNDEFINED) { // TODO
					debugMessageF("switch %d %d->%d\r\n", i, old_switches[i], inState);
				}
			}
		}

		// controller on/off
		uint8_t swControllerState = switches[RADIO_CONTROLLER_ON_OFF_SWITCH];
		if (swControllerState != old_switches[RADIO_CONTROLLER_ON_OFF_SWITCH]) {
			if (swControllerState != RC_SWITCH_STATE_UNDEFINED) {
				if (swControllerState > 0) {
					if (old_switches[RADIO_CONTROLLER_ON_OFF_SWITCH] == 0) {
						// controller off->on
						enableController();
					}
					if (swControllerState == 2) {
						enablePositionController(); // TODO: option to disable position controller globally
					} else {
						if (old_switches[RADIO_CONTROLLER_ON_OFF_SWITCH] == 2) {
							disablePositionController();
						}
					}
				} else {
					if (old_switches[RADIO_CONTROLLER_ON_OFF_SWITCH] == 2) {
						disablePositionController();
					}
					disableController();
				}
			} else {
				// TODO: invalid input signal?
			}
		}

#if PX4FLOW_DATA_RECEIVE == ENABLED

		// landing on/off, trajectory on/off
		switch (switches[RADIO_LANDING_SWITCH]) {
		case 0:
			landingRequest = 1;
			trajectoryEnabled = 0;
			break;
		case 1:
		case 2:
			landingRequest = 0;
			trajectoryEnabled = 1;
		default:
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

#if PC_COMMUNICATION == ENABLED
#if SURFSTAB_MODE == ENABLED
		// position control switch (AUX4)
		if (switches[3] != old_switches[3]) {
			debugMessageF("pos-ctrl switch %d\r\n", switches[3]);
			switch (switches[3]) {
			case 0:
				// no position control
				pcStay();
				in_trajectory_mode = 0;
				break;
			case 1:
				// position control - stay
				pcStay();
				in_trajectory_mode = 0;
				break;
			case 2:
				// position control - trajectory
				pcFollowTrajectory();
				in_trajectory_mode = 1;
				break;
			default:
				// input error
				;
			}
		}

		// reset switch (AUX2)
		if ((switches[1] == 1) && (old_switches[1] == 0)) {
			debugMessageF("reset switch\r\n");
			// reset condition
			if (in_trajectory_mode) {
				pcGotoHome();
			} else {
				pcResetAll();
			}
		}

		// switch F / AUX1
		if ((switches[0] == 1) && (old_switches[0] == 0)) {
			debugMessageF("F switch\r\n");
			pcNewTrajectoryPoint();
			// F switch on
		}
#endif
#endif

		// TODO: detekce vstupniho signalu a generovani fail-safe vystupu

		mergeSignalsToOutput();

		if ((main_cycle & 0x1FFF) == 0) {
			debugMessageF("RC=%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n", RCchannel[0], RCchannel[1], RCchannel[2], RCchannel[3], RCchannel[4], RCchannel[5], RCchannel[6],RCchannel[7], RCchannel[8]);
			debugMessageF("OUT=%d,%d,%d,%d\r\n", getOutputChannelValue(0), getOutputChannelValue(1), getOutputChannelValue(2), getOutputChannelValue(3));
		}

		if ((main_cycle & 0x1000) != 0) {
			led_green_on();
			if (dbg1 == 0) {
				debugMessage(".\r\n");
				dbg1 = 1;
			}
		} else {
			led_green_off();
			dbg1 = 0;
		}

		for (uint8_t i = 0; i < RC_SWITCH_COUNT; i++) {
			if (switches[i] != RC_SWITCH_STATE_UNDEFINED) old_switches[i] = switches[i];
		}
	}
}
