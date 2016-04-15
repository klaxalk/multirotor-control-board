/*
 * config.h
 *
 * Created: 28.7.2015 10:44:39
 *  Author: klaxalk
 */ 


#ifndef CONFIG_H_
#define CONFIG_H_

/* -------------------------------------------------------------------- */
/*	Choose onboard modules												*/
/* -------------------------------------------------------------------- */
#define RASPBERRY_PI		1
// #define ARGOS			1
// #define MULTICON			1

/* -------------------------------------------------------------------- */
/*	Choose position controller											*/
/* -------------------------------------------------------------------- */
// #define PID_POSITION_CONTROLLER		1
#define MPC_POSITION_CONTROLLER		1

/* -------------------------------------------------------------------- */
/*	Choose UAV model													*/
/* -------------------------------------------------------------------- */
// #define MIKROKOPTER_KK2		1
// #define TRICOPTER		1
#define PRASE			1

#define CIRCLE 1

/* -------------------------------------------------------------------- */
/*	If Raspberry is onboard, choose the camera heading					*/
/* -------------------------------------------------------------------- */
#if defined(RASPBERRY_PI)

	#define RASPBERRY_FORWARD	1
	// #define RASPBERRY_DOWNWARD	1

#endif
	
/* -------------------------------------------------------------------- */
/*	An automatic part of the configuration, do not modify 				*/
/* -------------------------------------------------------------------- */
	
#ifdef PRASE

	// tricopter uses single PPM input channel from RC receiver
	#define PPM_INPUT		1
	
	// prase has the magnetic gripper
	#define GRIPPER			1
#endif

#ifdef TRICOPTER

	// tricopter uses single PPM input channel from RC receiver
	#define PPM_INPUT		1
#endif

#ifdef MIKROKOPTER_KK2

	// mikrokopter uses 9 PWM channels from RC receiver
	#define PWM_INPUT		1
#endif

// check if only one position controller is used at the time
#if defined(MPC_POSITION_CONTROLLER) && defined(PID_POSITION_CONTROLLER)

	#error PID and MPC position controllers cannot be used simultaneously

#endif

// check if the position controller is specified
#if ! (defined(MPC_POSITION_CONTROLLER)  || defined(PID_POSITION_CONTROLLER))

	#error No position controller is specified (PID, MPC)

#endif

// check if the UAV model is specified
#if ! (defined(PRASE)  || defined(MIKROKOPTER_KK2) || defined(TRICOPTER))

#error No UAV type model is defined (PRASE, MIKROKOPTER_KK2, TRICOPTER)

#endif

#endif /* CONFIG_H_ */