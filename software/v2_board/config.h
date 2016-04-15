/*
 * config.h
 *
 * Created: 28.7.2015 10:44:39
 *  Author: klaxalk
 */ 


#ifndef CONFIG_H_
#define CONFIG_H_

#define RASPBERRY_PI		1
// #define ARGOS			1
// #define MULTICON			1

// #define IDENTIFICATION	1

// #define PID_POSITION_CONTROLLER		1
#define MPC_POSITION_CONTROLLER		1

// #define MIKROKOPTER_KK2		1
// #define TRICOPTER		1
#define PRASE			1

// #define LEFT_FOLLOWER	1
// #define RIGHT_FOLLOWER		1
#define LEADER			1

// #define MATOUS				1
// #define ZIG_ZAG	1
#define CIRCLE 1

#define RASPBERRY_FORWARD	1
// #define RASPBERRY_DOWNWARD	1
 
// #define MAGNETOMETER		1
	
/* -------------------------------------------------------------------- */
/*	An automatic part of the configuration, do not modify				*/
/* -------------------------------------------------------------------- */
	
#ifdef PRASE
	#define PPM_INPUT		1
	
	#define GRIPPER			1
#endif

#ifdef TRICOPTER
	#define PPM_INPUT		1
#endif

#ifdef MIKROKOPTER_KK2
	#define PWM_INPUT		1
#endif

// check if only one position controller is used at the time
#if defined(MPC_POSITION_CONTROLLER) && defined(PID_POSITION_CONTROLLER)

	#error PID and MPC position controllers cannot be used simultaneously

#endif

// chech if the UAV type is specified
#if ! (defined(MPC_POSITION_CONTROLLER)  || defined(PID_POSITION_CONTROLLER))

	#error No position controller is specified

#endif

#if ! (defined(PRASE)  || defined(MIKROKOPTER_KK2) || defined(TRICOPTER))

#error No UAV type defined (PRASE, MIKROKOPTER_KK2, TRICOPTER)

#endif

#endif /* CONFIG_H_ */