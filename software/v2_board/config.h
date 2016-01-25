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

#define IDENTIFICATION	1

// #define MIKROKOPTER_KK2		1
// #define TRICOPTER		1
#define PRASE			1

// #define LEFT_FOLLOWER		1
// #define RIGHT_FOLLOWER	1
#define LEADER				1

// #define RASPBERRY_FORWARD	1
#define RASPBERRY_DOWNWARD	1
 
// #define MAGNETOMETER		1

#ifdef PRASE
	#define PPM_INPUT		1
#endif

#ifdef TRICOPTER
	#define PPM_INPUT		1
#endif

#ifdef MIKROKOPTER_KK2
	#define PWM_INPUT		1
#endif

#endif /* CONFIG_H_ */