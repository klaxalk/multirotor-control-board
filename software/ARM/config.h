/*
 * config.h
 *
 *  Author: klaxalk
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#define MIKROKOPTER_KK2	1
// #define TRICOPTER		1

#ifdef	MIKROKOPTER_KK2

	#define ATTITUDE_P0	0.9783
	#define ATTITUDE_P1	0.000063151
	#define KALMAN_Q 120

#endif

#ifdef	TRICOPTER

	#define ATTITUDE_P0	0.9796
	#define ATTITUDE_P1	5.1300e-05
	#define KALMAN_Q 120

#endif

#endif
