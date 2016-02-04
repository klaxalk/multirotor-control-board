/*
 * config.h
 *
 *  Author: klaxalk
 */

#ifndef CONFIG_H_
#define CONFIG_H_

// #define MIKROKOPTER_KK2	1
// #define TRICOPTER		1
#define PRASE	1

#ifdef	MIKROKOPTER_KK2

	#define ATTITUDE_P0	0.9783
	#define ATTITUDE_P1	0.000063151
	#define KALMAN_Q 120

#endif

#ifdef	TRICOPTER

	#define ATTITUDE_P0	0.9759
	#define ATTITUDE_P1	0.00004279
	#define KALMAN_Q 120

#endif

#ifdef	PRASE
	#define ATTITUDE_P0	0.9777
	#define ATTITUDE_P1	0.000044664
	#define KALMAN_Q 120
#endif

#endif
