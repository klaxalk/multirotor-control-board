/*
 * config.h
 *
 *  Author: klaxalk
 */

#ifndef CONFIG_H_
#define CONFIG_H_

// #define MIKROKOPTER_KK2	1
#define TRICOPTER		1
// #define PRASE	1

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

	#define THROTTLE_A43 -0.6466
	#define THROTTLE_A44 1.5736
	#define THROTTLE_B31 0.002711
	#define THROTTLE_B32 0.003372
	#define THROTTLE_B33 -17.7363
	#define THROTTLE_B41 0.001897
	#define THROTTLE_B42 0.08581
	#define THROTTLE_B43 -13.3891
    #define THROTTLE_Q 120
#endif

#endif
