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

	#define THROTTLE_A43 -0.65658
	#define THROTTLE_A44 1.5714

	#define THROTTLE_B31 0.00091141
	#define THROTTLE_B32 -0.36830
	#define THROTTLE_B33 -2.8005
	#define THROTTLE_B41 0.0011525
	#define THROTTLE_B42 -0.052414
	#define THROTTLE_B43 -7.5757
    #define THROTTLE_Q 330
#endif

#endif
