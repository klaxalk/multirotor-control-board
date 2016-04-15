/*
 * zigZagTrajectory.h
 *
 * Created: 24.2.2016 10:15:26
 *  Author: klaxalk
 */ 


#ifndef ZIGZAGTRAJECTORY_H_
#define ZIGZAGTRAJECTORY_H_

#include "system.h"
#include <avr/pgmspace.h>

#ifdef ZIG_ZAG

#define TRAJECTORY_ZIG_ZAG_LENGTH 4341

const float elevatorZigZag[TRAJECTORY_ZIG_ZAG_LENGTH];
const float aileronZigZag[TRAJECTORY_ZIG_ZAG_LENGTH];

#endif

#endif /* ZIGZAGTRAJECTORY_H_ */