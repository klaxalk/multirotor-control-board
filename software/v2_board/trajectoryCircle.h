/*
 * zigZagTrajectory.h
 *
 * Created: 24.2.2016 10:15:26
 *  Author: klaxalk
 */ 


#ifndef CIRCLE_H_
#define CIRCLE_H_

#include "system.h"
#include <avr/pgmspace.h>

#ifdef CIRCLE

#define TRAJECTORY_CIRCLE_LENGTH 5800

const float elevatorCircle[TRAJECTORY_CIRCLE_LENGTH] PROGMEM;
const float aileronCircle[TRAJECTORY_CIRCLE_LENGTH] PROGMEM;

#endif

#endif /* ZIGZAGTRAJECTORY_H_ */