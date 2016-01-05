/*
 * trajectories.h
 *
 * Created: 16.2.2015 14:24:41
 *  Author: klaxalk
 */ 


#ifndef TRAJECTORIES_H_
#define TRAJECTORIES_H_

#include "system.h"
#include <avr/pgmspace.h>

#define TRAJECTORY_LENGTH 2717

const float elevator[TRAJECTORY_LENGTH] PROGMEM;
const float aileron[TRAJECTORY_LENGTH] PROGMEM;

#if defined(LEFT_FOLLOWER) || defined(RIGHT_FOLLOWER)

const float elevatorDiff[TRAJECTORY_LENGTH] PROGMEM;
const float aileronDiff[TRAJECTORY_LENGTH] PROGMEM;

#endif

#endif /* TRAJECTORIES_H_ */