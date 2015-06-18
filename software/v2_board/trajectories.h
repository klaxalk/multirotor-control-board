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

#define TRAJECTORY_CIRCLE_LENGTH 5654

const float elevatorCircle[TRAJECTORY_CIRCLE_LENGTH] PROGMEM;
const float aileronCircle[TRAJECTORY_CIRCLE_LENGTH] PROGMEM;

#endif /* TRAJECTORIES_H_ */