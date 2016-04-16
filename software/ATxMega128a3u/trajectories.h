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

#define TRAJECTORY_LENGTH 629

const float trajectoryElevator[TRAJECTORY_LENGTH] PROGMEM;
const float trajectoryAileron[TRAJECTORY_LENGTH] PROGMEM;


#endif /* TRAJECTORIES_H_ */