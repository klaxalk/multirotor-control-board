/*
 * elevatorKalman.h
 *
 *  Author: Tomas Baca
 */

#ifndef ELEVATORKALMAN_H_
#define ELEVATORKALMAN_H_

#include "kalman.h"

#define	NUMBER_OF_STATES_ELEVATOR	3
#define NUMBER_OF_INPUTS_ELEVATOR	1

#define DT_ELEVATOR					0.0114

// this struct holds all variables for this kalman
kalmanHandler_t elevatorKalmanHandler;

// this method initializes the kalman handler
void initializeElevatorKalman();

#endif /* ELEVATORKALMAN_H_ */
