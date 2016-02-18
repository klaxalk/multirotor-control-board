/*
 * elevatorKalman.h
 *
 *  Author: Tomas Baca
 */

#ifndef ELEVATORKALMAN_H_
#define ELEVATORKALMAN_H_

#include "kalman.h"

#define	NUMBER_OF_STATES_ELEVATOR	5
#define NUMBER_OF_INPUTS_ELEVATOR	1
#define NUMBER_OF_MEASUREMENTS_ELEVATOR	1

#define DT_ELEVATOR					0.0101

// this method initializes the kalman handler
kalmanHandler_t * initializeElevatorKalman();

#endif /* ELEVATORKALMAN_H_ */
