/*
 * throttleKalman.h
 *
 *  Author: Tomas Baca
 */

#ifndef THROTTLEKALMAN_H_
#define THROTTLEKALMAN_H_

#include "kalman.h"

#define	NUMBER_OF_STATES_THROTTLE	4
#define NUMBER_OF_INPUTS_THROTTLE	3
#define NUMBER_OF_MEASUREMENTS_THROTTLE	1

#define DT_THROTTLE					0.05

// this method initializes the kalman handler
kalmanHandler_t * initializeThrottleKalman();

#endif /* THROTTLEKALMAN_H_ */
