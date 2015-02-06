/*
 * aileronKalman.h
 *
 *  Author: Tomas Baca
 */

#ifndef AILERONKALMAN_H_
#define AILERONKALMAN_H_

#include "kalman.h"

#define	NUMBER_OF_STATES_AILERON	3
#define NUMBER_OF_INPUTS_AILERON	1

#define DT_AILERON					0.0114

// this struct holds all variables for this kalman
kalmanHandler_t aileronKalmanHandler;

// this method initializes the kalman handler
void initializeAileronKalman();

#endif /* AILERONKALMAN_H_ */
