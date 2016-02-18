/*
 * aileronKalman.h
 *
 *  Author: Tomas Baca
 */

#ifndef AILERONKALMAN_H_
#define AILERONKALMAN_H_

#include "kalman.h"

#define	NUMBER_OF_STATES_AILERON	5
#define NUMBER_OF_INPUTS_AILERON	1
#define NUMBER_OF_MEASUREMENTS_AILERON	1

#define DT_AILERON					0.0101

// this method initializes the kalman handler
kalmanHandler_t * initializeAileronKalman();

#endif /* AILERONKALMAN_H_ */
