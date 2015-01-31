/*
 * mpcTask.h
 *
 *  Author: Tomas Baca
 */

#ifndef MPCTASK_H_
#define MPCTASK_H_

void mpcTask(void *p);

#include "system.h"

#define REFERENCE_LENGTH	NUMBER_OF_STATES*HORIZON_LEN

// used to filter the setpoint
#define MAX_SPEED	0.4
#define DT			0.0114

typedef struct {

	float elevatorOutput;
	float aileronOutput;
} mpc2commMessage_t;

#endif /* MPCTASK_H_ */
