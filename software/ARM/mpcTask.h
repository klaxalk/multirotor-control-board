/*
 * mpcTask.h
 *
 *  Author: Tomas Baca
 */

#ifndef MPCTASK_H_
#define MPCTASK_H_

void mpcTask(void *p);

#include "system.h"

typedef struct {

	float elevatorOutput;
	float aileronOutput;
} mpcOutputMessage;

#endif /* MPCTASK_H_ */
