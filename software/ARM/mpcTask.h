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

	int_least16_t elevatorOutput;
	int_least16_t aileronOutput;
} mpcOutputMessage;

#endif /* MPCTASK_H_ */
