/*
 * commTask.h
 *
 *  Author: Tomas Baca
 */

#ifndef COMMTASK_H_
#define COMMTASK_H_

#include "system.h"

#define XMEGA_BUFFER_SIZE 64

// the communication task
void commTask(void *p);

typedef struct {

	float elevatorReference;
	float aileronReference;
} comm2mpcMessage_t;

#endif /* COMMTASK_H_ */
