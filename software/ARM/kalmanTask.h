/*
 * kalmanTask.h
 *
 *  Author: Tomas Baca
 */

#ifndef KALMANTASK_H_
#define KALMANTASK_H_

#include "system.h"
#include "CMatrixLib.h"
#include "kalman.h"
#include "miscellaneous.h"

typedef struct {

	float dt;
	float elevatorSpeed;
	float aileronSpeed;
	float elevatorInput;
	float aileronInput;
} comm2kalmanMessage_t;

typedef struct {

	float elevatorData[3];	/* REWORK */
	float aileronData[3];	/* REWORK */
} kalman2mpcMessage_t;

typedef struct {

	float elevatorData[3];	/* REWORK */
	float aileronData[3];	/* REWORK */
} kalman2commMessage_t;

// the communication task
void kalmanTask(void *p);

#endif /* KALMANTASK_H_ */
