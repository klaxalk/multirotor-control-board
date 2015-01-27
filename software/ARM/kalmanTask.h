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

#define NUMBER_OF_STATES 3
#define NUMBER_OF_INPUTS 1
#define NUMBER_OF_MEASURED_STATES 1

kalmanHandler elevatorHandler;
kalmanHandler aileronHandler;

typedef struct {

	float dt;
	float elevatorSpeed;
	float aileronSpeed;
	float elevatorInput;
	float aileronInput;
} comm2kalmanMessage_t;

typedef struct {

	float elevatorData[NUMBER_OF_STATES];
	float aileronData[NUMBER_OF_STATES];
} kalman2mpcMessage_t;

typedef struct {

	float elevatorData[3];
	float aileronData[3];
} kalman2commMessage_t;

// the communication task
void kalmanTask(void *p);

#endif /* KALMANTASK_H_ */
