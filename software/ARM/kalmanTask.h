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

kalmanHandler elevatorHandler;

typedef struct {

	float dt;
	float elevatorSpeed;
	float aileronSpeed;
	float elevatorInput;
	float aileronInput;
} px4flowMessage;

// the communication task
void kalmanTask(void *p);

#endif /* KALMANTASK_H_ */
