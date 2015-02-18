/*
 * mpcTask.h
 *
 *  Author: Tomas Baca
 */

#ifndef MPCTASK_H_
#define MPCTASK_H_

void mpcTask(void *p);

/**
 *	Set up the trajectory (over the horizon) interpolation method
 */
#define LINEAR		1
#define LOGARITHMIC	2
#define EXPONENTIAL	3

#define TRAJECTORY_INTERPOLATION	2

#include "system.h"

#endif /* MPCTASK_H_ */
