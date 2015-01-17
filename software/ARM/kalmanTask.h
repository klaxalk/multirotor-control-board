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

// the communication task
void kalmanTask(void *p);

#endif /* KALMANTASK_H_ */
