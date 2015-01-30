/*
 * mpcMatrices.h
 *
 *  Author: Tomas Baca
 */

#ifndef MPCMATRICES_H_
#define MPCMATRICES_H_

#include "CMatrixLib.h"

#define NUMBER_OF_STATES 	3
#define HORIZON_LEN			200
#define REDUCED_HORIZON		38

#define A_ROOF_HEIGHT		HORIZON_LEN*NUMBER_OF_STATES
#define A_ROOF_WIDTH		NUMBER_OF_STATES

#define Q_ROOF_DIAG_SIZE	HORIZON_LEN*NUMBER_OF_STATES

#define	B_ROOF_HEIGHT		HORIZON_LEN*NUMBER_OF_STATES
#define	B_ROOF_WIDTH		REDUCED_HORIZON

#define	H_INV_HEIGHT		REDUCED_HORIZON
#define	H_INV_WIDTH			REDUCED_HORIZON

const float A_roof_data[A_ROOF_HEIGHT*A_ROOF_WIDTH];

const float Q_roof_diag_data[Q_ROOF_DIAG_SIZE];

const float B_roof_data[B_ROOF_HEIGHT*B_ROOF_WIDTH];

// for P = 0.0005
const float H_inv_data[H_INV_HEIGHT*H_INV_WIDTH];

// for P = 0.03
const float H_inv_data2[H_INV_HEIGHT*H_INV_WIDTH];

#endif /* MPCMATRICES_H_ */
