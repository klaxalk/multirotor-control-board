/*
 * mpcMatrices.h
 *
 *  Author: Tomas Baca
 */

#ifndef MPCMATRICES_H_
#define MPCMATRICES_H_

#include "CMatrixLib.h"

#define A_ROOF_HEIGHT		600
#define A_ROOF_WIDTH		3

#define Q_ROOF_DIAG_SIZE	600

#define	B_ROOF_HEIGHT		600
#define	B_ROOF_WIDTH		38

#define	H_INV_HEIGHT		38
#define	H_INV_WIDTH			38

const float A_roof_data[A_ROOF_HEIGHT*A_ROOF_WIDTH];

const float Q_roof_diag_data[Q_ROOF_DIAG_SIZE];

const float B_roof_data[B_ROOF_HEIGHT*B_ROOF_WIDTH];

const float H_inv_data[H_INV_HEIGHT*H_INV_WIDTH];

#endif /* MPCMATRICES_H_ */
