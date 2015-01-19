/*
 * mpcTask.c
 *
 *  Author: Tomas Baca
 */

#include "mpcMatrices.h"
#include "uart_driver.h"
#include "system.h"

matrix_float A_roof;
vector_float Q_roof_diag;
matrix_float B_roof;
matrix_float H_inv;

void initMpcMatrices() {

	// setup the A_roof matrix
	A_roof.data = (float*) &A_roof_data;
	A_roof.height =	A_ROOF_HEIGHT;
	A_roof.width = A_ROOF_WIDTH;
	A_roof.name = "A_roof matrix";

	// setup the Q_roof_diag vector
	Q_roof_diag.data = (float*) &Q_roof_diag_data;
	Q_roof_diag.length = Q_ROOF_DIAG_SIZE;
	Q_roof_diag.name = "Q_roof_diag vector";

	// setup the B_roof matrix
	B_roof.data = (float*) &B_roof_data;
	B_roof.height =	B_ROOF_HEIGHT;
	B_roof.width = B_ROOF_WIDTH;
	B_roof.name = "B_roof matrix";

	// setup the H_inv matrix
	H_inv.data = (float*) &H_inv_data;
	H_inv.height =	H_INV_HEIGHT;
	H_inv.width = H_INV_WIDTH;
	H_inv.name = "H_inv matrix";

}

void mpcTask(void *p) {


}
