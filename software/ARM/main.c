/*
 * main.c
 *
 *  Author: Tomas Baca
 */

// GPIO initialization and other minor stuff
#include "system.h"
#include "commTask.h"
#include "kalmanTask.h"
#include "mpcTask.h"

int main(void)
{

	// this line is one ******* SUNDAY
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	// initialize the hardware
	boardInit();

	/* -------------------------------------------------------------------- */
	/*	Start the communication task routine								*/
	/* -------------------------------------------------------------------- */
	xTaskCreate(commTask, (char*) "commTask", 4092, NULL, 2, NULL);

	/* -------------------------------------------------------------------- */
	/*	Start the kalman filter task										*/
	/* -------------------------------------------------------------------- */
	xTaskCreate(kalmanTask, (char*) "kalman", 4092, NULL, 2, NULL);

	/* -------------------------------------------------------------------- */
	/*	Start the mpc task filter task										*/
	/* -------------------------------------------------------------------- */
	xTaskCreate(mpcTask, (char*) "mpcTask", 4092, NULL, 2, NULL);

	/* -------------------------------------------------------------------- */
	/*	Start the FreeRTOS scheduler										*/
	/* -------------------------------------------------------------------- */
	vTaskStartScheduler();

	return 0;
}
