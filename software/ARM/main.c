/*
 * main.c
 *
 *  Author: Tomas Baca
 */

// GPIO initialization and other minor stuff
#include "system.h"
#include "commTask.h"
#include "kalmanTask.h"

int main(void)
{

	// initialize the hardware
    boardInit();

	/* -------------------------------------------------------------------- */
	/*	Start the communication task routine								*/
	/* -------------------------------------------------------------------- */
	xTaskCreate(commTask, (char*) "commTask", 1024, NULL, 2, NULL);

	/* -------------------------------------------------------------------- */
	/*	Start the kalman filter task										*/
	/* -------------------------------------------------------------------- */
	xTaskCreate(kalmanTask, (char*) "kalman", 1024, NULL, 2, NULL);

	/* -------------------------------------------------------------------- */
	/*	Start the FreeRTOS scheduler										*/
	/* -------------------------------------------------------------------- */
	vTaskStartScheduler();

	return 0;
}
