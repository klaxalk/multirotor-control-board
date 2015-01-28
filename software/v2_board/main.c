/*
* main.c
*
* Created: 24.8.2014 15:10:04
*  Author: Tomas Baca
*/

#include "FreeRTOS.h"
#include "task.h"

// basic system functions
#include "system.h"

// the main task routine
#include "mainTask.h"

// the communication task
#include "commTask.h"

// the controllersTask
#include "controllersTask.h"

#include "logTask.h"

int main(void)
{
		
	// initialize the hardware
	boardInit();

	/* -------------------------------------------------------------------- */
	/*	Start the communication task routine								*/
	/* -------------------------------------------------------------------- */
	xTaskCreate(commTask, (signed char*) "commTask", 1024, NULL, 2, NULL);
	
	/* -------------------------------------------------------------------- */
	/*	Start the main task routine											*/
	/* -------------------------------------------------------------------- */
	xTaskCreate(mainTask, (signed char*) "mainTask", 512, NULL, 2, NULL);
	
	/* -------------------------------------------------------------------- */
	/*	Start the controllers task routine									*/
	/* -------------------------------------------------------------------- */
	xTaskCreate(controllersTask, (signed char*) "contTasks", 512, NULL, 2, NULL);
	
	/* -------------------------------------------------------------------- */
	/*	Start the data logging task routine									*/
	/* -------------------------------------------------------------------- */
	xTaskCreate(logTask, (signed char*) "logTask", 1024, NULL, 2, NULL);
	
	/* -------------------------------------------------------------------- */
	/*	Start the FreeRTOS scheduler										*/
	/* -------------------------------------------------------------------- */
	vTaskStartScheduler();
	
	return 0;
}

