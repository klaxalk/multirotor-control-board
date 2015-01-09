/*
* main.c
*
* Created: 24.8.2014 15:10:04
*  Author: Tomas Baca
*/

#include "FreeRTOS.h"
#include "task.h"
#include "packets.h"

// basic system functions
#include "system.h"

// the main task routine
#include "mainTask.h"

// the communication task
#include "commTask.h"

// the controllersTask
#include "controllersTask.h"

// the logTask
#include "logTask.h"

int main(void)
{
		
	// initialize the hardware
	boardInit();
	
	//XBee protocol constants initialize
	constInit();
	
	/* -------------------------------------------------------------------- */
	/*	Start the communication task routine								*/
	/* -------------------------------------------------------------------- */
	xTaskCreate(commTask, (signed char*) "commTask", 1024, NULL, 2, NULL);
	
	/* -------------------------------------------------------------------- */
	/*	Start the main task routine																					*/
	/* -------------------------------------------------------------------- */
	xTaskCreate(mainTask, (signed char*) "mainTask", 1024, NULL, 2, NULL);
	
	/* -------------------------------------------------------------------- */
	/*	Start the main task routine																					*/
	/* -------------------------------------------------------------------- */
	xTaskCreate(controllersTask, (signed char*) "contTasks", 1024, NULL, 2, NULL);
	
	/* -------------------------------------------------------------------- */
	/*	Start the openLog task routine																				*/
	/* -------------------------------------------------------------------- */
	xTaskCreate(logTask, (signed char*) "logTasks", 512, NULL, 2, NULL);
	
	/* -------------------------------------------------------------------- */
	/*	Start the FreeRTOS scheduler																				*/
	/* -------------------------------------------------------------------- */
	vTaskStartScheduler();
	
	return 0;
}

