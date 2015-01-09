/*
 * logTask.c
 *
 * Created: 9. 1. 2015 14:36:58
 *  Author: Martin
 */ 

#include "controllersTask.h"
#include "system.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ioport.h"
#include <openLog.h>

//#define USART_LOG_BAUDRATE	BAUD115200 + SD card

void logTask(void *p) {
	// Start logging to the file "FILELOG.TXT"
	startLogging("fileLog.txt");		
	
	while (1) {
		loggingData();
		vTaskDelay(16);
		//delay 2+ ---- 16=62,5Hz
	}
}