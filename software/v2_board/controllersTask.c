/*
* controllersTask.c
*
* Created: 13.9.2014 23:57:14
*  Author: Tomas Baca
*/

#include "controllersTask.h"
#include "controllers.h"
#include "communication.h"
#include "system.h"
#include "mpcHandler.h"

volatile char flightStarted = 0;

void controllersTask(void *p) {
	
	int16_t startTimeMillis, endTimeMillis;
	flightStarted = 0;
	
	while (1) {
		
		startTimeMillis = milisecondsTimer;

		if (flightStarted) {

			kalmanStep();
			
			if (altitudeControllerEnabled == true) {
				calculateNextThrottle();
			}
		} else {
			resetThrottleKalman();
		}
		
		if ((groundDistance > 0.35) && (RCchannel[THROTTLE] > 6000)) {
			flightStarted = 1;
		}
		
		
		endTimeMillis = milisecondsTimer;
		
		if (startTimeMillis > endTimeMillis) //timer overflowed
		endTimeMillis += 1000;
		
		vTaskDelay((int16_t) ((int16_t) DT_MS - (endTimeMillis - startTimeMillis))); // makes the 20Hz loop
	}
}