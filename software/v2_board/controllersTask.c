/*
 * controllersTask.c
 *
 * Created: 13.9.2014 23:57:14
 *  Author: Tomas Baca
 */ 

#include "controllersTask.h"
#include "controllers.h"
#include "system.h"

#include "mpcHandler.h"

void controllersTask(void *p) {
	
	unsigned int startTimeMillis, endTimeMillis;
	
	while (1) {
		
		startTimeMillis = milisecondsTimer;

		//altitudeEstimator();
		altitudeEvaluateAndSendToKalman();
		
		//if (altitudeControllerEnabled == true)
			//altitudeController();
				
		endTimeMillis = milisecondsTimer;
		
		if (startTimeMillis >= endTimeMillis) //timer overflowed
			endTimeMillis += 1000;
			
		vTaskDelay(DT_MS - (endTimeMillis - startTimeMillis)); // makes the 20Hz loop
	}
}