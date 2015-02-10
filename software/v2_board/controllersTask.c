/*
 * controllersTask.c
 *
 * Created: 13.9.2014 23:57:14
 *  Author: Tomas Baca
 */ 

#include "controllersTask.h"
#include "controllers.h"
#include "system.h"
#include "config.h"

#include "mpcHandler.h"

void controllersTask(void *p) {
	
	mpcSetpoints.elevatorSetpoint = 0;
	mpcSetpoints.aileronSetpoint = 0;
	
	while (1) {
		
		altitudeEstimator();
		
		if (altitudeControllerEnabled == true)
			altitudeController();
		
		// makes the 70Hz loop
		vTaskDelay(14);
	}
}