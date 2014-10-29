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


void controllersTask(void *p) {
	
	initTrajectory();
	while (1) {
			//If controllerEnabled == 0 the controller output signals
			//are "unplugged" (in mergeSignalsToOutput) but the
			//controllers keep running.
			//When the controllers are turned on, it's integral actions
			//are reset (in enableController).

			positionEstimator();
			altitudeEstimator();

			setpoints();

			landingStateAutomat();

			positionController();
			altitudeController();
						
		// makes the 70Hz loop
		vTaskDelay(14);
	}
	
}