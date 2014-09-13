/*
 * controllersTask.c
 *
 * Created: 13.9.2014 23:57:14
 *  Author: Tomas Baca
 */ 

#include "controllersTask.h"
#include "system.h"
#include "FreeRTOS.h"
#include "task.h"
#include "controllers.h"
#include "ioport.h"

void controllersTask(void *p) {
	
	while (1) {
		
		#if PX4FLOW_DATA_RECEIVE == ENABLED

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

		#endif // PX4FLOW_DATA_RECEIVE == ENABLED
		
		led_orange_on();
		
		vTaskDelay(14);
	}
	
}