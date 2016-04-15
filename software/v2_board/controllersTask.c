/*
 * controllersTask.c
 *
 * Created: 13.9.2014 23:57:14
 *  Author: Tomas Baca
 */ 

#include "controllersTask.h"
#include "controllers.h"
#include "system.h"
#include "controllers.h"
#include "mpcHandler.h"

void controllersTask(void *p) {
	
	while (1) {
		
		altitudeEstimator();
		
		if (altitudeControllerEnabled == true)
			altitudeController();
			
		#ifdef PID_POSITION_CONTROLLER 
		
			int16_t tempInt;
		
			if (positionControllerEnabled) {
				
				tempInt = calculatePID(elevator_reference, &elevator_prev_error, &elevator_integration, kalmanStates.elevator.position, 300, 800, 0.05, 0.014, 800);
				
				// stop the realtime OS from context switching while copying the result
				portENTER_CRITICAL();
				controllerElevatorOutput = tempInt;
				portEXIT_CRITICAL();
				
				tempInt = calculatePID(aileron_reference, &aileron_prev_error, &aileron_integration, kalmanStates.aileron.position, 300, 800, 0.05, 0.014, 800);
				
				// stop the realtime OS from context switching while copying the result
				portENTER_CRITICAL();
				controllerAileronOutput = tempInt;
				portEXIT_CRITICAL();
			}
		
		#endif
		
		// makes the 70Hz loop
		vTaskDelay(14);
	}
}