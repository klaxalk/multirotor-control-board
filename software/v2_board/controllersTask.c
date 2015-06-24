#include "controllersTask.h"
#include "controllers.h"
#include "system.h"
#include "constants.h"
#include "mpcHandler.h"

#include "communication.h"

void controllersTask(void *p) {	
	initTrajectory();
	while (1) {					

		//values calculation
		altitudeEstimator();
		setpointsCalculate();										
					
		//MANUAL
		if(controllerActive==CONTROLLERS_OFF){
			controllerAileronOutput=0;
			controllerElevatorOutput=0;
			controllerThrottleOutput=0;
		}else{		
			//FLIGHT
			if(landingState==LANDING_FLIGHT){				
				//POSITION
				if(controllerActive==CONTROLLERS_ALTITUDE){
					altitudeController(setpoints.altitude);	
					controllerElevatorOutput=0;
					controllerAileronOutput=0;
				}else
				//PREDICTIVE
				if(controllerActive==CONTROLLERS_MPC){					
					altitudeController(setpoints.altitude);	
					controllerElevatorOutput = saturationInt16(mpcElevatorOutput,CONTROLLER_ELEVATOR_SATURATION);
					controllerAileronOutput = saturationInt16(mpcAileronOutput,CONTROLLER_AILERON_SATURATION);	
				}
			}else{
				//LANDING
				landingController();						
			}	
		}		
		// makes the 70Hz loop
		vTaskDelay(14);
	}	
}