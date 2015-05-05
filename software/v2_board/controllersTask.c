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
		///setpointsCalculate();										
					
		//MANUAL
		if(controllerActive==CONTROLLERS.OFF){
			controllerAileronOutput=0;
			controllerElevatorOutput=0;
			controllerThrottleOutput=0;
		}else{		
			//FLIGHT
			if(landingState==LANDING.FLIGHT){	
				//VELOCITY	
				if(controllerActive==CONTROLLERS.VELOCITY){
					velocityController();
					altitudeController(altitudeTrajectory[0]);	
				}else
				//POSITION
				if(controllerActive==CONTROLLERS.POSITION){
					positionController(MPCElevatorTrajectory[0],MPCAileronTrajectory[0]);	
					altitudeController(altitudeTrajectory[0]);	
				}else
				//PREDICTIVE
				if(controllerActive==CONTROLLERS.MPC){					
					altitudeController(altitudeTrajectory[0]);	
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