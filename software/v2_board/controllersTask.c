#include "controllersTask.h"
#include "controllers.h"
#include "system.h"
#include "constants.h"
#include "mpcHandler.h"


void controllersTask(void *p) {	
	initTrajectory();
	while (1) {					
		
		//values calculation
		altitudeEstimator();
		setpointsCalculate();										
					
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
					altitudeController(positionSetpoint.altitude);	
				}else
				//POSITION
				if(controllerActive==CONTROLLERS.POSITION){
					positionController(positionSetpoint.elevator,positionSetpoint.aileron);	
					altitudeController(positionSetpoint.altitude);	
				}else
				//PREDICTIVE
				if(controllerActive==CONTROLLERS.MPC){					
					altitudeController(positionSetpoint.altitude);	
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