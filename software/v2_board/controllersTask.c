#include "controllersTask.h"
#include "controllers.h"
#include "system.h"
#include "constants.h"


void controllersTask(void *p) {	
	initTrajectory();
	controllerSet(CONTROLLERS.OFF);
	while (1) {					
		
		//values calculation
		positionEstimator();
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
					velocityController(speedDesiredSetpoint.elevator,speedDesiredSetpoint.aileron);
					altitudeController(positionDesiredSetpoint.altitude);	
				}else
				//POSITION
				if(controllerActive==CONTROLLERS.POSITION){
					positionController(positionDesiredSetpoint.elevator,positionDesiredSetpoint.aileron);	
					altitudeController(positionDesiredSetpoint.altitude);	
				}else
				//PREDICTIVE
				if(controllerActive==CONTROLLERS.MPC){
			
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