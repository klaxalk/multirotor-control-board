#include "controllersTask.h"
#include "controllers.h"
#include "system.h"
#include "config.h"
#include "packets.h"


void controllersTask(void *p) {	
	initTrajectory();
	controllerSet(CONTROLLERS.OFF);
	while (1) {
		leadingDataActualCheck();
		
		positionEstimator();
		altitudeEstimator();
		
		if(trajectoryEnabled==1 && controllerActive==CONTROLLERS.POSITION){
			trajectorySetpoints();
		}
					
		//MANUAL
		if(controllerActive==CONTROLLERS.OFF){
			controllerAileronOutput=0;
			controllerElevatorOutput=0;
			controllerThrottleOutput=0;
		}else		
		//FLIGHT
		if(landingState=LANDING.FLIGHT){	
			//VELOCITY	
			if(controllerActive==CONTROLLERS.VELOCITY){
				velocityController(&controllerElevatorOutput,&controllerAileronOutput,elevatorDesiredVelocitySetpoint,aileronDesiredVelocitySetpoint);
				altitudeController(&controllerThrottleOutput,throttleDesiredSetpoint);	
			}else
			//POSITION
			if(controllerActive==CONTROLLERS.POSITION){
				positionController(&controllerElevatorOutput,&controllerAileronOutput,elevatorDesiredPositionSetpoint,aileronDesiredPositionSetpoint);	
				altitudeController(&controllerThrottleOutput,throttleDesiredSetpoint);	
			}else
			//PREDICTIVE
			if(controllerActive==CONTROLLERS.MPC){
			
			}
		}else{
			//LANDING
			landingController(&controllerThrottleOutput,&controllerElevatorOutput,&controllerAileronOutput);						
		}			
		// makes the 70Hz loop
		vTaskDelay(14);
	}
	
}