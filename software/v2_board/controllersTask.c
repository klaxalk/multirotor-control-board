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
		leadingDataActualCheck();
		positionEstimator();
		altitudeEstimator();
		
		if(landingState!=LS_FLIGHT){
			setpointsFilter(landingThrottleSetpoint,estimatedAileronPos,estimatedElevatorPos,0,0);
		}else if(trajectoryEnabled==1 && positionControllerEnabled==1 && gumstixEnabled==0){
			trajectorySetpoints();
		}else{
			setpointsFilter(throttleDesiredSetpoint,aileronDesiredPositionSetpoint,elevatorDesiredPositionSetpoint,aileronDesiredVelocitySetpoint,elevatorDesiredVelocitySetpoint);
		}
			
		landingStateAutomat();
		
		velocityController();
		positionController();
		altitudeController();		
					
		// makes the 70Hz loop
		vTaskDelay(14);
	}
	
}