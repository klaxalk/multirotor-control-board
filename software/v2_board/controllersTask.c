#include "controllersTask.h"
#include "controllers.h"
#include "system.h"
#include "config.h"
#include "packets.h"


void controllersTask(void *p) {	
	initTrajectory();
	while (1) {
		leadingDataActualCheck();
		positionEstimator();
		altitudeEstimator();
		
		if(landingState!=LS_FLIGHT){
			setpointsFilter(landingThrottleSetpoint,estimatedAileronPos,estimatedElevatorPos,0,0);
		}else if(trajectoryEnabled==1 && controllerActive==CONTROLLERS.POSITION){
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