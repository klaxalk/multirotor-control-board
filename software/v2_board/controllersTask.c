#include "controllersTask.h"
#include "controllers.h"
#include "system.h"
#include "packets.h"


void controllersTask(void *p) {	
	initTrajectory();
	controllerSet(CONTROLLERS.OFF);
	while (1) {	
			
		//Leading data check
		if (leadFreshTimer>300){
			blobAileronDeflection=0;
			blobElevatorDeflection=0;
		}		
		
		//values calculation
		positionEstimator();
		altitudeEstimator();
		setpointsCalculate();						
		
		//lock on blob with leading data improvement
		if(lockOnBlobDistance>=0){
			lockOnBlob(lockOnBlobDistance);
		}		
		
					
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
					velocityController(elevatorDesiredVelocitySetpoint,aileronDesiredVelocitySetpoint);
					altitudeController(throttleDesiredSetpoint);	
				}else
				//POSITION
				if(controllerActive==CONTROLLERS.POSITION){
					positionController(elevatorDesiredPositionSetpoint,aileronDesiredPositionSetpoint);	
					altitudeController(throttleDesiredSetpoint);	
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