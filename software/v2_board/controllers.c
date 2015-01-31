#include "controllers.h"
#include "communication.h"

#include "packets.h"
#include "commands.h"

// controllers output variables
volatile int16_t controllerElevatorOutput=0;
volatile int16_t controllerAileronOutput=0;
volatile int16_t controllerRudderOutput=0;
volatile int16_t controllerThrottleOutput=0;

// controller select
volatile unsigned char controllerActive = 0x01;

//vars for estimators
volatile float estimatedElevatorPos = 0;
volatile float estimatedAileronPos  = 0;
volatile float estimatedThrottlePos = 0;

volatile float estimatedElevatorVel = 0;
volatile float estimatedAileronVel  = 0;
volatile float estimatedThrottleVel = 0;

volatile float estimatedElevatorAcc = 0;
volatile float estimatedAileronAcc = 0;

volatile float estimatedBlobDistance = 0;
volatile float estimatedBlobHorizontal = 0;
volatile float estimatedBlobVertical = 0;

//setpoints
volatile float elevatorPositionSetpoint =  DEFAULT_ELEVATOR_POSITION_SETPOINT;
volatile float aileronPositionSetpoint  = DEFAULT_AILERON_POSITION_SETPOINT;
volatile float throttleSetpoint = DEFAULT_THROTTLE_SETPOINT;
volatile float elevatorVelocitySetpoint =  DEFAULT_ELEVATOR_VELOCITY_SETPOINT;
volatile float aileronVelocitySetpoint  = DEFAULT_AILERON_VELOCITY_SETPOINT;

volatile float elevatorDesiredPositionSetpoint = DEFAULT_ELEVATOR_POSITION_SETPOINT;
volatile float aileronDesiredPositionSetpoint  = DEFAULT_AILERON_POSITION_SETPOINT;
volatile float throttleDesiredSetpoint = DEFAULT_THROTTLE_SETPOINT;
volatile float elevatorDesiredVelocitySetpoint = DEFAULT_ELEVATOR_VELOCITY_SETPOINT;
volatile float aileronDesiredVelocitySetpoint  = DEFAULT_AILERON_VELOCITY_SETPOINT;


//auto-landing variables
volatile unsigned char landingState = 0x00;


//auto-trajectory variables
volatile unsigned char trajectoryEnabled = 0;
volatile float trajTimer=0;
volatile int8_t trajIndex = -1;
volatile int8_t trajMaxIndex = -1;
volatile trajectoryPoint_t trajectory[TRAJECTORY_LENGTH];

static uint8_t estimator_cycle = 0;
static float   estimatedThrottlePos_prev = 0;

//no leading data counter
volatile uint8_t neLeadCounter=0;

void leadingDataActualCheck(){
	if(leadingDataReceived>0){
		neLeadCounter=0;
	}else{
		if(neLeadCounter<(LEADING_DATA_TTL/DT)){
			neLeadCounter++;
		}else{

		}
	}
	leadingDataReceived=0;
}

void initTrajectory(){
	int8_t i=0;
	// (i, time (s), x (+ forward), y (+ leftward), z (altitude))
	for(i=0;i<TRAJECTORY_LENGTH;i++){
		TRAJ_POINT(i,1,elevatorPositionSetpoint,aileronPositionSetpoint,throttleSetpoint);
	}
	trajMaxIndex=-1;
}

void enableTrajectoryFollow(){
	if (trajectoryEnabled==0){
		trajIndex=-1;
	}
	trajectoryEnabled=1;
}

void disableTrajectoryFollow(){
	trajectoryEnabled=0;
}

void enableLanding(){	
	if(landingState==LANDING.FLIGHT || landingState==LANDING.TAKE_OFF){
		landingState=LANDING.STABILIZATION;
	}
}

void disableLanding(){	
	if(landingState!=LANDING.FLIGHT){
		landingState=LANDING.TAKE_OFF;
	}
}

void controllerSet(unsigned char controllerDesired){
	if(controllerDesired!=controllerActive){
		//NONE
		if(controllerDesired==CONTROLLERS.OFF){
			controllerActive=CONTROLLERS.OFF;			
		}else
		//VELOCITY
		if(controllerDesired==CONTROLLERS.VELOCITY){
			controllerActive=CONTROLLERS.VELOCITY;					
		}else
		//POSITION
		if(controllerDesired==CONTROLLERS.POSITION){		
			controllerActive=CONTROLLERS.POSITION;			
		}else
		//MPC
		if(controllerDesired==3){				
		}		
	}
}

void trajectorySetpoints(){
	float dValue;
	static float aileronIncrement;
	static float elevatorIncrement;
	static float throttleIncrement;
	
	if(trajIndex < trajMaxIndex){
		//if copter reaches desired position move to next waypoint
		if(trajIndex < 0 
		||(fabs(trajectory[trajIndex].throttlePos - estimatedThrottlePos) < 0.1 
		&& fabs(trajectory[trajIndex].elevatorPos - estimatedElevatorPos) < 0.1 
		&& fabs(trajectory[trajIndex].aileronPos  - estimatedAileronPos ) < 0.1)){
			trajIndex++;
			trajTimer=0;
			//calculate increments to setpoints
			dValue = (trajectory[trajIndex].elevatorPos - elevatorPositionSetpoint);
			elevatorIncrement = dValue / trajectory[trajIndex].time;
			dValue = (trajectory[trajIndex].aileronPos - aileronPositionSetpoint);
			aileronIncrement = dValue / trajectory[trajIndex].time;
			dValue = (trajectory[trajIndex].throttlePos - throttleSetpoint);
			throttleIncrement = dValue / trajectory[trajIndex].time;
		}			
		
		//increment setpoints
		if(trajTimer<trajectory[trajIndex].time){
			trajTimer+=DT;
			throttleSetpoint  += throttleIncrement*DT;
			elevatorPositionSetpoint += elevatorIncrement*DT;
			aileronPositionSetpoint  += aileronIncrement*DT;
		}else{
			throttleSetpoint = trajectory[trajIndex].throttlePos;
			elevatorPositionSetpoint = trajectory[trajIndex].elevatorPos;
			aileronPositionSetpoint = trajectory[trajIndex].aileronPos;
		}
	} 
}

void positionEstimator() {
	static float estimatedElevatorVel_Prev = 0;
	static float estimatedAileronVel_Prev  = 0;	
	float acc_new;
	
	//gustix valid delay - filters faulty values
	static uint8_t gumstix_counter = 0;
	const uint8_t gumstix_delay = 7;

	if(validGumstix == 1) {
		if(gumstix_counter < gumstix_delay) gumstix_counter++;
	} else {
		gumstix_counter = 0;
	}
	
	//Blob detector 
	estimatedBlobDistance += (elevatorGumstix-estimatedBlobDistance) * (DT/GUMSTIX_FILTER_CONST);
	estimatedBlobHorizontal += (aileronGumstix-estimatedBlobHorizontal) * (DT/GUMSTIX_FILTER_CONST);
	estimatedBlobVertical += (throttleGumstix-estimatedBlobVertical) * (DT/GUMSTIX_FILTER_CONST);
	
	//elevator velocity
	estimatedElevatorVel += (elevatorSpeed-estimatedElevatorVel) * (DT/PX4FLOW_FILTER_CONST);	
	//elevator acceleration
	acc_new = (estimatedElevatorVel - estimatedElevatorVel_Prev) / DT;
	estimatedElevatorVel_Prev = estimatedElevatorVel;
	estimatedElevatorAcc += (acc_new - estimatedElevatorAcc) * (DT/PX4FLOW_FILTER_CONST);
	//elevator position
	if(estimatedThrottlePos>ALTITUDE_MINIMUM){
		estimatedElevatorPos += estimatedElevatorVel * DT;	
	}

	//aileron velocity
	estimatedAileronVel += (aileronSpeed-estimatedAileronVel) * (DT/PX4FLOW_FILTER_CONST);	
	//aileron acceleration
	acc_new = (estimatedAileronVel - estimatedAileronVel_Prev) / DT;
	estimatedAileronVel_Prev = estimatedAileronVel;
	estimatedAileronAcc += (acc_new - estimatedAileronAcc) * (DT/PX4FLOW_FILTER_CONST);
	//aileron position
	if(estimatedThrottlePos>ALTITUDE_MINIMUM){
		estimatedAileronPos += estimatedAileronVel * DT;		
	}
}

void velocityController(int16_t *elevator, int16_t *aileron, float elevatorSetpoint, float aileronSetpoint) {
	float error;
	static const float KI=10, KV=25, KA=35;
	static int64_t time = 0;
	static float elevatorIntegration = 0;
	static float aileronIntegration = 0;
	
	//controller turned on
	if((secondsTimer-time)>1){
		elevatorIntegration=0;
		aileronIntegration=0;
		aileronDesiredVelocitySetpoint = DEFAULT_AILERON_VELOCITY_SETPOINT;
		elevatorDesiredVelocitySetpoint = DEFAULT_ELEVATOR_VELOCITY_SETPOINT;		
	}
	time=secondsTimer;	

	//setpoint filter
	elevatorVelocitySetpoint += (saturationFloat(elevatorSetpoint,SPEED_MAX)-elevatorVelocitySetpoint) * (DT/SETPOINT_FILTER_CONST);
	aileronVelocitySetpoint += (saturationFloat(aileronSetpoint,SPEED_MAX)-aileronVelocitySetpoint) * (DT/SETPOINT_FILTER_CONST);	
			
	//elevator controller		
	error = elevatorVelocitySetpoint - estimatedElevatorVel;
	elevatorIntegration +=saturationFloat(KI * error * DT,CONTROLLER_ELEVATOR_SATURATION/3.0);

	*elevator=saturationInt((int16_t)((KV * error) + elevatorIntegration - (KA * estimatedElevatorAcc)),CONTROLLER_ELEVATOR_SATURATION);

		
	//aileron controller
	error = aileronVelocitySetpoint - estimatedAileronVel;	
	aileronIntegration += saturationFloat(KI * error * DT,CONTROLLER_AILERON_SATURATION/3.0);

	*aileron=saturationInt((int16_t)((KV * error) + aileronIntegration - (KA * estimatedAileronAcc)),CONTROLLER_AILERON_SATURATION);
}

void positionController(int16_t *elevator, int16_t *aileron, float elevatorSetpoint, float aileronSetpoint) {
	static const float KI=5, KP=85, KV=180, KA=9;
    float speed;
	float error;
	
	static int64_t time = 0;
	static float elevatorIntegration = 0;
	static float aileronIntegration = 0;
	
	//controller turned on	
	if((secondsTimer-time)>1){
		elevatorIntegration=0;
		aileronIntegration=0;
		elevatorPositionSetpoint = DEFAULT_ELEVATOR_POSITION_SETPOINT;
		aileronPositionSetpoint = DEFAULT_AILERON_POSITION_SETPOINT;
		elevatorDesiredPositionSetpoint = DEFAULT_ELEVATOR_POSITION_SETPOINT;
		aileronDesiredPositionSetpoint = DEFAULT_AILERON_POSITION_SETPOINT;
		estimatedElevatorPos = elevatorPositionSetpoint;
		estimatedAileronPos  = aileronPositionSetpoint;		
	}
	time=secondsTimer;

	//setpoints filter
	elevatorPositionSetpoint += (elevatorSetpoint-elevatorPositionSetpoint) * (DT/SETPOINT_FILTER_CONST);
	aileronPositionSetpoint += (aileronSetpoint-aileronPositionSetpoint) * (DT/SETPOINT_FILTER_CONST);

	//elevator controller
	error = elevatorPositionSetpoint - estimatedElevatorPos;
	speed = saturationFloat((KP/KV) * error,SPEED_MAX);
	elevatorIntegration += saturationFloat(KI * error * DT,CONTROLLER_ELEVATOR_SATURATION/3.0);
	
	*elevator=saturationInt((int16_t)(KV * (speed - estimatedElevatorVel) + elevatorIntegration - (KA * estimatedElevatorAcc)),CONTROLLER_ELEVATOR_SATURATION);

	//aileron controller
	error = aileronPositionSetpoint - estimatedAileronPos;
	speed = saturationFloat((KP/KV) * error,SPEED_MAX);
	aileronIntegration += saturationFloat(KI * error * DT,CONTROLLER_AILERON_SATURATION/3.0);
	
	*aileron=saturationInt((int16_t)(KV * (speed - estimatedAileronVel) + aileronIntegration - (KA * estimatedAileronAcc)),CONTROLLER_AILERON_SATURATION);
}

void altitudeEstimator() {
   //new cycle 
   estimator_cycle++;
    if(groundDistance != estimatedThrottlePos_prev) {//input data changed
        // extreme filter
        if(fabs(groundDistance - estimatedThrottlePos_prev) <= 0.5) {//limitation cca 3m/s
           // compute new values 
           estimatedThrottleVel = (groundDistance - estimatedThrottlePos_prev) / (7*DT);
           estimatedThrottlePos      = groundDistance;
           estimatedThrottlePos_prev = groundDistance;
           estimator_cycle = 0;

        }
    } else {
        if (estimator_cycle >= 8) { //safety reset
             estimatedThrottleVel = 0;
             estimatedThrottlePos = groundDistance;
             estimatedThrottlePos_prev = groundDistance;
             estimator_cycle = 0;
        } else { //estimate position
            estimatedThrottlePos += estimatedThrottleVel * DT;
        }
    }
}

void altitudeController(int16_t *throttle,float setpoint) {
	float error;
	static const float KX=0.9, KI=120, KV=200;
	static int64_t time = 0;
	static float throttleIntegration = 0;
	
	//controller turned on	
	if((secondsTimer-time)>1){
		throttleIntegration=0;
		throttleSetpoint=DEFAULT_THROTTLE_SETPOINT;				
	}
	time=secondsTimer;	
	
	//setpoint filter
	if(setpoint<THROTTLE_SP_LOW){setpoint=THROTTLE_SP_LOW;}
	if(setpoint>THROTTLE_SP_HIGH){setpoint=THROTTLE_SP_HIGH;}
	throttleSetpoint += (setpoint-throttleSetpoint) * (DT/SETPOINT_FILTER_CONST);
	
	
	//throttle controller
	error =(throttleSetpoint - estimatedThrottlePos);
	throttleIntegration += saturationFloat(KI * error * DT,CONTROLLER_THROTTLE_SATURATION*2/3.0) ;
	*throttle=saturationInt((int16_t)(KV * (saturationFloat(KX * error,KX * error) - estimatedThrottleVel) + throttleIntegration),CONTROLLER_THROTTLE_SATURATION);
}

void landingController(int16_t *throttle, int16_t *elevator, int16_t *aileron){
	static int8_t landingCounter=0;
		if(landingState==LANDING.ON_GROUND){
			*throttle = -CONTROLLER_THROTTLE_SATURATION;
			*elevator = 0;
			*aileron = 0;
		}else
		if(landingState=LANDING.TAKE_OFF){
			altitudeController(throttle,throttleDesiredSetpoint);
			velocityController(elevator,aileron,0,0);				
			//stabilize altitude for 0.5s
			if(fabs(throttleSetpoint - estimatedThrottlePos) < 0.1 && fabs(estimatedThrottleVel) < 0.2){
				landingCounter++;
			}else{
				landingCounter = 0;
			}
			if(landingCounter >= 35){
				landingState =LANDING.FLIGHT;
			}
		}else	
		if(landingState=LANDING.STABILIZATION){
			//stabilize altitude for 0.5s
			altitudeController(throttle,ALTITUDE_MINIMUM);
			velocityController(elevator,aileron,0,0);
			
			if(fabs(throttleSetpoint - estimatedThrottlePos) < 0.1 && fabs(estimatedThrottleVel) < 0.1){
				landingCounter++;
			}else{
				landingCounter = 0;
			}
			if(landingCounter >= 35){
				landingState = LANDING.LANDING;					
			}
		}else
		if(landingState=LANDING.LANDING){
			*throttle=*throttle-5;
			*elevator=*elevator;
			*aileron=*aileron;
			if(*throttle <= -CONTROLLER_THROTTLE_SATURATION){
				landingState = LANDING.ON_GROUND;
			}			
		}
}
