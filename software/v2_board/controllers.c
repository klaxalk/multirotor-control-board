#include "controllers.h"
#include "communication.h"

#include "constants.h"

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

volatile float estimatedBlobElevator = 0;
volatile float estimatedBlobAileron = 0;
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

//Blob 
volatile unsigned char gumstixStable=0;
volatile float blobElevatorDeflection=0;
volatile float blobAileronDeflection=0;
volatile float lockOnBlobDistance = -1; 

//Blob Deflection timer
volatile uint16_t leadFreshTimer;

//landing variables
volatile unsigned char landingState = 0x00;



//trajectory variables
volatile int8_t trajIndex = 0;
volatile int8_t trajMaxIndex = 0;
volatile trajectoryPoint_t trajectory[TRAJECTORY_LENGTH];

 
void initTrajectory(){
	uint32_t i=0;
	// (i, time (s), x (+ forward), y (+ leftward), z (altitude))
	for(i=0;i<TRAJECTORY_LENGTH;i++){
		TRAJ_POINT(i,secondsTimer+1,0,0,1);
	}
	trajMaxIndex=-1;
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

void enableLockOnBlob(float distance){
	lockOnBlobDistance=distance;
}

void disableLockOnBlob(){
	lockOnBlobDistance=-1;
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

void setpointsCalculate(){
	float distLeft,speed,timeLeft;
	
	//trajectory waypoint shift
	while((secondsTimer>=trajectory[trajIndex].time) && (trajIndex<trajMaxIndex)){
		trajIndex++;
	}
		
	//end of trajectory	
	if(secondsTimer>=trajectory[trajIndex].time){
		elevatorDesiredPositionSetpoint=trajectory[trajIndex].elevatorPos;	
		aileronDesiredPositionSetpoint=trajectory[trajIndex].aileronPos;	
			
		elevatorDesiredVelocitySetpoint=0;
		aileronDesiredVelocitySetpoint=0;	
	}else{
	//setpoints calculation			
		timeLeft=(float)(trajectory[trajIndex].time-secondsTimer)-(milisecondsTimer/1000.0);	
		//elevator
		distLeft=trajectory[trajIndex].elevatorPos-elevatorDesiredPositionSetpoint;		
		speed=distLeft/timeLeft;
		elevatorDesiredPositionSetpoint+=speed*DT;
		elevatorDesiredVelocitySetpoint=speed;
		//aileron
		distLeft=trajectory[trajIndex].aileronPos-aileronDesiredPositionSetpoint;
		speed=distLeft/timeLeft;
		aileronDesiredPositionSetpoint+=speed*DT;		
		aileronDesiredVelocitySetpoint=speed;	
		//throttle
		distLeft=trajectory[trajIndex].throttlePos-throttleDesiredSetpoint;
		speed=distLeft/timeLeft;
		throttleDesiredSetpoint+=speed*DT;			
	}
}

void positionEstimator() {
	static float estimatedElevatorVel_Prev = 0;
	static float estimatedAileronVel_Prev  = 0;	
	float accNew;
	
	//gustix valid delay - filters faulty values
	static uint8_t gumstixCounter = 0;
	const uint8_t gumstixDelay = 7;

	if(validGumstix == 1) {
		if(gumstixCounter < gumstixDelay){ 
			gumstixCounter++;
		}else{
			gumstixStable=1;		
		}
	} else {
		gumstixCounter = 0;
		gumstixStable=0;
	}
	

	if(gumstixStable==1){
		//Blob detector
		estimatedBlobElevator += (elevatorGumstix-estimatedBlobElevator) * (DT/GUMSTIX_FILTER_CONST);
		estimatedBlobAileron += (aileronGumstix-estimatedBlobAileron) * (DT/GUMSTIX_FILTER_CONST);
		estimatedBlobVertical += (throttleGumstix-estimatedBlobVertical) * (DT/GUMSTIX_FILTER_CONST);				
	}
	
	//elevator velocity
	estimatedElevatorVel += (elevatorSpeed-estimatedElevatorVel) * (DT/PX4FLOW_FILTER_CONST);	
	//elevator acceleration
	accNew = (estimatedElevatorVel - estimatedElevatorVel_Prev) / DT;
	estimatedElevatorVel_Prev = estimatedElevatorVel;
	estimatedElevatorAcc += (accNew - estimatedElevatorAcc) * (DT/PX4FLOW_FILTER_CONST);
	//elevator position
	if(estimatedThrottlePos>ALTITUDE_MINIMUM){
		estimatedElevatorPos += estimatedElevatorVel * DT;	
	}

	//aileron velocity
	estimatedAileronVel += (aileronSpeed-estimatedAileronVel) * (DT/PX4FLOW_FILTER_CONST);	
	//aileron acceleration
	accNew = (estimatedAileronVel - estimatedAileronVel_Prev) / DT;
	estimatedAileronVel_Prev = estimatedAileronVel;
	estimatedAileronAcc += (accNew - estimatedAileronAcc) * (DT/PX4FLOW_FILTER_CONST);
	//aileron position
	if(estimatedThrottlePos>ALTITUDE_MINIMUM){
		estimatedAileronPos += estimatedAileronVel * DT;		
	}
}

void velocityController(float elevatorSetpoint, float aileronSetpoint) {
	float error;
	static const float KI=10, KV=25, KA=35;
	static uint32_t time = 0;
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
	elevatorIntegration = saturationFloat(elevatorIntegration+(KI * error * DT),CONTROLLER_ELEVATOR_SATURATION/3.0);
	controllerElevatorOutput=saturationInt16((int16_t)((KV * error) + elevatorIntegration - (KA * estimatedElevatorAcc)),CONTROLLER_ELEVATOR_SATURATION);

		
	//aileron controller
	error = aileronVelocitySetpoint - estimatedAileronVel;	
	aileronIntegration = saturationFloat(aileronIntegration+(KI * error * DT),CONTROLLER_AILERON_SATURATION/3.0);
	controllerAileronOutput=saturationInt16((int16_t)((KV * error) + aileronIntegration - (KA * estimatedAileronAcc)),CONTROLLER_AILERON_SATURATION);
}

void positionController(float elevatorSetpoint, float aileronSetpoint) {
	static const float KI=5, KP=85, KV=180, KA=9;
    float speed;
	float error;
	
	static uint32_t time = 0;
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
	elevatorIntegration = saturationFloat(elevatorIntegration+(KI * error * DT),CONTROLLER_ELEVATOR_SATURATION/3.0);	
	controllerElevatorOutput=saturationInt16((int16_t)(KV * (speed - estimatedElevatorVel) + elevatorIntegration - (KA * estimatedElevatorAcc)),CONTROLLER_ELEVATOR_SATURATION);

	//aileron controller
	error = aileronPositionSetpoint - estimatedAileronPos;
	speed = saturationFloat((KP/KV) * error,SPEED_MAX);
	aileronIntegration = saturationFloat(aileronIntegration+(KI * error * DT),CONTROLLER_AILERON_SATURATION/3.0);	
	controllerAileronOutput=saturationInt16((int16_t)(KV * (speed - estimatedAileronVel) + aileronIntegration - (KA * estimatedAileronAcc)),CONTROLLER_AILERON_SATURATION);
}

void altitudeEstimator() {
static uint8_t estimator_cycle = 0;
static float   estimatedThrottlePos_prev = 0;	
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

void altitudeController(float setpoint) {
	float error;
	//static const float KX=0.9, KI=120, KV=200;
	static const float KX=0.9, KI=160, KV=200;
	static uint32_t time = 0;
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
	throttleIntegration = saturationFloat(throttleIntegration+(KI * error * DT),CONTROLLER_THROTTLE_SATURATION*(1.8/3.0)) ;
	controllerThrottleOutput=saturationInt16((int16_t)(KV * (saturationFloat(KX * error,ALTITUDE_SPEED_MAX) - estimatedThrottleVel) + throttleIntegration),CONTROLLER_THROTTLE_SATURATION);
}

void landingController(){
	static int8_t landingCounter=0;
		if(landingState==LANDING.ON_GROUND){
			controllerThrottleOutput = -CONTROLLER_THROTTLE_SATURATION;
			controllerElevatorOutput = 0;
			controllerAileronOutput = 0;
		}else
		if(landingState==LANDING.TAKE_OFF){
			altitudeController(throttleDesiredSetpoint);
			velocityController(0,0);				
			//stabilize altitude for 0.5s
			if(fabs(throttleSetpoint - estimatedThrottlePos) < 0.1 && fabs(estimatedThrottleVel) < 0.2){
				landingCounter++;
			}else{
				landingCounter = 0;
			}
			if(landingCounter >= 35){
				landingState=LANDING.FLIGHT;
				landingCounter=0;
			}
		}else	
		if(landingState==LANDING.STABILIZATION){
			//stabilize altitude for 0.5s
			altitudeController(ALTITUDE_MINIMUM);
			velocityController(0,0);
			
			if(fabs(throttleSetpoint - estimatedThrottlePos) < 0.1 && fabs(estimatedThrottleVel) < 0.2){
				landingCounter++;
			}else{
				landingCounter = 0;
			}
			if(landingCounter >= 35){
				landingState = LANDING.LANDING;	
				landingCounter=0;				
			}
		}else
		if(landingState==LANDING.LANDING){
			controllerThrottleOutput-=5;
			if(controllerThrottleOutput <= -CONTROLLER_THROTTLE_SATURATION){
				landingState = LANDING.ON_GROUND;
			}			
		}
}

void lockOnBlob(float distance){	
	float error;
	float drift;
	
	if(gumstixStable){
		error=estimatedElevatorPos-elevatorPositionSetpoint;
		drift=estimatedBlobElevator-blobElevatorDeflection-distance+error;		
		estimatedElevatorPos+=(-drift)*(DT/GUMSTIX_FILTER_CONST);
		
		error=estimatedAileronPos-aileronPositionSetpoint;
		drift=estimatedBlobAileron-blobAileronDeflection+error;
		estimatedAileronPos+=(-drift)*(DT/GUMSTIX_FILTER_CONST);
	}	
}
