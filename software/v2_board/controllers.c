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
volatile state_t position = {.aileron=0 , .elevator=0 , .altitude=0};
volatile state_t speed = {.aileron=0 , .elevator=0 , .altitude=0};
volatile state_t acceleration ={.aileron=0 , .elevator=0 , .altitude=0};
volatile state_t blob = {.aileron=0 , .elevator=0 , .altitude=0};		


//setpoints
volatile state_t positionSetpoint = {.aileron=DEFAULT_AILERON_POSITION_SETPOINT , .elevator=DEFAULT_ELEVATOR_POSITION_SETPOINT , .altitude=DEFAULT_THROTTLE_POSITION_SETPOINT};
volatile state_t speedSetpoint = {.aileron=DEFAULT_AILERON_VELOCITY_SETPOINT , .elevator=DEFAULT_ELEVATOR_VELOCITY_SETPOINT , .altitude=DEFAULT_THROTTLE_VELOCITY_SETPOINT};
volatile state_t positionDesiredSetpoint = {.aileron=DEFAULT_AILERON_POSITION_SETPOINT , .elevator=DEFAULT_ELEVATOR_POSITION_SETPOINT , .altitude=DEFAULT_THROTTLE_POSITION_SETPOINT};
volatile state_t speedDesiredSetpoint = {.aileron=DEFAULT_AILERON_VELOCITY_SETPOINT , .elevator=DEFAULT_ELEVATOR_VELOCITY_SETPOINT , .altitude=DEFAULT_THROTTLE_VELOCITY_SETPOINT};	


//Blob 
volatile unsigned char gumstixStable=0;


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
		if(controllerDesired==CONTROLLERS.MPC){				
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
		positionDesiredSetpoint.elevator=trajectory[trajIndex].elevatorPos;	
		positionDesiredSetpoint.aileron=trajectory[trajIndex].aileronPos;	
			
		speedDesiredSetpoint.elevator=0;
		speedDesiredSetpoint.aileron=0;	
	}else{
	//setpoints calculation			
		timeLeft=(float)(trajectory[trajIndex].time-secondsTimer)-(milisecondsTimer/1000.0);	
		//elevator
		distLeft=trajectory[trajIndex].elevatorPos-positionDesiredSetpoint.elevator;		
		speed=distLeft/timeLeft;
		positionDesiredSetpoint.elevator+=speed*DT;
		speedDesiredSetpoint.elevator=speed;
		//aileron
		distLeft=trajectory[trajIndex].aileronPos-positionDesiredSetpoint.aileron;
		speed=distLeft/timeLeft;
		positionDesiredSetpoint.aileron+=speed*DT;		
		speedDesiredSetpoint.aileron=speed;	
		//throttle
		distLeft=trajectory[trajIndex].throttlePos-positionDesiredSetpoint.altitude;
		speed=distLeft/timeLeft;
		positionDesiredSetpoint.altitude+=speed*DT;	
		speedDesiredSetpoint.altitude=speed;		
	}
}

void positionEstimator() {
	static state_t speedPrev = {.aileron = 0 , .elevator = 0 , .altitude = 0};
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
		blob.elevator += (elevatorGumstix-blob.elevator) * (DT/GUMSTIX_FILTER_CONST);
		blob.aileron += (aileronGumstix-blob.aileron) * (DT/GUMSTIX_FILTER_CONST);
		blob.altitude += (throttleGumstix-blob.altitude) * (DT/GUMSTIX_FILTER_CONST);				
	}
	
	//elevator velocity
	speed.elevator += (elevatorSpeed-speed.elevator) * (DT/PX4FLOW_FILTER_CONST);	
	//elevator acceleration
	accNew = (speed.elevator - speedPrev.elevator) / DT;
	speedPrev.elevator = speed.elevator;
	acceleration.elevator += (accNew - acceleration.elevator) * (DT/PX4FLOW_FILTER_CONST);
	//elevator position
	if(position.altitude>ALTITUDE_MINIMUM){
		position.elevator += speed.elevator * DT;	
	}

	//aileron velocity
	speed.aileron += (aileronSpeed-speed.aileron) * (DT/PX4FLOW_FILTER_CONST);	
	//aileron acceleration
	accNew = (speed.aileron - speedPrev.aileron) / DT;
	speedPrev.aileron = speed.aileron;
	acceleration.aileron += (accNew - acceleration.aileron) * (DT/PX4FLOW_FILTER_CONST);
	//aileron position
	if(position.altitude>ALTITUDE_MINIMUM){
		position.aileron += speed.aileron * DT;		
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
		speedDesiredSetpoint.aileron = DEFAULT_AILERON_VELOCITY_SETPOINT;
		speedDesiredSetpoint.elevator = DEFAULT_ELEVATOR_VELOCITY_SETPOINT;		
	}
	time=secondsTimer;	

	//setpoint filter
	speedSetpoint.elevator += (saturationFloat(elevatorSetpoint,SPEED_MAX)-speedSetpoint.elevator) * (DT/SETPOINT_FILTER_CONST);
	speedSetpoint.aileron += (saturationFloat(aileronSetpoint,SPEED_MAX)-speedSetpoint.aileron) * (DT/SETPOINT_FILTER_CONST);	
			
	//elevator controller		
	error = speedSetpoint.elevator - speed.elevator;
	elevatorIntegration = saturationFloat(elevatorIntegration+(KI * error * DT),CONTROLLER_ELEVATOR_SATURATION/3.0);
	controllerElevatorOutput=saturationInt16((int16_t)((KV * error) + elevatorIntegration - (KA * acceleration.elevator)),CONTROLLER_ELEVATOR_SATURATION);

		
	//aileron controller
	error = speedSetpoint.aileron - speed.aileron;	
	aileronIntegration = saturationFloat(aileronIntegration+(KI * error * DT),CONTROLLER_AILERON_SATURATION/3.0);
	controllerAileronOutput=saturationInt16((int16_t)((KV * error) + aileronIntegration - (KA * acceleration.aileron)),CONTROLLER_AILERON_SATURATION);
}

void positionController(float elevatorSetpoint, float aileronSetpoint) {
	static const float KI=5, KP=85, KV=180, KA=9;
    float speedDes;
	float error;
	
	static uint32_t time = 0;
	static float elevatorIntegration = 0;
	static float aileronIntegration = 0;
	
	//controller turned on	
	if((secondsTimer-time)>1){
		elevatorIntegration=0;
		aileronIntegration=0;
		positionSetpoint.elevator = DEFAULT_ELEVATOR_POSITION_SETPOINT;
		positionSetpoint.aileron = DEFAULT_AILERON_POSITION_SETPOINT;
		positionDesiredSetpoint.elevator = DEFAULT_ELEVATOR_POSITION_SETPOINT;
		positionDesiredSetpoint.aileron = DEFAULT_AILERON_POSITION_SETPOINT;
		position.elevator = positionSetpoint.elevator;
		position.aileron  = positionSetpoint.aileron;		
	}
	time=secondsTimer;

	//setpoints filter
	positionSetpoint.elevator += (elevatorSetpoint-positionSetpoint.elevator) * (DT/SETPOINT_FILTER_CONST);
	positionSetpoint.aileron += (aileronSetpoint-positionSetpoint.aileron) * (DT/SETPOINT_FILTER_CONST);

	//elevator controller
	error = positionSetpoint.elevator - position.elevator;
	speedDes = saturationFloat((KP/KV) * error,SPEED_MAX);
	elevatorIntegration = saturationFloat(elevatorIntegration+(KI * error * DT),CONTROLLER_ELEVATOR_SATURATION/3.0);	
	controllerElevatorOutput=saturationInt16((int16_t)(KV * (speedDes - speed.elevator) + elevatorIntegration - (KA * acceleration.elevator)),CONTROLLER_ELEVATOR_SATURATION);

	//aileron controller
	error = positionSetpoint.aileron - position.aileron;
	speedDes = saturationFloat((KP/KV) * error,SPEED_MAX);
	aileronIntegration = saturationFloat(aileronIntegration+(KI * error * DT),CONTROLLER_AILERON_SATURATION/3.0);	
	controllerAileronOutput=saturationInt16((int16_t)(KV * (speedDes - speed.aileron) + aileronIntegration - (KA * acceleration.aileron)),CONTROLLER_AILERON_SATURATION);
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
           speed.altitude = (groundDistance - estimatedThrottlePos_prev) / (7*DT);
           position.altitude = groundDistance;
           estimatedThrottlePos_prev = groundDistance;
           estimator_cycle = 0;

        }
    } else {
        if (estimator_cycle >= 8) { //safety reset
             speed.altitude = 0;
             position.altitude = groundDistance;
             estimatedThrottlePos_prev = groundDistance;
             estimator_cycle = 0;
        } else { //estimate position
            position.altitude += speed.altitude * DT;
        }
    }
}

void altitudeController(float setpoint) {
	float error;
	static const float KX=0.9, KI=130, KV=200;
	static uint32_t time = 0;
	static float throttleIntegration = 0;
	
	//controller turned on	
	if((secondsTimer-time)>1){
		throttleIntegration=0;
		positionSetpoint.altitude=DEFAULT_THROTTLE_POSITION_SETPOINT;	
		positionDesiredSetpoint.altitude=DEFAULT_THROTTLE_POSITION_SETPOINT;
	}
	time=secondsTimer;	
	
	//setpoint filter
	if(setpoint<THROTTLE_SP_LOW){setpoint=THROTTLE_SP_LOW;}
	if(setpoint>THROTTLE_SP_HIGH){setpoint=THROTTLE_SP_HIGH;}
	positionSetpoint.altitude += (setpoint-positionSetpoint.altitude) * (DT/SETPOINT_FILTER_CONST);
		
	//throttle controller
	error =(positionSetpoint.altitude - position.altitude);
	throttleIntegration = saturationFloat(throttleIntegration+(KI * error * DT),CONTROLLER_THROTTLE_SATURATION*(1.8/3.0)) ;
	controllerThrottleOutput=saturationInt16((int16_t)(KV * (saturationFloat(KX * error,ALTITUDE_SPEED_MAX) - speed.altitude) + throttleIntegration),CONTROLLER_THROTTLE_SATURATION);
}

void landingController(){
	static int8_t landingCounter=0;
		if(landingState==LANDING.ON_GROUND){
			controllerThrottleOutput = -CONTROLLER_THROTTLE_SATURATION;
			controllerElevatorOutput = 0;
			controllerAileronOutput = 0;
		}else
		if(landingState==LANDING.TAKE_OFF){
			altitudeController(positionDesiredSetpoint.altitude);
			velocityController(0,0);				
			//stabilize altitude for 0.5s
			if(fabs(positionSetpoint.altitude - position.altitude) < 0.1 && fabs(speed.altitude) < 0.2){
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
			
			if(fabs(positionSetpoint.altitude - position.altitude) < 0.1 && fabs(speed.altitude) < 0.2){
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

