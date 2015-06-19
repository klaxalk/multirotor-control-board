#include "controllers.h"
#include "communication.h"
#include "constants.h"
#include "mpcHandler.h"

// controllers output variables
volatile int16_t controllerElevatorOutput=0;
volatile int16_t controllerAileronOutput=0;
volatile int16_t controllerRudderOutput=0;
volatile int16_t controllerThrottleOutput=0;

// controller select
volatile unsigned char controllerActive = 0x01;

//vars for estimators
volatile state_t positionShift = {.aileron=0 , .elevator=0 , .altitude=0};
volatile state2_t altitude = {.position=0, .speed=0, .acceleration=0};
	
/*
Transformace z SS MAV a Global
akt=kopt+shift
kopt=akt-shift
*/		

//landing variables
volatile unsigned char landingState = 0x04;

//trajectory variables
volatile int8_t trajMaxIndex = 0;
volatile trajectoryPoint_t trajectory[TRAJECTORY_LENGTH];
volatile state_t setpoints={.aileron=DEFAULT_AILERON_POSITION_SETPOINT, .elevator=DEFAULT_ELEVATOR_POSITION_SETPOINT, .altitude=DEFAULT_THROTTLE_POSITION_SETPOINT};
volatile char trajSend = 0;

 
void initTrajectory(){
	uint32_t i=0;
	// (i, time (s), x (+ forward), y (+ leftward), z (altitude))
	for(i=0;i<TRAJECTORY_LENGTH;i++){
		TRAJ_POINT(i,0,DEFAULT_ELEVATOR_POSITION_SETPOINT,DEFAULT_AILERON_POSITION_SETPOINT,DEFAULT_THROTTLE_POSITION_SETPOINT);
	}
	trajMaxIndex=0;
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
			controllerActive=CONTROLLERS.MPC;	
			stmResetKalman(0,0);
			stmResetKalman(0,0);
			stmResetKalman(0,0);		
			positionShift.elevator=0;
			positionShift.aileron=0;		
		}		
	}
}


void setpointsCalculate(){
	//actual setpoint in kalman coordinate system
	int8_t index = 0;
	
	float distLeft,speed,timeLeft;
	
		//trajectory waypoint shift
		while((secondsTimer>=trajectory[index].time) && (index<trajMaxIndex)){
			index++;
		}
		
		//end of trajectory
		if(secondsTimer>=trajectory[index].time){
			setpoints.elevator=trajectory[index].elevatorPos;
			setpoints.aileron=trajectory[index].aileronPos;
			setpoints.altitude=trajectory[index].throttlePos;
		}else{
			//setpoints calculation
			timeLeft=(float)(trajectory[index].time-secondsTimer)-(milisecondsTimer/1000.0);
			//elevator
			distLeft=trajectory[index].elevatorPos-setpoints.elevator;
			speed=distLeft/timeLeft;
			setpoints.elevator+=speed*DT;
			//aileron
			distLeft=trajectory[index].aileronPos-setpoints.aileron;
			speed=distLeft/timeLeft;		
			setpoints.aileron+=speed*DT;
			//throttle
			distLeft=trajectory[index].throttlePos-setpoints.altitude;
			speed=distLeft/timeLeft;						
			setpoints.altitude+=speed*DT;			
		}						
	trajSend=1;		
}


void velocityController() {
	float error;
	static const float KI=10, KV=25, KA=35;
	static uint32_t time = 0;
	static float elevatorIntegration = 0;
	static float aileronIntegration = 0;
	
	//controller turned on
	if((secondsTimer-time)>1){
		elevatorIntegration=0;
		aileronIntegration=0;
	}
	time=secondsTimer;	
			
	//elevator controller		
	error = -kalmanStates.elevator.velocity;
	elevatorIntegration = saturationFloat(elevatorIntegration+(KI * error * DT),CONTROLLER_ELEVATOR_SATURATION/3.0);
	controllerElevatorOutput=saturationInt16((int16_t)((KV * error) + elevatorIntegration - (KA * kalmanStates.elevator.acceleration)),CONTROLLER_ELEVATOR_SATURATION);
	controllerElevatorOutput*=3;
		
	//aileron controller
	error = -kalmanStates.aileron.velocity;
	aileronIntegration = saturationFloat(aileronIntegration+(KI * error * DT),CONTROLLER_AILERON_SATURATION/3.0);
	controllerAileronOutput=saturationInt16((int16_t)((KV * error) + aileronIntegration - (KA * kalmanStates.aileron.acceleration)),CONTROLLER_AILERON_SATURATION);
	controllerAileronOutput*=3;
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
	}
	time=secondsTimer;

	//elevator controller
	error = elevatorSetpoint - kalmanStates.elevator.position;
	speedDes = saturationFloat((KP/KV) * error,SPEED_MAX);
	elevatorIntegration = saturationFloat(elevatorIntegration+(KI * error * DT),CONTROLLER_ELEVATOR_SATURATION/3.0);	
	controllerElevatorOutput=saturationInt16((int16_t)(KV * (speedDes - kalmanStates.elevator.velocity) + elevatorIntegration - (KA * kalmanStates.elevator.acceleration)),CONTROLLER_ELEVATOR_SATURATION);
	controllerElevatorOutput*=3;

	//aileron controller
	error = aileronSetpoint - kalmanStates.aileron.position;
	speedDes = saturationFloat((KP/KV) * error,SPEED_MAX);
	aileronIntegration = saturationFloat(aileronIntegration+(KI * error * DT),CONTROLLER_AILERON_SATURATION/3.0);	
	controllerAileronOutput=saturationInt16((int16_t)(KV * (speedDes - kalmanStates.aileron.velocity) + aileronIntegration - (KA * kalmanStates.aileron.acceleration)),CONTROLLER_AILERON_SATURATION);
	controllerAileronOutput*=3;
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
           altitude.speed = (groundDistance - estimatedThrottlePos_prev) / (7*DT);
           altitude.position = groundDistance;
           estimatedThrottlePos_prev = groundDistance;
           estimator_cycle = 0;

        }
    } else {
        if (estimator_cycle >= 8) { //safety reset
             altitude.speed = 0;
             altitude.position = groundDistance;
             estimatedThrottlePos_prev = groundDistance;
             estimator_cycle = 0;
        } else { //estimate position
            altitude.position += altitude.speed * DT;
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
	}
	time=secondsTimer;		
		
	//throttle controller
	error =(setpoint - altitude.position);
	throttleIntegration = saturationFloat(throttleIntegration+(KI * error * DT),CONTROLLER_THROTTLE_SATURATION*(1.8/3.0)) ;
	controllerThrottleOutput=saturationInt16((int16_t)(KV * (saturationFloat(KX * error,ALTITUDE_SPEED_MAX) - altitude.speed) + throttleIntegration),CONTROLLER_THROTTLE_SATURATION);
}

void landingController(){
	static int8_t landingCounter=0;
		if(landingState==LANDING.ON_GROUND){
			controllerThrottleOutput = -CONTROLLER_THROTTLE_SATURATION;
			controllerElevatorOutput = 0;
			controllerAileronOutput = 0;
		}else
		if(landingState==LANDING.TAKE_OFF){
			altitudeController(setpoints.altitude);
			velocityController();				
			//stabilize altitude for 0.5s
			if(fabs(setpoints.altitude - altitude.position) < 0.1 && fabs(altitude.speed) < 0.2){
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
			velocityController();
			
			if(fabs(setpoints.altitude - altitude.position) < 0.1 && fabs(altitude.speed) < 0.2){
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

