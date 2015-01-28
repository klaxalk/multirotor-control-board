#include "controllers.h"
#include "communication.h"

#include "packets.h"
#include "commands.h"

// controllers output variables
volatile int16_t velocityControllerElevatorOutput;
volatile int16_t velocityControllerAileronOutput;
volatile int16_t positionControllerElevatorOutput;
volatile int16_t positionControllerAileronOutput;

volatile int16_t controllerThrottleOutput;
volatile int16_t controllerRudderOutput;

//vars for estimators
volatile float estimatedElevatorPos = 0;
volatile float estimatedAileronPos  = 0;
volatile float estimatedThrottlePos = 0;

volatile float estimatedElevatorVel = 0;
volatile float estimatedElevatorVel2 = 0;
volatile float estimatedAileronVel  = 0;
volatile float estimatedAileronVel2 = 0;
volatile float estimatedThrottleVel = 0;
volatile float estimatedElevatorVel_Prev = 0;
volatile float estimatedAileronVel_Prev  = 0;

volatile float estimatedElevatorAcc = 0;
volatile float estimatedAileronAcc = 0;

volatile float estimatedBlobDistance = 0;
volatile float estimatedBlobHorizontal = 0;
volatile float estimatedBlobVertical = 0;

//vars for controllers
volatile float elevatorPositionIntegration = 0;
volatile float aileronPositionIntegration  = 0;
volatile float elevatorVelocityIntegration = 0;
volatile float aileronVelocityIntegration  = 0;
volatile float throttleIntegration = 0;

volatile float elevatorDesiredSpeedPosController=0;
volatile float aileronDesiredSpeedPosController=0;
volatile float elevatorDesiredSpeedPosControllerLeader=0;
volatile float aileronDesiredSpeedPosControllerLeader=0;

volatile float elevatorPosContError=0;
volatile float aileronPosContError=0;

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
volatile unsigned char landingRequest = 0;
volatile unsigned char landingState = LS_ON_GROUND;
volatile uint8_t landingCounter = 0;
volatile float landingThrottleSetpoint = 0;
volatile int16_t landingThrottleOutput;

// controller on/off
volatile unsigned char velocityControllerEnabled = 0;
volatile unsigned char positionControllerEnabled = 0;

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
			elevatorDesiredSpeedPosControllerLeader=0;
			aileronDesiredSpeedPosControllerLeader=0;
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
	landingRequest=1;
}

void disableLanding(){	
	landingRequest=0;
}


void enableVelocityController() {
	//if both controllers was off then set altitude controller to default
	if(velocityControllerEnabled==0 && positionControllerEnabled==0){
		throttleIntegration = 0;
		throttleDesiredSetpoint = DEFAULT_THROTTLE_SETPOINT;
	}
	//set controller to default
	if (velocityControllerEnabled == 0) {
		elevatorVelocityIntegration = 0;
		aileronVelocityIntegration = 0;		
						
		aileronDesiredVelocitySetpoint = DEFAULT_AILERON_VELOCITY_SETPOINT;
		elevatorDesiredVelocitySetpoint = DEFAULT_ELEVATOR_VELOCITY_SETPOINT;
	}
	velocityControllerEnabled = 1;
}

void disableVelocityController() {
	velocityControllerEnabled = 0;
}

void enablePositionController() {
	//if both controllers was off then set altitude controller to default
	if(velocityControllerEnabled==0 && positionControllerEnabled==0){
		throttleIntegration = 0;
		throttleDesiredSetpoint = DEFAULT_THROTTLE_SETPOINT;
	}	
	//set controller to default
	if (positionControllerEnabled == 0) {
		elevatorPositionIntegration = 0;
		aileronPositionIntegration = 0;
		
		elevatorPositionSetpoint = DEFAULT_ELEVATOR_POSITION_SETPOINT;
		aileronPositionSetpoint = DEFAULT_AILERON_POSITION_SETPOINT;
		elevatorDesiredPositionSetpoint = DEFAULT_ELEVATOR_POSITION_SETPOINT;
		aileronDesiredPositionSetpoint = DEFAULT_AILERON_POSITION_SETPOINT;
		
		estimatedElevatorPos = elevatorPositionSetpoint;
		estimatedAileronPos  = aileronPositionSetpoint;
	}
	positionControllerEnabled = 1;
}

void disablePositionController() {
	positionControllerEnabled = 0;
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

void setpointsFilter(float throttleDSP,float aileronPosDSP,float elevatorPosDSP,float aileronVelDSP,float elevatorVelDSP) {	
	//safety saturation
	if(throttleDSP<THROTTLE_SP_LOW){throttleDSP=THROTTLE_SP_LOW;}
	if(throttleDSP>THROTTLE_SP_HIGH){throttleDSP=THROTTLE_SP_HIGH;}
		
	if(elevatorVelDSP<-SPEED_MAX){elevatorVelDSP=-SPEED_MAX;}
	if(elevatorVelDSP>SPEED_MAX){elevatorVelDSP=SPEED_MAX;}
		
	if(aileronVelDSP<-SPEED_MAX){aileronVelDSP=-SPEED_MAX;}
	if(aileronVelDSP>SPEED_MAX){aileronVelDSP=SPEED_MAX;}
			
	//filter	
	elevatorPositionSetpoint += (elevatorPosDSP-elevatorPositionSetpoint) * (DT/SETPOINT_FILTER_CONST);
	aileronPositionSetpoint += (aileronPosDSP-aileronPositionSetpoint) * (DT/SETPOINT_FILTER_CONST);
	
	throttleSetpoint += (throttleDSP-throttleSetpoint) * (DT/SETPOINT_FILTER_CONST);
	
	elevatorVelocitySetpoint += (elevatorVelDSP-elevatorVelocitySetpoint) * (DT/SETPOINT_FILTER_CONST);
	aileronVelocitySetpoint += (aileronVelDSP-aileronVelocitySetpoint) * (DT/SETPOINT_FILTER_CONST);			
}

void positionEstimator() {
	//help variable for acceleration
	float acc_new;
	
	//gustix valid delay - filters faulty values
	static uint8_t gumstix_counter = 0;
	uint8_t gumstix_delay = 7;

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
	estimatedElevatorVel2 += (estimatedElevatorVel-estimatedElevatorVel2)* (DT/(PX4FLOW_FILTER_CONST+0.02));
	
	//elevator acceleration
	acc_new = (estimatedElevatorVel2 - estimatedElevatorVel_Prev) / DT;
	estimatedElevatorVel_Prev = estimatedElevatorVel2;
	estimatedElevatorAcc += (acc_new - estimatedElevatorAcc) * (DT/PX4FLOW_FILTER_CONST);

	//elevator position
	if(estimatedThrottlePos>ALTITUDE_MINIMUM){
		estimatedElevatorPos += estimatedElevatorVel * DT;	
	}

	//aileron velocity
	estimatedAileronVel += (aileronSpeed-estimatedAileronVel) * (DT/PX4FLOW_FILTER_CONST);
	estimatedAileronVel2 += (estimatedAileronVel-estimatedAileronVel2) * (DT/PX4FLOW_FILTER_CONST+0.02);
	
	//aileron acceleration
	acc_new = (estimatedAileronVel2 - estimatedAileronVel_Prev) / DT;
	estimatedAileronVel_Prev = estimatedAileronVel2;
	estimatedAileronAcc += (acc_new - estimatedAileronAcc) * (DT/PX4FLOW_FILTER_CONST);

	//aileron position
	if(estimatedThrottlePos>ALTITUDE_MINIMUM){
		estimatedAileronPos += estimatedAileronVel * DT;		
	}
}

void velocityController() {
	float error;
	float KI, KV, KA;

	//set controller constants
	KI = VELOCITY_KI;
	KV = VELOCITY_KV;
	KA = VELOCITY_KA;

	//velocity controller
	error = elevatorVelocitySetpoint - estimatedElevatorVel2;

	elevatorVelocityIntegration += KI * error * DT;
	if (elevatorVelocityIntegration > CONTROLLER_ELEVATOR_SATURATION/3) {elevatorVelocityIntegration = CONTROLLER_ELEVATOR_SATURATION/3;} else
	if (elevatorVelocityIntegration < -CONTROLLER_ELEVATOR_SATURATION/3) {elevatorVelocityIntegration = -CONTROLLER_ELEVATOR_SATURATION/3;}

	velocityControllerElevatorOutput = (KV * error) + elevatorPositionIntegration - (KA * estimatedElevatorAcc);
	if (velocityControllerElevatorOutput > CONTROLLER_ELEVATOR_SATURATION) {velocityControllerElevatorOutput = CONTROLLER_ELEVATOR_SATURATION;} else 
	if (velocityControllerElevatorOutput < -CONTROLLER_ELEVATOR_SATURATION) {velocityControllerElevatorOutput = -CONTROLLER_ELEVATOR_SATURATION;}


	//aileron controller
	error = aileronVelocitySetpoint - estimatedAileronVel2;	

	aileronVelocityIntegration += KI * error * DT;
	if (aileronVelocityIntegration > CONTROLLER_AILERON_SATURATION/3) {aileronVelocityIntegration = CONTROLLER_AILERON_SATURATION/3;} else 
	if (aileronVelocityIntegration < -CONTROLLER_AILERON_SATURATION/3) {aileronVelocityIntegration = -CONTROLLER_AILERON_SATURATION/3;}

	velocityControllerAileronOutput = (KV * error) + aileronPositionIntegration - (KA * estimatedAileronAcc);
	if (velocityControllerAileronOutput > CONTROLLER_AILERON_SATURATION) {velocityControllerAileronOutput = CONTROLLER_AILERON_SATURATION;} else 
	if (velocityControllerAileronOutput < -CONTROLLER_AILERON_SATURATION) {velocityControllerAileronOutput = -CONTROLLER_AILERON_SATURATION;}	
}

void positionController() {
	float KX, KI, KP, KV, KA;

	//set controller constants
	KI = POSITION_KI;
	KP = POSITION_KP;
	KV = POSITION_KV;
	KA = POSITION_KA;
	KX = KP / KV;

	//elevator controller
	elevatorPosContError = elevatorPositionSetpoint - estimatedElevatorPos;
	elevatorDesiredSpeedPosController = KX * elevatorPosContError/* + elevatorDesiredSpeedPosControllerLeader*/;
	if(elevatorDesiredSpeedPosController > +SPEED_MAX) elevatorDesiredSpeedPosController = +SPEED_MAX;
	if(elevatorDesiredSpeedPosController < -SPEED_MAX) elevatorDesiredSpeedPosController = -SPEED_MAX;

	elevatorPositionIntegration += KI * elevatorPosContError * DT;
	if (elevatorPositionIntegration > CONTROLLER_ELEVATOR_SATURATION/3) {elevatorPositionIntegration = CONTROLLER_ELEVATOR_SATURATION/3;} else 
	if (elevatorPositionIntegration < -CONTROLLER_ELEVATOR_SATURATION/3) {elevatorPositionIntegration = -CONTROLLER_ELEVATOR_SATURATION/3;}

	positionControllerElevatorOutput = KV * (elevatorDesiredSpeedPosController - estimatedElevatorVel2) + elevatorPositionIntegration - (KA * estimatedElevatorAcc);
	if (positionControllerElevatorOutput > CONTROLLER_ELEVATOR_SATURATION) {positionControllerElevatorOutput = CONTROLLER_ELEVATOR_SATURATION;} else 
	if (positionControllerElevatorOutput < -CONTROLLER_ELEVATOR_SATURATION) {positionControllerElevatorOutput = -CONTROLLER_ELEVATOR_SATURATION;}


	//aileron controller
	aileronPosContError = aileronPositionSetpoint - estimatedAileronPos;
	aileronDesiredSpeedPosController = KX * aileronPosContError/* + aileronDesiredSpeedPosControllerLeader*/;
	if(aileronDesiredSpeedPosController > +SPEED_MAX) aileronDesiredSpeedPosController = +SPEED_MAX;
	if(aileronDesiredSpeedPosController < -SPEED_MAX) aileronDesiredSpeedPosController = -SPEED_MAX;

	aileronPositionIntegration += KI * aileronPosContError * DT;
	if (aileronPositionIntegration > CONTROLLER_AILERON_SATURATION/3) {aileronPositionIntegration = CONTROLLER_AILERON_SATURATION/3;} else 
	if (aileronPositionIntegration < -CONTROLLER_AILERON_SATURATION/3) {aileronPositionIntegration = -CONTROLLER_AILERON_SATURATION/3;} 

	positionControllerAileronOutput = KV * (aileronDesiredSpeedPosController - estimatedAileronVel2) + aileronPositionIntegration - (KA * estimatedAileronAcc);
	if (positionControllerAileronOutput > CONTROLLER_AILERON_SATURATION) {positionControllerAileronOutput = CONTROLLER_AILERON_SATURATION;} else 
	if (positionControllerAileronOutput < -CONTROLLER_AILERON_SATURATION) {positionControllerAileronOutput = -CONTROLLER_AILERON_SATURATION;}

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

void altitudeController() {
	float error;
	float vd; //desired velocity
	float KX, KI, KV;

	KX = ((float)ALTITUDE_KP / ALTITUDE_KV);
	KI = ALTITUDE_KI;
	KV = ALTITUDE_KV;

	error =(throttleSetpoint - estimatedThrottlePos);
	vd = KX * error;
	if(vd > +ALTITUDE_SPEED_MAX) vd = +ALTITUDE_SPEED_MAX;
	if(vd < -ALTITUDE_SPEED_MAX) vd = -ALTITUDE_SPEED_MAX;
	

	// calculate integrational
	throttleIntegration += KI * error * DT;
	if (throttleIntegration > CONTROLLER_THROTTLE_SATURATION*2/3) {
		throttleIntegration = CONTROLLER_THROTTLE_SATURATION*2/3;
	}
	if (throttleIntegration <  -CONTROLLER_THROTTLE_SATURATION*2/3) {
		throttleIntegration = -CONTROLLER_THROTTLE_SATURATION*2/3;
	}

	//total output
	controllerThrottleOutput =
		KV * (vd - estimatedThrottleVel) + throttleIntegration;
	if (controllerThrottleOutput > CONTROLLER_THROTTLE_SATURATION) {
		controllerThrottleOutput = CONTROLLER_THROTTLE_SATURATION;
	}
	if (controllerThrottleOutput < -CONTROLLER_THROTTLE_SATURATION) {
		controllerThrottleOutput = -CONTROLLER_THROTTLE_SATURATION;
	}

}

void landingStateAutomat(){
	switch(landingState){
		case LS_ON_GROUND:
			landingThrottleOutput = -CONTROLLER_THROTTLE_SATURATION;
			if(!landingRequest){
				landingCounter = 0;
				landingState = LS_TAKEOFF;
			} break;
		case LS_TAKEOFF:
			if(landingRequest){
				landingCounter = 0;
				landingState = LS_STABILIZATION;				
			}else{
				landingThrottleSetpoint = throttleDesiredSetpoint;
				landingThrottleOutput=controllerThrottleOutput;
				
				//stabilize altitude for 0.5s
				if(fabs(throttleSetpoint - estimatedThrottlePos) < 0.1 && fabs(estimatedThrottleVel) < 0.2){
					landingCounter++;
				}else{
					landingCounter = 0;
				}
				if(landingCounter >= 35){
					landingState = LS_FLIGHT;
				}
			} break;
		case LS_FLIGHT:
			if(landingRequest){
				landingCounter = 0;
				landingState = LS_STABILIZATION;
			} break;
		case LS_STABILIZATION:
			if(!landingRequest){
				landingState = LS_TAKEOFF;
			}else{				
				//stabilize altitude for 0.5s
				landingThrottleSetpoint = ALTITUDE_MINIMUM;
				if(fabs(throttleSetpoint - estimatedThrottlePos) < 0.1 && fabs(estimatedThrottleVel ) < 0.2){
					landingCounter++;
					landingThrottleOutput+=(controllerThrottleOutput-landingThrottleOutput)*(DT/PX4FLOW_FILTER_CONST);
				}else{
					landingCounter = 0;
					landingThrottleOutput=controllerThrottleOutput;
				}
				if(landingCounter >= 35){
					landingState = LS_LANDING;					
				}
			} break;
		default: //LS_LANDING
			if(!landingRequest){
				landingCounter = 0;
				landingState = LS_TAKEOFF;
			}else{
				landingThrottleOutput-=5;
				if(landingThrottleOutput <= -CONTROLLER_THROTTLE_SATURATION){
					landingState = LS_ON_GROUND;
				}
			}
	} //endswitch

}
