#include <stdio.h>
#include "packets.h"
#include "commands.h"
#include "communication.h"
#include "controllers.h"
#include "system.h"
#include "commTask.h"



//XBee data payload
unsigned char dataOUT[200];


//GENERAL
void dataTypeError(unsigned char *address64,unsigned char *address16,unsigned char *data){
}
void packetTypeError(unsigned char *inPacket){
}


//TELEMETRY
float telemetryValue(unsigned char type){
	float f=0.0;
	if(type==TELEMETRIES.GROUND_DISTANCE_ESTIMATED){
		f=estimatedThrottlePos;
	}else if(type==TELEMETRIES.GROUND_DISTANCE){
		f=groundDistance;
	}else if(type==TELEMETRIES.ELEVATOR_SPEED){
		f=elevatorSpeed;
	}else if(type==TELEMETRIES.AILERON_SPEED){
		f=aileronSpeed;
	}else if(type==TELEMETRIES.ELEVATOR_SPEED_ESTIMATED){
		f=estimatedElevatorVel;
	}else if(type==TELEMETRIES.AILERON_SPEED_ESTIMATED){
		f=estimatedAileronVel;
	}else if(type==TELEMETRIES.ELEVATOR_POS_ESTIMATED){
		f=estimatedElevatorPos;
	}else if(type==TELEMETRIES.AILERON_POS_ESTIMATED){
		f=estimatedAileronPos;
	}else if(type==TELEMETRIES.THROTTLE_CONTROLLER_OUTPUT){
		f=controllerThrottleOutput;
	}else if(type==TELEMETRIES.THROTTLE_SPEED){
		f=estimatedThrottleVel;
	}else if(type==TELEMETRIES.AILERON_CONTROLLER_OUTPUT){
		f=controllerAileronOutput;
	}else if(type==TELEMETRIES.ELEVATOR_CONTROLLER_OUTPUT){
		f=controllerElevatorOutput;
	}else if(type==TELEMETRIES.THROTTLE_SETPOINT){
		f=throttleSetpoint;
	}else if(type==TELEMETRIES.ELEVATOR_POS_SETPOINT){
		f=elevatorPositionSetpoint;
	}else if(type==TELEMETRIES.AILERON_POS_SETPOINT){
		f=aileronPositionSetpoint;
	}else if(type==TELEMETRIES.ELEVATOR_VEL_SETPOINT){
		f=elevatorVelocitySetpoint;
	}else if(type==TELEMETRIES.AILERON_VEL_SETPOINT){
		f=aileronVelocitySetpoint;
	}else if(type==TELEMETRIES.ELEVATOR_ACC){
		f=estimatedElevatorAcc;
	}else if(type==TELEMETRIES.AILERON_ACC){
		f=estimatedAileronAcc;
	}else if(type==TELEMETRIES.VALID_GUMSTIX){
		f=validGumstix;
    }else if(type==TELEMETRIES.OUTPUT_THROTTLE){
		f=(float)outputThrottle;
    }else if(type==TELEMETRIES.OUTPUT_ELEVATOR){
		f=(float)outputElevator;
    }else if(type==TELEMETRIES.OUTPUT_AILERON){
		f=(float)outputAileron;
    }else if(type==TELEMETRIES.OUTPUT_RUDDER){
		f=(float)outputRudder;
    }else if(type==TELEMETRIES.BLOB_DISTANCE){
		f=estimatedBlobDistance;
    }else if(type==TELEMETRIES.BLOB_HORIZONTAL){
		f=estimatedBlobHorizontal;
    }else if(type==TELEMETRIES.BLOB_VERTICAL){
		f=estimatedBlobVertical;
    }else if(type==TELEMETRIES.PITCH_ANGLE){
		f=(float)pitchAngle/10.0;
    }else if(type==TELEMETRIES.ROLL_ANGLE){
		f=(float)rollAngle/10.0;
    }
	return f;
}
void telemetrySend(unsigned char *address64,unsigned char *address16,unsigned char type,unsigned char frameID){
	float f=0;
	unsigned char *ch;

	f=telemetryValue(type);
	
	ch=(unsigned char *) &f;
	*dataOUT='t';
	*(dataOUT+1)=type;
	*(dataOUT+2)=*ch;
	*(dataOUT+3)=*(ch+1);
	*(dataOUT+4)=*(ch+2);
	*(dataOUT+5)=*(ch+3);
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,6);
}
void telemetryRequest(unsigned char *address64,unsigned char *address16,unsigned char type, unsigned char frameID){
	*dataOUT='c';
	*(dataOUT+1)=COMMANDS.TELEMETRY;
	*(dataOUT+2)=type;
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,3);
}
void telemetryReceive(unsigned char *address64,unsigned char *address16,unsigned char type,float value){
	if(type==TELEMETRIES.GROUND_DISTANCE_ESTIMATED){
		
		}else if(type==TELEMETRIES.GROUND_DISTANCE){
		
		}else if(type==TELEMETRIES.ELEVATOR_SPEED){
		
		}else if(type==TELEMETRIES.AILERON_SPEED){
		
		}else if(type==TELEMETRIES.ELEVATOR_SPEED_ESTIMATED){
		
		}else if(type==TELEMETRIES.AILERON_SPEED_ESTIMATED){
		
		}else if(type==TELEMETRIES.ELEVATOR_POS_ESTIMATED){
		
		}else if(type==TELEMETRIES.AILERON_POS_ESTIMATED){
		
		}else if(type==TELEMETRIES.THROTTLE_CONTROLLER_OUTPUT){
		
		}else if(type==TELEMETRIES.THROTTLE_SPEED){
		
		}else if(type==TELEMETRIES.AILERON_CONTROLLER_OUTPUT){
		
		}else if(type==TELEMETRIES.ELEVATOR_CONTROLLER_OUTPUT){
		
		}else if(type==TELEMETRIES.THROTTLE_SETPOINT){
		
		}else if(type==TELEMETRIES.ELEVATOR_POS_SETPOINT){

		}else if(type==TELEMETRIES.AILERON_POS_SETPOINT){
		
		}else if(type==TELEMETRIES.ELEVATOR_VEL_SETPOINT){
		
		}else if(type==TELEMETRIES.AILERON_VEL_SETPOINT){

		}else if(type==TELEMETRIES.ELEVATOR_ACC){
		
		}else if(type==TELEMETRIES.AILERON_ACC){
		
		}else if(type==TELEMETRIES.VALID_GUMSTIX){
				
		}
}

void telemetryToCoordinatorSend(){
	float f=0;
	unsigned char *ch;
	unsigned char type=0xFF;
	unsigned char i=0;
	unsigned char counter=0;
	
	*dataOUT='t';
	for(i=0;i<TELEMETRY_VARIABLES;i++){
		if(telemetryToCoordinatorArr[i]==ONOFF.ON){			
			type=i;			

			f=telemetryValue(type);
		
			ch=(unsigned char *) &f;		
			*(dataOUT+1+counter*5)=type;
			*(dataOUT+2+counter*5)=*ch;
			*(dataOUT+3+counter*5)=*(ch+1);
			*(dataOUT+4+counter*5)=*(ch+2);
			*(dataOUT+5+counter*5)=*(ch+3);
			counter++;
		}
	}
	if(counter>0){makeTRPacket(ADDRESS.COORDINATOR,ADDRESS.UNKNOWN16,0x01,0x00,dataOUT,counter*5+1);}
}
void telemetryToCoordinatorSet(unsigned char *address64,unsigned char *address16,unsigned char type,unsigned char on, unsigned char frameID){
	*dataOUT='c';
	*(dataOUT+1)=COMMANDS.TELEMETRY_COORDINATOR;
	*(dataOUT+2)=type;
	*(dataOUT+3)=on;
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,4);
}
void telemetryToCoordinator(unsigned char *address64,unsigned char *address16,unsigned char type,unsigned char on){
	telemetryToCoordinatorArr[type]=on;
}
void telemetryToCoordinatorStatusRequest(unsigned char *address64,unsigned char *address16,unsigned char type,unsigned char frameID){
	*dataOUT='c';
	*(dataOUT+1)=COMMANDS.TELEMETRY_COORDINATOR;
	*(dataOUT+2)=GET_STATUS;
	*(dataOUT+3)=type;
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,4);
}
void telemetryToCoordinatorReport(unsigned char *address64,unsigned char *address16,unsigned char type,unsigned char frameID){
	*dataOUT='r';
	*(dataOUT+1)=COMMANDS.TELEMETRY_COORDINATOR;
	*(dataOUT+2)=type;
	*(dataOUT+3)=telemetryToCoordinatorArr[type];
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,4);
}
void telemetryToCoordinatorReportRecieved(unsigned char *address64,unsigned char *address16,unsigned char type,unsigned char status){
	
}

//LANDING
void kopterLandRequest(unsigned char *address64,unsigned char *address16,unsigned char options,unsigned char frameID){
	*(dataOUT)='c';
	*(dataOUT+1)=COMMANDS.LANDING;
	*(dataOUT+2)=options;
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,3);
}
void kopterLand(unsigned char *address64,unsigned char *address16,unsigned char on){
	if(on){
		enableLanding();		
	}else{
		disableLanding();
	}
}
void kopterLandStatusRequest(unsigned char *address64,unsigned char *address16,unsigned char frameID){
	*dataOUT='c';
	*(dataOUT+1)=COMMANDS.LANDING;
	*(dataOUT+2)=GET_STATUS;
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,3);
}
void kopterLandReport(unsigned char *address64,unsigned char *address16,unsigned char frameID){
	*dataOUT='r';
	*(dataOUT+1)=COMMANDS.LANDING;
	*(dataOUT+2)=landingState;	
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,3);
}
void kopterLandReportRecieved(unsigned char *address64,unsigned char *address16,unsigned char status){
	if(status==LANDING.LANDING){
		
	}else if(status==LANDING.FLIGHT){
		
	}else if(status==LANDING.STABILIZATION){
	
	}else if(status==LANDING.ON_GROUND){

	}else if(status==LANDING.TAKE_OFF){

	}
}

//SETPOINTS
void kopterSetpointsSetRequest(unsigned char *address64,unsigned char *address16,unsigned char type,unsigned char positionType,float value,unsigned char frameID){
	unsigned char *ch;
	
	*(dataOUT)='c';
	*(dataOUT+1)=COMMANDS.SET_SETPOINTS;
	*(dataOUT+2)=type;
	*(dataOUT+3)=positionType;
	
	ch=(unsigned char *) &value;
	*(dataOUT+4)=*ch;
	*(dataOUT+5)=*(ch+1);
	*(dataOUT+6)=*(ch+2);
	*(dataOUT+7)=*(ch+3);
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,8);
}
void kopterSetpointsSet(unsigned char *address64,unsigned char *address16,unsigned char type,unsigned char positionType,float value){
	//positions setpoints
	if(type==SETPOINTS.THROTTLE_SP){
		if(positionType==POSITIONS.ABSOLUT){
			throttleDesiredSetpoint=value;
		}else if(positionType==POSITIONS.RELATIV){
			throttleDesiredSetpoint+=value;
		}
		if(throttleDesiredSetpoint > +THROTTLE_SP_HIGH) throttleDesiredSetpoint = +THROTTLE_SP_HIGH;
		if(throttleDesiredSetpoint < -THROTTLE_SP_LOW)  throttleDesiredSetpoint = -THROTTLE_SP_LOW;
		
		}else if(type==SETPOINTS.ELEVATOR_POSITION){
		if(positionType==POSITIONS.ABSOLUT){
			elevatorDesiredPositionSetpoint=value;
			}else if(positionType==POSITIONS.RELATIV){
			elevatorDesiredPositionSetpoint+=value;
		}
		
		}else if(type==SETPOINTS.AILERON_POSITION){
		if(positionType==POSITIONS.ABSOLUT){
			aileronDesiredPositionSetpoint=value;
			}else if(positionType==POSITIONS.RELATIV){
			aileronDesiredPositionSetpoint+=value;
		}
		
		}else if(type==SETPOINTS.ELEVATOR_VELOCITY){
		if(positionType==POSITIONS.ABSOLUT){
			elevatorDesiredVelocitySetpoint=value;
			}else if(positionType==POSITIONS.RELATIV){
			elevatorDesiredVelocitySetpoint+=value;
		}
		if(elevatorDesiredVelocitySetpoint > +SPEED_MAX) elevatorDesiredVelocitySetpoint = +SPEED_MAX;
		if(elevatorDesiredVelocitySetpoint < -SPEED_MAX) elevatorDesiredVelocitySetpoint = -SPEED_MAX;
		
		}else if(type==SETPOINTS.AILERON_VELOCITY){
		if(positionType==POSITIONS.ABSOLUT){
			aileronDesiredVelocitySetpoint=value;
			}else if(positionType==POSITIONS.RELATIV){
			aileronDesiredVelocitySetpoint+=value;
		}
		if(aileronDesiredVelocitySetpoint > +SPEED_MAX) aileronDesiredVelocitySetpoint = +SPEED_MAX;
		if(aileronDesiredVelocitySetpoint < -SPEED_MAX) aileronDesiredVelocitySetpoint = -SPEED_MAX;
		
	}
}
void kopterSetpointStatusRequest(unsigned char *address64,unsigned char *address16,unsigned char type,unsigned char frameID){
	*(dataOUT)='c';
	*(dataOUT+1)=COMMANDS.SET_SETPOINTS;
	*(dataOUT+2)=GET_STATUS;
	*(dataOUT+3)=type;
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,4);
}
void kopterSetpointsReport(unsigned char *address64,unsigned char *address16,unsigned char type,unsigned char frameID){
	float f=0;
	unsigned char *ch;
	
	*(dataOUT)='r';
	*(dataOUT+1)=COMMANDS.SET_SETPOINTS;
	*(dataOUT+2)=type;
	if(type==SETPOINTS.THROTTLE_SP){
		f=throttleDesiredSetpoint;
	}else if(type==SETPOINTS.ELEVATOR_POSITION){
		f=elevatorDesiredPositionSetpoint;
	}else if(type==SETPOINTS.AILERON_POSITION){
		f=aileronDesiredPositionSetpoint;
	}else if(type==SETPOINTS.ELEVATOR_VELOCITY){
		f=elevatorDesiredVelocitySetpoint;
	}else if(type==SETPOINTS.AILERON_VELOCITY){
		f=aileronDesiredVelocitySetpoint;
	}
	
	ch=(unsigned char *) &f;
	*(dataOUT+3)=*ch;
	*(dataOUT+4)=*(ch+1);
	*(dataOUT+5)=*(ch+2);
	*(dataOUT+6)=*(ch+3);
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,7);
}
void kopterSetpointsReportReceived(unsigned char *address64,unsigned char *address16,unsigned char type,float value){
	if(		 type==SETPOINTS.THROTTLE_SP){
		
		}else if(type==SETPOINTS.AILERON_POSITION){
		
		}else if(type==SETPOINTS.ELEVATOR_POSITION){

		}else if(type==SETPOINTS.AILERON_VELOCITY){
		
		}else if(type==SETPOINTS.ELEVATOR_VELOCITY){
	}
}

//CONTROLLERS
void kopterControllersRequest(unsigned char *address64,unsigned char *address16,unsigned char option,unsigned char frameID){
	*(dataOUT)='c';
	*(dataOUT+1)=COMMANDS.CONTROLLERS;
	*(dataOUT+2)=option;
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,3);
}
void kopterControllers(unsigned char *address64,unsigned char *address16,unsigned char option){
	portENTER_CRITICAL();
	if(		 option==CONTROLLERS.OFF){
		controllerSet(CONTROLLERS.OFF);	
	}else if(option==CONTROLLERS.POSITION){
		controllerSet(CONTROLLERS.POSITION);	
	}else if(option==CONTROLLERS.VELOCITY){
		controllerSet(CONTROLLERS.VELOCITY);	
	}else if(option==CONTROLLERS.MPC){
		controllerSet(CONTROLLERS.MPC);	
	}
	portEXIT_CRITICAL();
}
void kopterControllersStatusRequest(unsigned char *address64,unsigned char *address16,unsigned char frameID){
	*(dataOUT)='c';
	*(dataOUT+1)=COMMANDS.CONTROLLERS;
	*(dataOUT+2)=GET_STATUS;
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,3);
}
void kopterControllersReport(unsigned char *address64,unsigned char *address16,unsigned char frameID){
	*(dataOUT)='r';
	*(dataOUT+1)=COMMANDS.CONTROLLERS;
	*(dataOUT+2)=controllerActive;
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,3);
}
void kopterControllersReportReceived(unsigned char *address64,unsigned char *address16,unsigned char status){
	if(		 status==CONTROLLERS.OFF){
		
	}else if(status==CONTROLLERS.POSITION){
	
	}else if(status==CONTROLLERS.VELOCITY){

	}else if(status==CONTROLLERS.MPC){
		
	}
}

//TRAJECTORY
void kopterTrajectoryFollowRequest(unsigned char *address64,unsigned char *address16,unsigned char options,unsigned char frameID){
	*(dataOUT)='c';
	*(dataOUT+1)=COMMANDS.TRAJECTORY_FOLLOW;
	*(dataOUT+2)=options;
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,3);
}
void kopterTrajectoryFollow(unsigned char *address64,unsigned char *address16,unsigned char on){
	trajectoryEnabled=on;
}
void kopterTrajectoryFollowStatusRequest(unsigned char *address64,unsigned char *address16,unsigned char frameID){
	*dataOUT='c';
	*(dataOUT+1)=COMMANDS.TRAJECTORY_FOLLOW;
	*(dataOUT+2)=GET_STATUS;
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,3);
}
void kopterTrajectoryFollowReport(unsigned char *address64,unsigned char *address16,unsigned char frameID){
	*dataOUT='r';
	*(dataOUT+1)=COMMANDS.TRAJECTORY_FOLLOW;
	if(trajectoryEnabled){
		*(dataOUT+2)=ONOFF.ON;
	}else{
		*(dataOUT+2)=ONOFF.OFF;
	}
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,3);
}
void kopterTrajectoryFollowReportRecieved(unsigned char *address64,unsigned char *address16,unsigned char status){
	if(			status==ONOFF.ON){
		
	}else if(status==ONOFF.OFF){
		
	}
}

//TRAJECTORY POINTS
void kopterTrajectoryAddPointRequest(unsigned char *address64,unsigned char *address16,unsigned char index,float time,float elevatorPos,float aileronPos,float throttlePos,unsigned char frameID){
	unsigned char *ch;
	
	*(dataOUT)='c';
	*(dataOUT+1)=COMMANDS.TRAJECTORY_POINTS;
	*(dataOUT+2)=index;
	
	ch=(unsigned char *) &time;
	*(dataOUT+3)=*ch; *(dataOUT+4)=*(ch+1); *(dataOUT+5)=*(ch+2); *(dataOUT+6)=*(ch+3);
	
	ch=(unsigned char *) &elevatorPos;
	*(dataOUT+7)=*ch; *(dataOUT+8)=*(ch+1); *(dataOUT+9)=*(ch+2); *(dataOUT+10)=*(ch+3);
	
	ch=(unsigned char *) &aileronPos;
	*(dataOUT+11)=*ch; *(dataOUT+12)=*(ch+1); *(dataOUT+13)=*(ch+2); *(dataOUT+14)=*(ch+3);	
	
	ch=(unsigned char *) &throttlePos;
	*(dataOUT+15)=*ch; *(dataOUT+16)=*(ch+1); *(dataOUT+17)=*(ch+2); *(dataOUT+18)=*(ch+3);	
		
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,19);
}
void kopterTrajectoryAddPoint(unsigned char *address64,unsigned char *address16,unsigned char index,float time,float elevatorPos,float aileronPos,float throttlePos){
	if(index<TRAJECTORY_LENGTH){
		trajMaxIndex=index;
		TRAJ_POINT(index,time,elevatorPos,aileronPos,throttlePos);
	}		
}
void kopterTrajectoryPointStatusRequest(unsigned char *address64,unsigned char *address16,unsigned char frameID){
	*(dataOUT)='c';
	*(dataOUT+1)=COMMANDS.TRAJECTORY_POINTS;
	*(dataOUT+2)=GET_STATUS;
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,4);
}
void kopterTrajectoryPointReport(unsigned char *address64,unsigned char *address16,unsigned char index,unsigned char frameID){
	unsigned char *ch;
	
	*(dataOUT)='r';
	*(dataOUT+1)=COMMANDS.TRAJECTORY_POINTS;
	*(dataOUT+2)=index;
	
	ch=(unsigned char *) &trajectory[index].time;
	*(dataOUT+3)=*ch; *(dataOUT+4)=*(ch+1); *(dataOUT+5)=*(ch+2); *(dataOUT+6)=*(ch+3);
	
	ch=(unsigned char *) &trajectory[index].elevatorPos;
	*(dataOUT+7)=*ch; *(dataOUT+8)=*(ch+1); *(dataOUT+9)=*(ch+2); *(dataOUT+10)=*(ch+3);
	
	ch=(unsigned char *) &trajectory[index].aileronPos;
	*(dataOUT+11)=*ch; *(dataOUT+12)=*(ch+1); *(dataOUT+13)=*(ch+2); *(dataOUT+14)=*(ch+3);
	
	ch=(unsigned char *) &trajectory[index].throttlePos;
	*(dataOUT+15)=*ch; *(dataOUT+16)=*(ch+1); *(dataOUT+17)=*(ch+2); *(dataOUT+18)=*(ch+3);
	
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,19);
}
void kopterTrajectoryPointReportReceived(unsigned char *address64,unsigned char *address16,unsigned char index,float time,float elevatorPos,float aileronPos,float throttlePos){
	
}

//FOLLOWER SET
void kopterFollowerSetRequest(unsigned char *address64,unsigned char *address16,unsigned char *followerAddr,unsigned char frameID){
	char i;
	*(dataOUT)='c';
	*(dataOUT+1)=COMMANDS.FOLLOWER_SET;
	for (i=0;i<8;i++){
		*(dataOUT+2+i)=*(followerAddr+i);
	}
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,10);
}
void kopterFollowerSet(unsigned char *address64,unsigned char *address16,unsigned char *followerAddr){
	char i;
	for(i=0;i<8;i++){
	*(leadKopter+i)=*(followerAddr+i);
	}
}
void kopterFollowerSetStatusRequest(unsigned char *address64,unsigned char *address16,unsigned char frameID){
	*(dataOUT)='c';
	*(dataOUT+1)=COMMANDS.FOLLOWER_SET;
	*(dataOUT+2)=GET_STATUS;
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,3);
}
void kopterFollowerSetReport(unsigned char *address64,unsigned char *address16,unsigned char frameID){
	char i;
	*(dataOUT)='r';
	*(dataOUT+1)=COMMANDS.FOLLOWER_SET;
	for (i=0;i<8;i++){
		*(dataOUT+2+i)=*(leadKopter+i);
	}
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,10);
}
void kopterFollowerSetReportRecieved(unsigned char *address64,unsigned char *address16,unsigned char *followerAddr){
	
}

//MESSAGES
void sendXBeeMessage(unsigned char *address64,unsigned char *address16,char *message,unsigned char frameID){
	char out[100];
	sprintf(out,"%s%s","0",message);
	*(out)='m';
	makeTRPacket(address64,address16,0x00,frameID,(unsigned char*)out,(unsigned char)(strlen(message)+1));
}
void receiveXBeeMessage(unsigned char *address64,unsigned char *address16,char *message){
	
}

//LEADING
void kopterLeadDataSend(unsigned char *address64,unsigned char *address16,volatile float altitude,volatile float elevatorError,volatile float aileronError,unsigned char frameID){
	unsigned char *ch;	
	*(dataOUT)='l';
	
	ch=(unsigned char *) &altitude;
	*(dataOUT+1)=*ch; *(dataOUT+2)=*(ch+1); *(dataOUT+3)=*(ch+2); *(dataOUT+4)=*(ch+3);
	
	ch=(unsigned char *) &elevatorError;
	*(dataOUT+5)=*ch; *(dataOUT+6)=*(ch+1); *(dataOUT+7)=*(ch+2); *(dataOUT+8)=*(ch+3);
	
	ch=(unsigned char *) &aileronError;
	*(dataOUT+9)=*ch; *(dataOUT+10)=*(ch+1); *(dataOUT+11)=*(ch+2); *(dataOUT+12)=*(ch+3);		
	
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,13);
}
void kopterLeadDataReceived(unsigned char *address64,unsigned char *address16,float altitude, float elevatorError, float aileronError){
	throttleDesiredSetpoint=altitude;
}

//TIME
void kopterTimeRequest(unsigned char *address64,unsigned char *address16,int64_t time,unsigned char frameID){
		unsigned char *ch;		
		
		*(dataOUT)='c';
		*(dataOUT+1)=COMMANDS.TIME;
		
		ch=(unsigned char *) &time;
		*(dataOUT+2)=*ch;
		*(dataOUT+3)=*(ch+1);
		*(dataOUT+4)=*(ch+2);
		*(dataOUT+5)=*(ch+3);
		makeTRPacket(address64,address16,0x00,frameID,dataOUT,6);
}
void kopterTime(unsigned char *address64,unsigned char *address16,int64_t time){	
	secondsTimer=time;
}
void kopterTimeStatusRequest(unsigned char *address64,unsigned char *address16,unsigned char frameID){
	*(dataOUT)='c';
	*(dataOUT+1)=COMMANDS.TIME;
	*(dataOUT+2)=GET_STATUS;
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,3);	
}
void kopterTimeReport(unsigned char *address64,unsigned char *address16,unsigned char frameID){
			unsigned char *ch;
			
			*(dataOUT)='r';
			*(dataOUT+1)=COMMANDS.TIME;
			
			ch=(unsigned char *) &secondsTimer;
			*(dataOUT+2)=*ch;
			*(dataOUT+3)=*(ch+1);
			*(dataOUT+4)=*(ch+2);
			*(dataOUT+5)=*(ch+3);
			makeTRPacket(address64,address16,0x00,frameID,dataOUT,6);	
}
void kopterTimeReportReceived(unsigned char *address64,unsigned char *address16,int64_t time){
	
}