#include <stdio.h>
#include "packets.h"
#include "commands.h"
#include "communication.h"
#include "controllers.h"
#include "system.h"



//UDP XBee packetOUT
unsigned char dataOUT[25];


//GENERAL
void dataTypeError(unsigned char *address64,unsigned char *address16,unsigned char *data){
}
void packetTypeError(unsigned char *inPacket){
}


//TELEMETRY
void telemetrySend(unsigned char *address64,unsigned char *address16,unsigned char type,unsigned char frameID){
	float f=0;
	unsigned char *ch;

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
	}else if(type==TELEMETRIES.AILERON_POS_CONTROLLER_OUTPUT){
		f=positionControllerAileronOutput;
	}else if(type==TELEMETRIES.ELEVATOR_POS_CONTROLLER_OUTPUT){
		f=positionControllerElevatorOutput;
	}else if(type==TELEMETRIES.AILERON_VEL_CONTROLLER_OUTPUT){
		f=velocityControllerAileronOutput;
	}else if(type==TELEMETRIES.ELEVATOR_VEL_CONTROLLER_OUTPUT){
		f=velocityControllerElevatorOutput;
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
	}	
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
	
	}else if(type==TELEMETRIES.AILERON_POS_CONTROLLER_OUTPUT){
	
	}else if(type==TELEMETRIES.ELEVATOR_POS_CONTROLLER_OUTPUT){
	
	}else if(type==TELEMETRIES.AILERON_VEL_CONTROLLER_OUTPUT){
	
	}else if(type==TELEMETRIES.ELEVATOR_VEL_CONTROLLER_OUTPUT){
	
	}else if(type==TELEMETRIES.THROTTLE_SETPOINT){
	
	}else if(type==TELEMETRIES.ELEVATOR_POS_SETPOINT){

	}else if(type==TELEMETRIES.AILERON_POS_SETPOINT){
	
	}else if(type==TELEMETRIES.ELEVATOR_VEL_SETPOINT){
	
	}else if(type==TELEMETRIES.AILERON_VEL_SETPOINT){
	
	}
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
	if(status==LS_LANDING){
		
	}else if(status==LS_FLIGHT){
		
	}else if(status==LS_STABILIZATION){
	
	}else if(status==LS_ON_GROUND){

	}else if(status==LS_TAKEOFF){

	}
}

//TRAJECTORY
void kopterTrajectoryRequest(unsigned char *address64,unsigned char *address16,unsigned char options,unsigned char frameID){
	*(dataOUT)='c';
	*(dataOUT+1)=COMMANDS.TRAJECTORY_FOLLOW;
	*(dataOUT+2)=options;
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,3);
}
void kopterTrajectory(unsigned char *address64,unsigned char *address16,unsigned char on){
	trajectoryEnabled=on;
}
void kopterTrajectoryStatusRequest(unsigned char *address64,unsigned char *address16,unsigned char frameID){
	*dataOUT='c';
	*(dataOUT+1)=COMMANDS.TRAJECTORY_FOLLOW;
	*(dataOUT+2)=GET_STATUS;
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,3);
}
void kopterTrajectoryReport(unsigned char *address64,unsigned char *address16,unsigned char frameID){
	*dataOUT='r';
	*(dataOUT+1)=COMMANDS.TRAJECTORY_FOLLOW;
	if(trajectoryEnabled){
		*(dataOUT+2)=ONOFF.ON;
		}else{
		*(dataOUT+2)=ONOFF.OFF;
	}
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,3);
}
void kopterTrajectoryReportRecieved(unsigned char *address64,unsigned char *address16,unsigned char status){
	if(			status==ONOFF.ON){
		
	}else if(status==ONOFF.OFF){
		
	}
}

//TRAJECTORY POINTS
void kopterTrajectoryAddPointRequest(unsigned char *address64,unsigned char *address16,unsigned char index,float time,float elevatorPos,float aileronPos,float throttlePos,unsigned char frameID){
	unsigned char *ch;
	
	*(dataOUT)='c';
	*(dataOUT+1)=COMMANDS.TRAJECTORY_POINTS;
	*(dataOUT+2)=COMMANDS.TRAJECTORY_POINTS;
	*(dataOUT+3)=index;
	
	ch=(unsigned char *) &time;
	*(dataOUT+4)=*ch; *(dataOUT+5)=*(ch+1); *(dataOUT+6)=*(ch+2); *(dataOUT+7)=*(ch+3);
	
	ch=(unsigned char *) &elevatorPos;
	*(dataOUT+8)=*ch; *(dataOUT+9)=*(ch+1); *(dataOUT+10)=*(ch+2); *(dataOUT+11)=*(ch+3);
	
	ch=(unsigned char *) &aileronPos;
	*(dataOUT+12)=*ch; *(dataOUT+13)=*(ch+1); *(dataOUT+14)=*(ch+2); *(dataOUT+15)=*(ch+3);	
	
	ch=(unsigned char *) &throttlePos;
	*(dataOUT+16)=*ch; *(dataOUT+17)=*(ch+1); *(dataOUT+18)=*(ch+2); *(dataOUT+19)=*(ch+3);	
		
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,20);
}
void kopterTrajectoryAddPoint(unsigned char *address64,unsigned char *address16,unsigned char index,float time,float elevatorPos,float aileronPos,float throttlePos){
	if(index<TRAJECTORY_LENGTH){
		if(index>trajMaxIndex){
			trajMaxIndex=index;
		}
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
		
		}else if(type==SETPOINTS.AILERON_VELOCITY){
			if(positionType==POSITIONS.ABSOLUT){
				aileronDesiredVelocitySetpoint=value;
			}else if(positionType==POSITIONS.RELATIV){
				aileronDesiredVelocitySetpoint+=value;
			}
		}			
}
void kopterSetpointStatusRequest(unsigned char *address64,unsigned char *address16,unsigned char type,unsigned char frameID){
	*(dataOUT)='c';
	*(dataOUT+1)=COMMANDS.SET_SETPOINTS;
	*(dataOUT+2)=type;
	*(dataOUT+3)=GET_STATUS;
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,4);
}
void kopterSetpointsReport(unsigned char *address64,unsigned char *address16,unsigned char type,unsigned char frameID){	
	float f=0;
	unsigned char *ch;
	
	*(dataOUT)='r';
	*(dataOUT+1)=COMMANDS.SET_SETPOINTS;
	*(dataOUT+2)=type;
	//TODO add desired
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
		if(		 option==CONTROLLERS.OFF){
			disablePositionController();
			disableVelocityController();			
		}else if(option==CONTROLLERS.POSITION){
			enablePositionController();					
			disableVelocityController();	 
		}else if(option==CONTROLLERS.VELOCITY){
			enableVelocityController();			
			disablePositionController();
		}else if(option==CONTROLLERS.BOTH){
			enableVelocityController();
			enablePositionController();
		}
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
	if(positionControllerEnabled==1 && velocityControllerEnabled==1){
		*(dataOUT+2)=CONTROLLERS.BOTH;
	}else if(positionControllerEnabled){
		*(dataOUT+2)=CONTROLLERS.POSITION;
	}else if(velocityControllerEnabled){
		*(dataOUT+2)=CONTROLLERS.VELOCITY;
	}else{
		*(dataOUT+2)=CONTROLLERS.OFF;
	}
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,3);
}
void kopterControllersReportReceived(unsigned char *address64,unsigned char *address16,unsigned char status){	
	if(		 status==CONTROLLERS.OFF){
		
	}else if(status==CONTROLLERS.POSITION){
		
	}else if(status==CONTROLLERS.VELOCITY){

	}else if(status==CONTROLLERS.BOTH){
		
	}
}

//GUMSTIX
void kopterGumstixRequest(unsigned char *address64,unsigned char *address16,unsigned char options,unsigned char frameID){
	*(dataOUT)='c';
	*(dataOUT+1)=COMMANDS.GUMSTIX;
	*(dataOUT+2)=options;
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,3);
}
void kopterGumstix(unsigned char *address64,unsigned char *address16,unsigned char on){
	if(on){
		enableGumstix();	
	}else{
		disableGumstix();
	}
}
void kopterGumstixStatusRequest(unsigned char *address64,unsigned char *address16,unsigned char frameID){
	*(dataOUT)='c';
	*(dataOUT+1)=COMMANDS.GUMSTIX;
	*(dataOUT+2)=GET_STATUS;
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,3);
}
void kopterGumstixReport(unsigned char *address64,unsigned char *address16,unsigned char frameID){
	*dataOUT='r';
	*(dataOUT+1)=COMMANDS.GUMSTIX;
	if(gumstixEnabled){
		*(dataOUT+2)=ONOFF.ON;
	}else{
		*(dataOUT+2)=ONOFF.OFF;
	}
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,3);	
}
void kopterGumstixReportRecieved(unsigned char *address64,unsigned char *address16,unsigned char status){
	if(			status==ONOFF.ON){
		
	}else if(status==ONOFF.OFF){
		
	}
}