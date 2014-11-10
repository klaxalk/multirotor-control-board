#include <stdio.h>
#include "packets.h"
#include "commands.h"
#include "communication.h"
#include "controllers.h"
#include "system.h"

extern volatile unsigned char landingRequest;

//system.c 
//variables for gestures
extern volatile int16_t gestureElevatorOutput;
extern volatile int16_t gestureAileronOutput;
extern volatile int16_t gestureThrottleOutput;
extern volatile int16_t gestureRudderOutput;
extern volatile unsigned char gestured;

unsigned char data[20];


//TELEMETRY
void telemetrySend(unsigned char *address64,unsigned char *address16,unsigned char type,unsigned char frameID){
	float f=0;
	unsigned char *d;
	
	if(type==TELEMETRIES.GROUND_DISTANCE){
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
	}
	
	d=(unsigned char *) &f;
	*data='t';
	*(data+1)=type;
	*(data+2)=*d;
	*(data+3)=*(d+1);
	*(data+4)=*(d+2);
	*(data+5)=*(d+3);
	makeTRPacket(address64,address16,0x00,frameID,data,6);
}
void telemetryRequest(unsigned char *address64,unsigned char *address16,unsigned char type,unsigned char options, unsigned char frameID){
	*data='c';
	*(data+1)=type;
	*(data+2)=options;
	makeTRPacket(address64,address16,0x00,frameID,data,3);
}
void telemetryReceive(unsigned char *address64,unsigned char *address16,unsigned char type,float value){
	if(type==TELEMETRIES.GROUND_DISTANCE){
		
	}else if(type==TELEMETRIES.ELEVATOR_SPEED){
		
	}else if(type==TELEMETRIES.AILERON_SPEED){
	
	}else if(type==TELEMETRIES.ELEVATOR_SPEED_ESTIMATED){
	
	}else if(type==TELEMETRIES.AILERON_SPEED_ESTIMATED){
	
	}else if(type==TELEMETRIES.ELEVATOR_POS_ESTIMATED){
	
	}else if(type==TELEMETRIES.AILERON_POS_ESTIMATED){
	
	}
}

//GENERAL
void dataTypeError(unsigned char *address64,unsigned char *address16,unsigned char *data){
}
void packetTypeError(unsigned char *inPacket){
}


//LANDING
void kopterLandRequest(unsigned char *address64,unsigned char *address16,unsigned char options,unsigned char frameID){
	*(data)='c';
	*(data+1)=COMMANDS.LANDING;
	*(data+2)=options;
	makeTRPacket(address64,address16,0x00,frameID,data,3);
}
void kopterLand(unsigned char *address64,unsigned char *address16,unsigned char on){
	landingRequest=on;
}
void kopterLandStatusRequest(unsigned char *address64,unsigned char *address16,unsigned char frameID){
	*data='c';
	*(data+1)=COMMANDS.LANDING;
	*(data+2)=GET_STATUS;
	makeTRPacket(address64,address16,0x00,frameID,data,3);
}
void kopterLandReport(unsigned char *address64,unsigned char *address16,unsigned char frameID){
	*data='r';
	*(data+1)=COMMANDS.LANDING;
	if(landingRequest){
		*(data+2)=LANDING.LAND_ON;
		}else{
		*(data+2)=LANDING.LAND_OFF;
	}
	makeTRPacket(address64,address16,0x00,frameID,data,3);
}
void kopterLandReportRecieved(unsigned char *address64,unsigned char *address16,unsigned char status){
	if(status==LANDING.LAND_ON){
		
		}else if(status==LANDING.LAND_OFF){
		
	}
}

//TRAJECTORY
void kopterTrajectoryRequest(unsigned char *address64,unsigned char *address16,unsigned char options,unsigned char frameID){
	*(data)='c';
	*(data+1)=COMMANDS.TRAJECTORY;
	*(data+2)=options;
	makeTRPacket(address64,address16,0x00,frameID,data,3);
}
void kopterTrajectory(unsigned char *address64,unsigned char *address16,unsigned char on){
	trajectoryEnabled=on;
}
void kopterTrajectoryStatusRequest(unsigned char *address64,unsigned char *address16,unsigned char frameID){
	*data='c';
	*(data+1)=COMMANDS.TRAJECTORY;
	*(data+2)=GET_STATUS;
	makeTRPacket(address64,address16,0x00,frameID,data,3);
}
void kopterTrajectoryReport(unsigned char *address64,unsigned char *address16,unsigned char frameID){
	*data='r';
	*(data+1)=COMMANDS.TRAJECTORY;
	if(trajectoryEnabled){
		*(data+2)=TRAJECTORY.FOLLOW;
		}else{
		*(data+2)=TRAJECTORY.NOT_FOLLOW;
	}
	makeTRPacket(address64,address16,0x00,frameID,data,3);
}
void kopterTrajectoryReportRecieved(unsigned char *address64,unsigned char *address16,unsigned char status){
	if(			status==TRAJECTORY.FOLLOW){
		
		}else if(status==TRAJECTORY.NOT_FOLLOW){
		
	}
}

//TRAJECTORY POINTS
void kopterTrajectoryAddPointRequest(unsigned char *address64,unsigned char *address16,unsigned char index,float time,float elevatorPos,float aileronPos,float throttlePos,unsigned char frameID){
	unsigned char *d;
	
	*(data)='c';
	*(data+1)=COMMANDS.TRAJECTORY_POINTS;
	*(data+2)=COMMANDS.TRAJECTORY_POINTS;
	*(data+3)=index;
	
	d=(unsigned char *) &time;
	*(data+4)=*d; *(data+5)=*(d+1); *(data+6)=*(d+2); *(data+7)=*(d+3);
	
	d=(unsigned char *) &elevatorPos;
	*(data+8)=*d; *(data+9)=*(d+1); *(data+10)=*(d+2); *(data+11)=*(d+3);
	
	d=(unsigned char *) &aileronPos;
	*(data+12)=*d; *(data+13)=*(d+1); *(data+14)=*(d+2); *(data+15)=*(d+3);	
	
	d=(unsigned char *) &throttlePos;
	*(data+16)=*d; *(data+17)=*(d+1); *(data+18)=*(d+2); *(data+19)=*(d+3);	
		
	makeTRPacket(address64,address16,0x00,frameID,data,20);
}
void kopterTrajectoryAddPoint(unsigned char *address64,unsigned char *address16,unsigned char index,float time,float elevatorPos,float aileronPos,float throttlePos){
	if(index==0){
		if(trajMaxIndex<TRAJECTORY_LENGTH){
			trajMaxIndex++;
			TRAJ_POINT(trajMaxIndex-1,time,elevatorPos,aileronPos,throttlePos);
		}
	}else{
		if(index<=TRAJECTORY_LENGTH){
			if(index>trajMaxIndex){
				trajMaxIndex=index;
			}
			TRAJ_POINT(index-1,time,elevatorPos,aileronPos,throttlePos);
		}
	}
}
void kopterTrajectoryPointStatusRequest(unsigned char *address64,unsigned char *address16,unsigned char index,unsigned char frameID){
	unsigned char *d;
	*(data)='c';
	*(data+1)=COMMANDS.TRAJECTORY_POINTS;
	*(data+2)=GET_STATUS;
	*(data+3)=index;
	makeTRPacket(address64,address16,0x00,frameID,data,4);
}
void kopterTrajectoryPointReport(unsigned char *address64,unsigned char *address16,unsigned char index,unsigned char frameID){
	unsigned char *d;
	
	*(data)='r';
	*(data+1)=COMMANDS.TRAJECTORY_POINTS;
	*(data+2)=index;
	
	d=(unsigned char *) &trajectory[index-1].time;
	*(data+3)=*d; *(data+4)=*(d+1); *(data+5)=*(d+2); *(data+6)=*(d+3);
	
	d=(unsigned char *) &trajectory[index-1].elevatorPos;
	*(data+7)=*d; *(data+8)=*(d+1); *(data+9)=*(d+2); *(data+10)=*(d+3);
	
	d=(unsigned char *) &trajectory[index-1].aileronPos;
	*(data+11)=*d; *(data+12)=*(d+1); *(data+13)=*(d+2); *(data+14)=*(d+3);
	
	d=(unsigned char *) &trajectory[index-1].throttlePos;
	*(data+15)=*d; *(data+16)=*(d+1); *(data+17)=*(d+2); *(data+18)=*(d+3);
	
	makeTRPacket(address64,address16,0x00,frameID,data,19);
}
void kopterTrajectoryPointReportReceived(unsigned char *address64,unsigned char *address16,unsigned char index,float time,float elevatorPos,float aileronPos,float throttlePos){
	
}

//SETPOINTS
void kopterSetpointsSetRequest(unsigned char *address64,unsigned char *address16,unsigned char type,unsigned char positionType,float value,unsigned char frameID){
	unsigned char *d;
			
	*(data)='c';
	*(data+1)=COMMANDS.SET_SETPOINTS;
	*(data+2)=type;
	*(data+3)=positionType;
	
	d=(unsigned char *) &value;
	*(data+4)=*d;
	*(data+5)=*(d+1);
	*(data+6)=*(d+2);
	*(data+7)=*(d+3);
	makeTRPacket(address64,address16,0x00,frameID,data,8);
}
void kopterSetpointsSet(unsigned char *address64,unsigned char *address16,unsigned char type,unsigned char positionType,float value){
	//positions setpoints
		if(type==SETPOINTS.THROTTLE){
			if(		 positionType==POSITIONS.ABSOLUT){
				throttleDesiredSetpoint=value;
			}else if(positionType==POSITIONS.RELATIV){
				throttleDesiredSetpoint+=value;
			}
			
		}else if(type==SETPOINTS.ELEVATOR){
			if(			 positionType==POSITIONS.ABSOLUT){
					elevatorDesiredSetpoint=value;
				}else if(positionType==POSITIONS.RELATIV){
					elevatorDesiredSetpoint+=value;
			}		
				
		}else if(type==SETPOINTS.AILERON){
			if(			 positionType==POSITIONS.ABSOLUT){
					aileronDesiredSetpoint=value;
				}else if(positionType==POSITIONS.RELATIV){
					aileronDesiredSetpoint+=value;
			}			
		}		
}
void kopterSetpointStatusRequest(unsigned char *address64,unsigned char *address16,unsigned char type,unsigned char frameID){
	*(data)='c';
	*(data+1)=COMMANDS.SET_SETPOINTS;
	*(data+2)=type;
	*(data+3)=GET_STATUS;
	makeTRPacket(address64,address16,0x00,frameID,data,4);
}
void kopterSetpointsReport(unsigned char *address64,unsigned char *address16,unsigned char type,unsigned char frameID){	
	float f=0;
	unsigned char *d;
	
	*(data)='r';
	*(data+1)=COMMANDS.SET_SETPOINTS;
	*(data+2)=type;
	
	if(type==SETPOINTS.THROTTLE){
		f=throttleSetpoint;
	}else if(type==SETPOINTS.ELEVATOR){
		f=elevatorSetpoint;
	}else if(type==SETPOINTS.AILERON){
		f=aileronSetpoint;
	}
	
	
	d=(unsigned char *) &f;
	*(data+3)=*d;
	*(data+4)=*(d+1);
	*(data+5)=*(d+2);
	*(data+6)=*(d+3);
	makeTRPacket(address64,address16,0x00,frameID,data,7);
}
void kopterSetpointsReportReceived(unsigned char *address64,unsigned char *address16,unsigned char type,float value){
	if(		 type==SETPOINTS.THROTTLE){
		
	}else if(type==SETPOINTS.AILERON){
	
	}else if(type==SETPOINTS.ELEVATOR){

	}	
}

//CONTROLLERS
void kopterControllersRequest(unsigned char *address64,unsigned char *address16,unsigned char option,unsigned char frameID){
		*(data)='c';
		*(data+1)=COMMANDS.CONTROLLERS;
		*(data+2)=option;
		makeTRPacket(address64,address16,0x00,frameID,data,3);	
}
void kopterControllers(unsigned char *address64,unsigned char *address16,unsigned char option){
		if(		 option==CONTROLLERS.OFF){
			disablePositionController();
			disableController();			
		}else if(option==CONTROLLERS.POSITION){
			enableController();
			enablePositionController();
		}else if(option==CONTROLLERS.VELOCITY){
			disablePositionController();
			enableController();
		}
}
void kopterControllersStatusRequest(unsigned char *address64,unsigned char *address16,unsigned char frameID){
	*(data)='c';
	*(data+1)=COMMANDS.CONTROLLERS;
	*(data+2)=GET_STATUS;
	makeTRPacket(address64,address16,0x00,frameID,data,3);
}
void kopterControllersReport(unsigned char *address64,unsigned char *address16,unsigned char frameID){
	*(data)='r';
	*(data+1)=COMMANDS.CONTROLLERS;		
	
	if(positionControllerEnabled){
		*(data+2)=CONTROLLERS.POSITION;
	}else if(controllerEnabled){
		*(data+2)=CONTROLLERS.VELOCITY;
	}else{
		*(data+2)=CONTROLLERS.OFF;
	}
	makeTRPacket(address64,address16,0x00,frameID,data,3);
}
void kopterControllersReportReceived(unsigned char *address64,unsigned char *address16,unsigned char status){	
	if(		 status==CONTROLLERS.OFF){
		
	}else if(status==CONTROLLERS.POSITION){
		
	}else if(status==CONTROLLERS.VELOCITY){

	}	
}

//GESTURES
void kopterGestureRequest(unsigned char *address64,unsigned char *address16,unsigned char type,unsigned char frameID){
	*(data)='c';
	*(data+1)=COMMANDS.GESTURES;
	*(data+2)=type;
	makeTRPacket(address64,address16,0x00,frameID,data,3);
}
void kopterGesture(unsigned char *address64,unsigned char *address16,unsigned char type){
	if(		 type==GESTURES.ARM){
		gestureElevatorOutput=PULSE_OUT_MIN;		
		gestureRudderOutput=PULSE_OUT_MAX;
		gestureAileronOutput=PULSE_OUT_MIDDLE;
		gestureThrottleOutput=PULSE_OUT_MIN;
		gestured=0;
	}else if(type==GESTURES.DISARM){
		gestureElevatorOutput=PULSE_OUT_MIN;
		gestureRudderOutput=PULSE_OUT_MIN;
		gestureAileronOutput=PULSE_OUT_MIDDLE;
		gestureThrottleOutput=PULSE_OUT_MIN;
		gestured=0;
	}
}

