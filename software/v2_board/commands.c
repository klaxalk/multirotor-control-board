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
	if(type==TELEMETRIES.ALTITUDE_ESTIMATED){
		f=position.altitude;
	}else if(type==TELEMETRIES.ALTITUDE){
		f=groundDistance;
	}else if(type==TELEMETRIES.ELEVATOR_SPEED){
		f=elevatorSpeed;
	}else if(type==TELEMETRIES.AILERON_SPEED){
		f=aileronSpeed;
	}else if(type==TELEMETRIES.ELEVATOR_SPEED_ESTIMATED){
		f=speed.elevator;
	}else if(type==TELEMETRIES.AILERON_SPEED_ESTIMATED){
		f=speed.aileron;
	}else if(type==TELEMETRIES.ELEVATOR_POSITION){
		f=position.elevator;
	}else if(type==TELEMETRIES.AILERON_POSITION){
		f=position.aileron;
	}else if(type==TELEMETRIES.ALTITUDE_CONTROLLER_OUTPUT){
		f=controllerThrottleOutput;
	}else if(type==TELEMETRIES.ALTITUDE_SPEED){
		f=speed.altitude;
	}else if(type==TELEMETRIES.AILERON_CONTROLLER_OUTPUT){
		f=controllerAileronOutput;
	}else if(type==TELEMETRIES.ELEVATOR_CONTROLLER_OUTPUT){
		f=controllerElevatorOutput;
	}else if(type==TELEMETRIES.ALTITUDE_SETPOINT){
		f=positionSetpoint.altitude;
	}else if(type==TELEMETRIES.ELEVATOR_POS_SETPOINT){
		f=positionSetpoint.elevator;
	}else if(type==TELEMETRIES.AILERON_POS_SETPOINT){
		f=positionSetpoint.aileron;
	}else if(type==TELEMETRIES.ELEVATOR_SPEED_SETPOINT){
		f=speedSetpoint.elevator;
	}else if(type==TELEMETRIES.AILERON_SPEED_SETPOINT){
		f=speedSetpoint.aileron;
	}else if(type==TELEMETRIES.ELEVATOR_ACC){
		f=acceleration.elevator;
	}else if(type==TELEMETRIES.AILERON_ACC){
		f=acceleration.aileron;
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
    }else if(type==TELEMETRIES.BLOB_ELEVATOR){
		f=blob.elevator;
    }else if(type==TELEMETRIES.BLOB_AILERON){
		f=blob.aileron;
    }else if(type==TELEMETRIES.BLOB_ALTITUDE){
		f=blob.altitude;
    }else if(type==TELEMETRIES.PITCH_ANGLE){
		f=(float)pitchAngle/10.0;
    }else if(type==TELEMETRIES.ROLL_ANGLE){
		f=(float)rollAngle/10.0;
    }
	return f;
}
void telemetryReceive(unsigned char *address64,unsigned char *address16,unsigned char type,float value){
	if(type==TELEMETRIES.ALTITUDE_ESTIMATED){
		
		}else if(type==TELEMETRIES.ALTITUDE){
		
		}else if(type==TELEMETRIES.ELEVATOR_SPEED){
		
		}else if(type==TELEMETRIES.AILERON_SPEED){
		
		}else if(type==TELEMETRIES.ELEVATOR_SPEED_ESTIMATED){
		
		}else if(type==TELEMETRIES.AILERON_SPEED_ESTIMATED){
		
		}else if(type==TELEMETRIES.ELEVATOR_POSITION){
		
		}else if(type==TELEMETRIES.AILERON_POSITION){
		
		}else if(type==TELEMETRIES.ALTITUDE_CONTROLLER_OUTPUT){
		
		}else if(type==TELEMETRIES.ALTITUDE_SPEED){
		
		}else if(type==TELEMETRIES.AILERON_CONTROLLER_OUTPUT){
		
		}else if(type==TELEMETRIES.ELEVATOR_CONTROLLER_OUTPUT){
		
		}else if(type==TELEMETRIES.ALTITUDE_SETPOINT){
		
		}else if(type==TELEMETRIES.ELEVATOR_POS_SETPOINT){

		}else if(type==TELEMETRIES.AILERON_POS_SETPOINT){
		
		}else if(type==TELEMETRIES.ELEVATOR_SPEED_SETPOINT){
		
		}else if(type==TELEMETRIES.AILERON_SPEED_SETPOINT){

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

//TRAJECTORY POINTS
void kopterTrajectorySetRequest(unsigned char *address64,unsigned char *address16,unsigned char size,uint32_t* time,float* elevatorPos,float* aileronPos,float* throttlePos,unsigned char frameID){
	unsigned char *ch;
	uint8_t i;
	
	*(dataOUT)='c';
	*(dataOUT+1)=COMMANDS.TRAJECTORY_POINTS;
	*(dataOUT+2)=size;
	for(i=0;i<size;i++){		
		ch=(unsigned char *) time+i;
		*(dataOUT+3+i*16)=*ch; *(dataOUT+4+i*16)=*(ch+1); *(dataOUT+5+i*16)=*(ch+2); *(dataOUT+6+i*16)=*(ch+3);
	
		ch=(unsigned char *) elevatorPos+i;
		*(dataOUT+7+i*16)=*ch; *(dataOUT+8+i*16)=*(ch+1); *(dataOUT+9+i*16)=*(ch+2); *(dataOUT+10+i*16)=*(ch+3);
	
		ch=(unsigned char *) aileronPos+i;
		*(dataOUT+11+i*16)=*ch; *(dataOUT+12+i*16)=*(ch+1); *(dataOUT+13+i*16)=*(ch+2); *(dataOUT+14+i*16)=*(ch+3);	
	
		ch=(unsigned char *) throttlePos+i;
		*(dataOUT+15+i*16)=*ch; *(dataOUT+16+i*16)=*(ch+1); *(dataOUT+17+i*16)=*(ch+2); *(dataOUT+18+i*16)=*(ch+3);	
	}
		
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,size*16+3);
}
void kopterTrajectorySet(unsigned char *address64,unsigned char *address16,unsigned char index,uint32_t time,float elevatorPos,float aileronPos,float throttlePos){
	if(index<TRAJECTORY_LENGTH){
		trajMaxIndex=index;
		trajIndex=0;
		TRAJ_POINT(index,time,elevatorPos,aileronPos,throttlePos);
	}		
}
void kopterTrajectorySetStatusRequest(unsigned char *address64,unsigned char *address16,unsigned char frameID){
	*(dataOUT)='c';
	*(dataOUT+1)=COMMANDS.TRAJECTORY_POINTS;
	*(dataOUT+2)=GET_STATUS;
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,4);
}
void kopterTrajectorySetReport(unsigned char *address64,unsigned char *address16,unsigned char index,unsigned char frameID){
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
void kopterTrajectorySetReportReceived(unsigned char *address64,unsigned char *address16,unsigned char index,uint32_t time,float elevatorPos,float aileronPos,float throttlePos){
	
}

//POSITION SLAVE SET
void kopterPositionSlaveSetRequest(unsigned char *address64,unsigned char *address16,unsigned char *slaveAddr,unsigned char frameID){
	char i;
	*(dataOUT)='c';
	*(dataOUT+1)=COMMANDS.POSITION_SLAVE_SET;
	for (i=0;i<8;i++){
		*(dataOUT+2+i)=*(slaveAddr+i);
	}
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,10);
}
void kopterPositionSlaveSet(unsigned char *address64,unsigned char *address16,unsigned char *slaveAddr){
	char i;
	for(i=0;i<8;i++){
		*(posSlave+i)=*(slaveAddr+i);
	}
}
void kopterPositionSlaveSetStatusRequest(unsigned char *address64,unsigned char *address16,unsigned char frameID){
	*(dataOUT)='c';
	*(dataOUT+1)=COMMANDS.POSITION_SLAVE_SET;
	*(dataOUT+2)=GET_STATUS;
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,3);
}
void kopterPositionSlaveSetReport(unsigned char *address64,unsigned char *address16,unsigned char frameID){
	char i;
	*(dataOUT)='r';
	*(dataOUT+1)=COMMANDS.POSITION_SLAVE_SET;
	for (i=0;i<8;i++){
		*(dataOUT+2+i)=*(posSlave+i);
	}
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,10);
}
void kopterPositionSlaveSetReportRecieved(unsigned char *address64,unsigned char *address16,unsigned char *slaveAddr){
	
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

//TIME
void kopterTimeRequest(unsigned char *address64,unsigned char *address16,uint32_t time,unsigned char frameID){
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
void kopterTime(unsigned char *address64,unsigned char *address16,uint32_t time){	
	secondsTimer=time;
	milisecondsTimer=0;
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
void kopterTimeReportReceived(unsigned char *address64,unsigned char *address16,uint32_t time){
	
}

void kopterPositionSetRequest(unsigned char *address64,unsigned char *address16,float elevator,float aileron,unsigned char frameID){
	unsigned char *ch;
	
	*(dataOUT)='c';
	*(dataOUT+1)=COMMANDS.POSITION_SET;
	
	ch=(unsigned char *) &elevator;
	*(dataOUT+2)=*ch;
	*(dataOUT+3)=*(ch+1);
	*(dataOUT+4)=*(ch+2);
	*(dataOUT+5)=*(ch+3);

	ch=(unsigned char *) &aileron;
	*(dataOUT+6)=*ch;
	*(dataOUT+7)=*(ch+1);
	*(dataOUT+8)=*(ch+2);
	*(dataOUT+9)=*(ch+3);	
	
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,10);		
}
void kopterPositionSet(unsigned char *address64,unsigned char *address16,float elevator,float aileron){
	position.elevator=elevator;
	position.aileron=aileron;
}