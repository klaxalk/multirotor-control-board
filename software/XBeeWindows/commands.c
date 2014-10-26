#include <stdio.h>
#include "packets.h"
#include "commands.h"
#include "serialLink.h"
#include "main.h"
#include "defines.h"

unsigned char data[20];

//TELEMETRY
void telemetryRequest(unsigned char *address64,unsigned char *address16,unsigned char type,unsigned char options,unsigned char frameID){
    *data='c';
    *(data+1)=type;
    *(data+2)=options;
    makeTRPacket(address64,address16,0x00,frameID,data,3);
    #ifdef DEBUG
     printf("TELEMETRY REQUEST\n");
    #endif // DEBUG
}

void telemetryReceive(unsigned char *address64,unsigned char *address16,unsigned char type,float value){
	if(type==TELEMETRIES.GROUND_DISTANCE){
        printf("Ground Distance: %f\n",value);
	}else if(type==TELEMETRIES.ELEVATOR_SPEED){
        printf("Elevator Speed: %f\n",value);
	}else if(type==TELEMETRIES.AILERON_SPEED){
        printf("Aileron Speed: %f\n",value);
	}else if(type==TELEMETRIES.ELEVATOR_SPEED_ESTIMATED){
        printf("Elevator Speed Est: %f\n",value);
	}else if(type==TELEMETRIES.AILERON_SPEED_ESTIMATED){
        printf("Aileron Speed Est: %f\n",value);
	}else if(type==TELEMETRIES.ELEVATOR_POS_ESTIMATED){
        printf("Elevator Pos Est: %f\n",value);
	}else if(type==TELEMETRIES.AILERON_POS_ESTIMATED){
        printf("Aileron Pos Est: %f\n",value);
	}
}

//GENERAL
void dataTypeError(unsigned char *address64,unsigned char *address16,unsigned char *data){
}

void packetTypeError(unsigned char *inPacket){
    int i;
    printf("Unknown Packet: ");
    for(i=0;i<*(inPacket+2)+4;i++){
        printf("%X ",*(inPacket+i));
    }printf("\n");
}

//LANDING
void kopterLandRequest(unsigned char *address64,unsigned char *address16,unsigned char options,unsigned char frameID){
    *(data)='c';
    *(data+1)=COMMANDS.LANDING;
    *(data+2)=options;
    makeTRPacket(address64,address16,0x00,frameID,data,3);
}

void kopterLandStatusRequest(unsigned char *address64,unsigned char *address16,unsigned char frameID){
		*data='c';
		*(data+1)=COMMANDS.LANDING;
		*(data+2)=GET_STATUS;
		makeTRPacket(address64,address16,0x00,frameID,data,3);
}

void kopterLandReportRecieved(unsigned char *address64,unsigned char *address16,unsigned char status){
    if(status==LANDING.LAND_ON){
        printf("LANDING ON\n");
    }else if(status==LANDING.LAND_OFF){
        printf("LANDING OFF\n");
    }
}

//TRAJECTORY
void kopterTrajectoryRequest(unsigned char *address64,unsigned char *address16,unsigned char options,unsigned char frameID){
	*(data)='c';
	*(data+1)=COMMANDS.TRAJECTORY;
	*(data+2)=options;
	makeTRPacket(address64,address16,0x00,frameID,data,3);
}

void kopterTrajectoryStatusRequest(unsigned char *address64,unsigned char *address16,unsigned char frameID){
	*data='c';
	*(data+1)=COMMANDS.TRAJECTORY;
	*(data+2)=GET_STATUS;
	makeTRPacket(address64,address16,0x00,frameID,data,3);
}

void kopterTrajectoryReportRecieved(unsigned char *address64,unsigned char *address16,unsigned char status){
	if(			status==TRAJECTORY.FOLLOW){
        printf("TRAJECTORY FOLLOWING\n");
	}else if(status==TRAJECTORY.NOT_FOLLOW){
        printf("TRAJECTORY NOT FOLLOWING\n");
	}
}

//TRAJECTORY POINTS

void kopterTrajectoryAddPointRequest(unsigned char *address64,unsigned char *address16,unsigned char index,float time,float elevatorPos,float aileronPos,float throttlePos,unsigned char frameID){
	unsigned char *ch;

	*(data)='c';
	*(data+1)=COMMANDS.TRAJECTORY_POINTS;
	*(data+2)=COMMANDS.TRAJECTORY_POINTS;
	*(data+3)=index;

	ch=(unsigned char *) &time;
	*(data+4)=*ch; *(data+5)=*(ch+1); *(data+6)=*(ch+2); *(data+7)=*(ch+3);

	ch=(unsigned char *) &elevatorPos;
	*(data+8)=*ch; *(data+9)=*(ch+1); *(data+10)=*(ch+2); *(data+11)=*(ch+3);

	ch=(unsigned char *) &aileronPos;
	*(data+12)=*ch; *(data+13)=*(ch+1); *(data+14)=*(ch+2); *(data+15)=*(ch+3);

	ch=(unsigned char *) &throttlePos;
	*(data+16)=*ch; *(data+17)=*(ch+1); *(data+18)=*(ch+2); *(data+19)=*(ch+3);

	makeTRPacket(address64,address16,0x00,frameID,data,20);
}

void kopterTrajectoryPointStatusRequest(unsigned char *address64,unsigned char *address16,unsigned char index,unsigned char frameID){
	unsigned char *d;
	*(data)='c';
	*(data+1)=COMMANDS.TRAJECTORY_POINTS;
	*(data+2)=GET_STATUS;
	*(data+3)=index;
	makeTRPacket(address64,address16,0x00,frameID,data,4);
}

void kopterTrajectoryPointReportReceived(unsigned char *address64,unsigned char *address16,unsigned char index,float time,float elevatorPos,float aileronPos,float throttlePos){
	printf("%i.) T:%.2f El:%.2f Ai:%.2f Th:%.2f\n",index,time,elevatorPos,aileronPos,throttlePos);
}

//SETPOINTS
void kopterSetpointsSetRequest(unsigned char *address64,unsigned char *address16,unsigned char type,unsigned char positionType,float value,unsigned char frameID){
	float f=value;
	unsigned char *ch;

	*(data)='c';
	*(data+1)=COMMANDS.SET_SETPOINTS;
	*(data+2)=type;
	*(data+3)=positionType;

	ch=(unsigned char *) &f;
	*(data+4)=*ch;
	*(data+5)=*(ch+1);
	*(data+6)=*(ch+2);
	*(data+7)=*(ch+3);
	makeTRPacket(address64,address16,0x00,frameID,data,8);
}

void kopterSetpointStatusRequest(unsigned char *address64,unsigned char *address16,unsigned char type,unsigned char frameID){
    *(data)='c';
    *(data+1)=COMMANDS.SET_SETPOINTS;
    *(data+2)=type;
    *(data+3)=GET_STATUS;
    makeTRPacket(address64,address16,0x00,frameID,data,4);
}

void kopterSetpointsReportReceived(unsigned char *address64,unsigned char *address16,unsigned char type,float value){
	if(		 type==SETPOINTS.THROTTLE){
        printf("Throttle setpoint: %f\n",value);
	}else if(type==SETPOINTS.AILERON){
        printf("Aileron setpoint: %f\n",value);
	}else if(type==SETPOINTS.ELEVATOR){
        printf("Elevator setpoint: %f\n",value);
	}
}

//CONTROLLERS
void kopterControllersRequest(unsigned char *address64,unsigned char *address16,unsigned char option,unsigned char frameID){
		*(data)='c';
		*(data+1)=COMMANDS.CONTROLLERS;
		*(data+2)=option;
		makeTRPacket(address64,address16,0x00,frameID,data,3);
}

void kopterControllersStatusRequest(unsigned char *address64,unsigned char *address16,unsigned char frameID){
	*(data)='c';
	*(data+1)=COMMANDS.CONTROLLERS;
	*(data+2)=GET_STATUS;
	makeTRPacket(address64,address16,0x00,frameID,data,3);
}

void kopterControllersReportReceived(unsigned char *address64,unsigned char *address16,unsigned char status){
	if(		 status==CONTROLLERS.OFF){
		printf("Controllers: OFF\n");
	}else if(status==CONTROLLERS.POSITION){
		printf("Controllers: POSITION\n");
	}else if(status==CONTROLLERS.VELOCITY){
        printf("Controllers: VELOCITY\n");
	}
}
