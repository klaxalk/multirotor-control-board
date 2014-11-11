#include <stdio.h>
#include "packets.h"
#include "commands.h"
#include "serialLink.h"
#include "main.h"
#include "defines.h"

unsigned char dataOUT[25];

//TELEMETRY
void telemetryRequest(unsigned char *address64,unsigned char *address16,unsigned char type, unsigned char frameID){
	*dataOUT='c';
	*(dataOUT+1)=COMMANDS.TELEMETRY;
	*(dataOUT+2)=type;
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,3);
    #ifdef DEBUG
     printf("TELEMETRY REQUEST\n");
    #endif // DEBUG
}
void telemetryReceive(unsigned char *address64,unsigned char *address16,unsigned char type,float value){
	if(type==TELEMETRIES.GROUND_DISTANCE_ESTIMATED){
        printf("Ground Distance Est: %f\n",value);
	}else if(type==TELEMETRIES.GROUND_DISTANCE){
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
	}else if(type==TELEMETRIES.THROTTLE_CONTROLLER_OUTPUT){
        printf("Throttle Controller Out: %f\n",value);
	}else if(type==TELEMETRIES.THROTTLE_SPEED){
        printf("Throttle Speed Est: %f\n",value);
	}else if(type==TELEMETRIES.AILERON_POS_CONTROLLER_OUTPUT){
        printf("Pos Aileron Controller Out: %f\n",value);
	}else if(type==TELEMETRIES.ELEVATOR_POS_CONTROLLER_OUTPUT){
        printf("Pos Elevator Controller Out: %f\n",value);
	}else if(type==TELEMETRIES.AILERON_VEL_CONTROLLER_OUTPUT){
        printf("Vel Aileron Controller Out: %f\n",value);
	}else if(type==TELEMETRIES.ELEVATOR_VEL_CONTROLLER_OUTPUT){
        printf("Vel Elevator Controller Out: %f\n",value);
	}else if(type==TELEMETRIES.THROTTLE_SETPOINT){
		printf("Throttle Setpoint: %f\n",value);
	}else if(type==TELEMETRIES.ELEVATOR_POS_SETPOINT){
		printf("Elevator Pos Setpoint: %f\n",value);
	}else if(type==TELEMETRIES.AILERON_POS_SETPOINT){
		printf("Aileron Pos Setpoint: %f\n",value);
	}else if(type==TELEMETRIES.ELEVATOR_VEL_SETPOINT){
		printf("Elevator Vel Setpoint: %f\n",value);
	}else if(type==TELEMETRIES.AILERON_VEL_SETPOINT){
		printf("Aileron Vel Setpoint: %f\n",value);
	}
}

void telemetryToCoordinatorSet(unsigned char *address64,unsigned char *address16,unsigned char type,unsigned char on, unsigned char frameID){
	*dataOUT='c';
	*(dataOUT+1)=COMMANDS.TELEMETRY_COORDINATOR;
	*(dataOUT+2)=type;
	*(dataOUT+3)=on;
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,4);
}
void telemetryToCoordinatorStatusRequest(unsigned char *address64,unsigned char *address16,unsigned char type,unsigned char frameID){
	*dataOUT='c';
	*(dataOUT+1)=COMMANDS.TELEMETRY_COORDINATOR;
	*(dataOUT+2)=GET_STATUS;
	*(dataOUT+3)=type;
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,4);
}
void telemetryToCoordinatorReportRecieved(unsigned char *address64,unsigned char *address16,unsigned char type,unsigned char status){
	printf("Type:%d = %d",type,status);
}

//GENERAL
void dataTypeError(unsigned char *address64,unsigned char *address16,unsigned char *dataOUT){
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
    *(dataOUT)='c';
    *(dataOUT+1)=COMMANDS.LANDING;
    *(dataOUT+2)=options;
    makeTRPacket(address64,address16,0x00,frameID,dataOUT,3);
}
void kopterLandStatusRequest(unsigned char *address64,unsigned char *address16,unsigned char frameID){
		*dataOUT='c';
		*(dataOUT+1)=COMMANDS.LANDING;
		*(dataOUT+2)=GET_STATUS;
		makeTRPacket(address64,address16,0x00,frameID,dataOUT,3);
}
void kopterLandReportRecieved(unsigned char *address64,unsigned char *address16,unsigned char status){
	if(status==LS_LANDING){
        printf("LANDING\n");
	}else if(status==LS_FLIGHT){
        printf("FLIGHT\n");
	}else if(status==LS_STABILIZATION){
        printf("STABILIZATION\n");
	}else if(status==LS_ON_GROUND){
        printf("ON GROUND\n");
	}else if(status==LS_TAKEOFF){
        printf("TAKE OFF\n");
	}
}

//TRAJECTORY
void kopterTrajectoryRequest(unsigned char *address64,unsigned char *address16,unsigned char options,unsigned char frameID){
	*(dataOUT)='c';
	*(dataOUT+1)=COMMANDS.TRAJECTORY_FOLLOW;
	*(dataOUT+2)=options;
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,3);
}
void kopterTrajectoryStatusRequest(unsigned char *address64,unsigned char *address16,unsigned char frameID){
	*dataOUT='c';
	*(dataOUT+1)=COMMANDS.TRAJECTORY_FOLLOW;
	*(dataOUT+2)=GET_STATUS;
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,3);
}
void kopterTrajectoryReportRecieved(unsigned char *address64,unsigned char *address16,unsigned char status){
	if(			status==ONOFF.ON){
        printf("TRAJECTORY FOLLOWING\n");
	}else if(status==ONOFF.OFF){
        printf("TRAJECTORY NOT FOLLOWING\n");
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
void kopterTrajectoryPointStatusRequest(unsigned char *address64,unsigned char *address16,unsigned char frameID){
	*(dataOUT)='c';
	*(dataOUT+1)=COMMANDS.TRAJECTORY_POINTS;
	*(dataOUT+2)=GET_STATUS;
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,4);
}
void kopterTrajectoryPointReportReceived(unsigned char *address64,unsigned char *address16,unsigned char index,float time,float elevatorPos,float aileronPos,float throttlePos){
	printf("%i.) T:%.2f El:%.2f Ai:%.2f Th:%.2f\n",index,time,elevatorPos,aileronPos,throttlePos);
}

//SETPOINTS
void kopterSetpointsSetRequest(unsigned char *address64,unsigned char *address16,unsigned char type,unsigned char positionType,float value,unsigned char frameID){
	float f=value;
	unsigned char *ch;

	*(dataOUT)='c';
	*(dataOUT+1)=COMMANDS.SET_SETPOINTS;
	*(dataOUT+2)=type;
	*(dataOUT+3)=positionType;

	ch=(unsigned char *) &f;
	*(dataOUT+4)=*ch;
	*(dataOUT+5)=*(ch+1);
	*(dataOUT+6)=*(ch+2);
	*(dataOUT+7)=*(ch+3);
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,8);
}
void kopterSetpointStatusRequest(unsigned char *address64,unsigned char *address16,unsigned char type,unsigned char frameID){
    *(dataOUT)='c';
    *(dataOUT+1)=COMMANDS.SET_SETPOINTS;
    *(dataOUT+2)=type;
    *(dataOUT+3)=GET_STATUS;
    makeTRPacket(address64,address16,0x00,frameID,dataOUT,4);
}
void kopterSetpointsReportReceived(unsigned char *address64,unsigned char *address16,unsigned char type,float value){
	if(		 type==SETPOINTS.THROTTLE){
        printf("Throttle setpoint: %f\n",value);
	}else if(type==SETPOINTS.AILERON_POSITION){
        printf("Aileron Pos setpoint: %f\n",value);
	}else if(type==SETPOINTS.ELEVATOR_POSITION){
        printf("Elevator Pos setpoint: %f\n",value);
	}else if(type==SETPOINTS.AILERON_VELOCITY){
        printf("Aileron Vel setpoint: %f\n",value);
	}else if(type==SETPOINTS.ELEVATOR_VELOCITY){
        printf("Elevator Vel setpoint: %f\n",value);
	}
}

//CONTROLLERS
void kopterControllersRequest(unsigned char *address64,unsigned char *address16,unsigned char option,unsigned char frameID){
		*(dataOUT)='c';
		*(dataOUT+1)=COMMANDS.CONTROLLERS;
		*(dataOUT+2)=option;
		makeTRPacket(address64,address16,0x00,frameID,dataOUT,3);
}
void kopterControllersStatusRequest(unsigned char *address64,unsigned char *address16,unsigned char frameID){
	*(dataOUT)='c';
	*(dataOUT+1)=COMMANDS.CONTROLLERS;
	*(dataOUT+2)=GET_STATUS;
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,3);
}
void kopterControllersReportReceived(unsigned char *address64,unsigned char *address16,unsigned char status){
	if(		 status==CONTROLLERS.OFF){
		printf("Controllers: OFF\n");
	}else if(status==CONTROLLERS.POSITION){
		printf("Controllers: POSITION\n");
	}else if(status==CONTROLLERS.VELOCITY){
        printf("Controllers: VELOCITY\n");
	}else if(status==CONTROLLERS.BOTH){
		printf("Controllers: BOTH\n");
	}
}

//GUMSTIX
void kopterGumstixRequest(unsigned char *address64,unsigned char *address16,unsigned char options,unsigned char frameID){
	*(dataOUT)='c';
	*(dataOUT+1)=COMMANDS.GUMSTIX;
	*(dataOUT+2)=options;
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,3);
}
void kopterGumstixStatusRequest(unsigned char *address64,unsigned char *address16,unsigned char frameID){
	*(dataOUT)='c';
	*(dataOUT+1)=COMMANDS.GUMSTIX;
	*(dataOUT+2)=GET_STATUS;
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,3);
}
void kopterGumstixReportRecieved(unsigned char *address64,unsigned char *address16,unsigned char status){
	if(status==ONOFF.ON){
        printf("GUMSTIX ENABLED\n");
	}else if(status==ONOFF.OFF){
        printf("GUMSTIX DISABLED\n");
	}
}
//
