#include <stdio.h>
#include "packets.h"
#include "commands.h"
#include "serialLink.h"
#include "serial.h"
#include "defines.h"
#include "receive.h"
#include "const.h"

unsigned char dataOUT[25];

float Telemetry[KOPTERS_COUNT][TELEMETRY_VARIABLES]={0};
unsigned char Reports[KOPTERS_COUNT][REPORTS_COUNT]={0};


int addrEqual(unsigned char *adr1,unsigned char *adr2){
    int i;
    for (i=0;i<8;i++){
        if(*(adr1+i)!=*(adr2+i)) return 0;
    }
    return 1;
}
unsigned char getKopter(unsigned char *adr){
    unsigned char kopter=KOPTERS.UNKNOWN;
    if(addrEqual(adr,ADDRESS.K1)){kopter=KOPTERS.K1;}
    if(addrEqual(adr,ADDRESS.K2)){kopter=KOPTERS.K2;}
    if(addrEqual(adr,ADDRESS.K3)){kopter=KOPTERS.K3;}
    if(addrEqual(adr,ADDRESS.KC1)){kopter=KOPTERS.KC1;}
    return kopter;
}
unsigned char mapReports(unsigned char repIndicator){
    if(repIndicator==COMMANDS.LANDING){return 0x01;}
    else if(repIndicator==COMMANDS.CONTROLLERS){return 0x02;}
    else if(repIndicator==COMMANDS.TRAJECTORY_FOLLOW){return 0x03;}
    else if(repIndicator==COMMANDS.GUMSTIX){return 0x04;}
    else return 0x00;
}

//GET ARRAYS
float getTelemetry(unsigned char kopter, unsigned char type){
    return Telemetry[kopter][type];
}
unsigned char getStatus(unsigned char kopter, unsigned char type){
    return Reports[kopter][mapReports(type)];
}

//TELEMETRY
void telemetryRequest(unsigned char *address64,unsigned char *address16,unsigned char type, unsigned char frameID){
	*dataOUT='c';
	*(dataOUT+1)=COMMANDS.TELEMETRY;
	*(dataOUT+2)=type;
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,3);
}
void telemetryReceive(unsigned char *address64,unsigned char *address16,unsigned char type,float value){
    unsigned char kopter=getKopter(address64);

    if(kopter!=KOPTERS.UNKNOWN){
        Telemetry[kopter][type]=value;
        receivedTelemetry(kopter,type,value);
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
    unsigned char kopter=getKopter(address64);

    if(kopter!=KOPTERS.UNKNOWN){
        receivedTelToCoordReport(kopter,type,status);
    }
}

//GENERAL
void dataTypeError(unsigned char *address64,unsigned char *address16,unsigned char *dataOUT){
}
void packetTypeError(unsigned char *inPacket){
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
    unsigned char kopter=getKopter(address64);
    if(kopter!=KOPTERS.UNKNOWN){
        Reports[kopter][mapReports(COMMANDS.LANDING)]=status;
        receivedLandingReport(kopter,status);
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
    unsigned char kopter=getKopter(address64);
    if(kopter!=KOPTERS.UNKNOWN){
        Reports[kopter][mapReports(COMMANDS.TRAJECTORY_FOLLOW)]=status;
        receivedTrajectoryFollowReport(kopter,status);
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
void kopterTrajectoryPointStatusRequest(unsigned char *address64,unsigned char *address16,unsigned char frameID){
	*(dataOUT)='c';
	*(dataOUT+1)=COMMANDS.TRAJECTORY_POINTS;
	*(dataOUT+2)=GET_STATUS;
	makeTRPacket(address64,address16,0x00,frameID,dataOUT,4);
}
void kopterTrajectoryPointReportReceived(unsigned char *address64,unsigned char *address16,unsigned char index,float time,float elevatorPos,float aileronPos,float throttlePos){
    unsigned char kopter=getKopter(address64);
    if(kopter!=KOPTERS.UNKNOWN){
        receivedTrajectoryPointReport(kopter,index,time,elevatorPos,aileronPos,throttlePos);
    }
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
    *(dataOUT+2)=GET_STATUS;
    *(dataOUT+3)=type;
    makeTRPacket(address64,address16,0x00,frameID,dataOUT,4);
}
void kopterSetpointsReportReceived(unsigned char *address64,unsigned char *address16,unsigned char type,float value){
    unsigned char kopter=getKopter(address64);
    if(kopter!=KOPTERS.UNKNOWN){
        receivedDesiredSetpointReport(kopter,type,value);
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
    unsigned char kopter=getKopter(address64);
    if(kopter!=KOPTERS.UNKNOWN){
        Reports[kopter][mapReports(COMMANDS.CONTROLLERS)]=status;
        receivedControllerReport(kopter,status);
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
    unsigned char kopter=getKopter(address64);
    if(kopter!=KOPTERS.UNKNOWN){
        Reports[kopter][mapReports(COMMANDS.GUMSTIX)]=status;
        receivedGumstixReport(kopter,status);
    }
}


