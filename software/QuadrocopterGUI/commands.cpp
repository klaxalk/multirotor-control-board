#include <stdio.h>
#include "packets.h"
#include "commands.h"
#include "serialLink.h"
#include "serial.h"
#include "defines.h"
#include "receive.h"
#include "const.h"

unsigned char dataOUT[200];
int trajMaxIndex;

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
   // return Telemetry[kopter][type];

    if(kopter==KOPTERS.K1){printf("cteni1\n");return Telemetry[KOPTERS.K1][type];}
    if(kopter==KOPTERS.K2){printf("cteni2\n");return Telemetry[KOPTERS.K2][type];printf("cteni2\n");}
    if(kopter==KOPTERS.K3){printf("cteni3\n");return Telemetry[KOPTERS.K3][type];printf("cteni3\n");}
    if(kopter==KOPTERS.KC1){printf("cteni4\n");return Telemetry[KOPTERS.KC1][type];printf("cteni4\n");}
}
unsigned char getStatus(unsigned char kopter, unsigned char type){
    return Reports[kopter][mapReports(type)];
}

void telemetryReceive(unsigned char *address64,unsigned char *address16,unsigned char type,float value){
    unsigned char kopter=getKopter(address64);

    if(kopter!=KOPTERS.UNKNOWN){
        Telemetry[kopter][type]=value;
        receivedTelemetry(kopter,type,value);
    }

    if(addrEqual(address64,ADDRESS.K1)){Telemetry[KOPTERS.K1][type]=value;printf("1\n");}
    if(addrEqual(address64,ADDRESS.K2)){Telemetry[KOPTERS.K2][type]=value;printf("2\n");}
    if(addrEqual(address64,ADDRESS.K3)){Telemetry[KOPTERS.K3][type]=value;printf("3\n");}
    if(addrEqual(address64,ADDRESS.KC1)){Telemetry[KOPTERS.KC1][type]=value;printf("4\n");}
}

//GENERAL
void dataTypeError(unsigned char *address64,unsigned char *address16,unsigned char *dataOUT){
}
void packetTypeError(unsigned char *inPacket){
}

//LANDING
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


//TELEMETRY
/*float telemetryValue(unsigned char type){
    float f=0.0;
    if(type==TELEMETRIES.ALTITUDE_ESTIMATED){
        f=altitude.position;
    }else if(type==TELEMETRIES.ALTITUDE){
        f=groundDistance;
    }else if(type==TELEMETRIES.ELEVATOR_SPEED){
        f=elevatorSpeed;
    }else if(type==TELEMETRIES.AILERON_SPEED){
        f=aileronSpeed;
    }else if(type==TELEMETRIES.ELEVATOR_SPEED_ESTIMATED){
        f=kalmanStates.elevator.velocity;
    }else if(type==TELEMETRIES.AILERON_SPEED_ESTIMATED){
        f=kalmanStates.aileron.velocity;
    }else if(type==TELEMETRIES.ELEVATOR_POSITION){
        f=kalmanStates.elevator.position+positionShift.elevator;
    }else if(type==TELEMETRIES.AILERON_POSITION){
        f=kalmanStates.aileron.position+positionShift.aileron;
    }else if(type==TELEMETRIES.ALTITUDE_CONTROLLER_OUTPUT){
        f=controllerThrottleOutput;
    }else if(type==TELEMETRIES.ALTITUDE_SPEED){
        f=altitude.speed;
    }else if(type==TELEMETRIES.AILERON_CONTROLLER_OUTPUT){
        f=controllerAileronOutput;
    }else if(type==TELEMETRIES.ELEVATOR_CONTROLLER_OUTPUT){
        f=controllerElevatorOutput;
    }else if(type==TELEMETRIES.ALTITUDE_SETPOINT){
        f=altitudeTrajectory[0];
    }else if(type==TELEMETRIES.ELEVATOR_POS_SETPOINT){
        //f=MPCElevatorTrajectory[0]+positionShift.elevator;
        f=mpcSetpoints.elevator+positionShift.elevator;
    }else if(type==TELEMETRIES.AILERON_POS_SETPOINT){
        //f=MPCAileronTrajectory[0]+positionShift.aileron;
        f=mpcSetpoints.aileron+positionShift.aileron;
    }else if(type==TELEMETRIES.ELEVATOR_ACC){
        f=kalmanStates.elevator.acceleration;
    }else if(type==TELEMETRIES.AILERON_ACC){
        f=kalmanStates.aileron.acceleration;
    }else if(type==TELEMETRIES.VALID_GUMSTIX){
        f=gumstixStable;
    }else if(type==TELEMETRIES.OUTPUT_THROTTLE){
        f=(float)outputThrottle;
    }else if(type==TELEMETRIES.OUTPUT_ELEVATOR){
        f=(float)outputElevator;
    }else if(type==TELEMETRIES.OUTPUT_AILERON){
        f=(float)outputAileron;
    }else if(type==TELEMETRIES.OUTPUT_RUDDER){
        f=(float)outputRudder;
    }else if(type==TELEMETRIES.BLOB_ELEVATOR){
        f=elevatorGumstix;
    }else if(type==TELEMETRIES.BLOB_AILERON){
        f=aileronGumstix;
    }else if(type==TELEMETRIES.BLOB_ALTITUDE){
        f=throttleGumstix;
    }else if(type==TELEMETRIES.PITCH_ANGLE){
        f=(float)pitchAngle/10.0;
    }else if(type==TELEMETRIES.ROLL_ANGLE){
        f=(float)rollAngle/10.0;
    }else if(type==TELEMETRIES.ELEVATOR_ACC_ERROR){
        f=kalmanStates.elevator.acceleration_error;
    }else if(type==TELEMETRIES.ELEVATOR_ACC_INPUT){
        f=kalmanStates.elevator.acceleration_input;
    }else if(type==TELEMETRIES.AILERON_ACC_ERROR){
        f=kalmanStates.aileron.acceleration_error;
    }else if(type==TELEMETRIES.AILERON_ACC_INPUT){
        f=kalmanStates.aileron.acceleration_input;
    }else if(type==TELEMETRIES.ELEVATOR_SHIFT){
        f=positionShift.elevator;
    }else if(type==TELEMETRIES.AILERON_SHIFT){
        f=positionShift.aileron;
    }
    return f;
}*/
/*void telemetryReceive(unsigned char *address64,unsigned char *address16,unsigned char type,float value){
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

        }else if(type==TELEMETRIES.ELEVATOR_ACC){

        }else if(type==TELEMETRIES.AILERON_ACC){

        }else if(type==TELEMETRIES.VALID_GUMSTIX){

        }
}*/

/*void telemetryToCoordinatorSend(){
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
}*/
void telemetryToCoordinatorSet(unsigned char *address64,unsigned char *address16,unsigned char type,unsigned char on, unsigned char frameID){
    *dataOUT='c';
    *(dataOUT+1)=COMMANDS.TELEMETRY_COORDINATOR;
    *(dataOUT+2)=on;
    *(dataOUT+3)=type;
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
    *(dataOUT+2)=telemetryToCoordinatorArr[type];
    *(dataOUT+3)=type;

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
/*void kopterLand(unsigned char *address64,unsigned char *address16,unsigned char on){
    if(on){
        enableLanding();
    }else{
        disableLanding();
    }
}*/
void kopterLandStatusRequest(unsigned char *address64,unsigned char *address16,unsigned char frameID){
    *dataOUT='c';
    *(dataOUT+1)=COMMANDS.LANDING;
    *(dataOUT+2)=GET_STATUS;
    makeTRPacket(address64,address16,0x00,frameID,dataOUT,3);
}
/*void kopterLandReport(unsigned char *address64,unsigned char *address16,unsigned char frameID){
    *dataOUT='r';
    *(dataOUT+1)=COMMANDS.LANDING;
    *(dataOUT+2)=landingState;
    makeTRPacket(address64,address16,0x00,frameID,dataOUT,3);
}*/
/*
void kopterLandReportRecieved(unsigned char *address64,unsigned char *address16,unsigned char status){
    if(status==LANDING.LANDING){

    }else if(status==LANDING.FLIGHT){

    }else if(status==LANDING.STABILIZATION){

    }else if(status==LANDING.ON_GROUND){

    }else if(status==LANDING.TAKE_OFF){

    }
}*/


//CONTROLLERS
void kopterControllersRequest(unsigned char *address64,unsigned char *address16,unsigned char option,unsigned char frameID){
    *(dataOUT)='c';
    *(dataOUT+1)=COMMANDS.CONTROLLERS;
    *(dataOUT+2)=option;
    makeTRPacket(address64,address16,0x00,frameID,dataOUT,3);
}
/*void kopterControllers(unsigned char *address64,unsigned char *address16,unsigned char option){
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
}*/
void kopterControllersStatusRequest(unsigned char *address64,unsigned char *address16,unsigned char frameID){
    *(dataOUT)='c';
    *(dataOUT+1)=COMMANDS.CONTROLLERS;
    *(dataOUT+2)=GET_STATUS;
    makeTRPacket(address64,address16,0x00,frameID,dataOUT,3);
}
/*void kopterControllersReport(unsigned char *address64,unsigned char *address16,unsigned char frameID){
    *(dataOUT)='r';
    *(dataOUT+1)=COMMANDS.CONTROLLERS;
    *(dataOUT+2)=controllerActive;
    makeTRPacket(address64,address16,0x00,frameID,dataOUT,3);
}*/
/*void kopterControllersReportReceived(unsigned char *address64,unsigned char *address16,unsigned char status){
    if(		 status==CONTROLLERS.OFF){

    }else if(status==CONTROLLERS.POSITION){

    }else if(status==CONTROLLERS.VELOCITY){

    }else if(status==CONTROLLERS.MPC){

    }
}*/

//TRAJECTORY POINTS
void kopterTrajectorySetRequest(unsigned char *address64,unsigned char *address16,unsigned char size,int* time,float* elevatorPos,float* aileronPos,float* throttlePos,unsigned char frameID){
    unsigned char *ch;
    int i;

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
/*void kopterTrajectorySet(unsigned char *address64,unsigned char *address16,unsigned char index,uint32_t time,float elevatorPos,float aileronPos,float throttlePos){
    if(index<TRAJECTORY_LENGTH){
        trajMaxIndex=index;
        TRAJ_POINT(index,time,elevatorPos,aileronPos,throttlePos);
    }
}*/
void kopterTrajectorySetStatusRequest(unsigned char *address64,unsigned char *address16,unsigned char frameID){
    *(dataOUT)='c';
    *(dataOUT+1)=COMMANDS.TRAJECTORY_POINTS;
    *(dataOUT+2)=GET_STATUS;
    makeTRPacket(address64,address16,0x00,frameID,dataOUT,4);
}
/*void kopterTrajectorySetReport(unsigned char *address64,unsigned char *address16,unsigned char frameID){
    unsigned char *ch;
    int i;

    *(dataOUT)='r';
    *(dataOUT+1)=COMMANDS.TRAJECTORY_POINTS;
    *(dataOUT+2)=trajMaxIndex+1;

    for(i=0;i<trajMaxIndex+1;i++){
        ch=(unsigned char *) &trajectory[i].time;
        *(dataOUT+3+i*16)=*ch; *(dataOUT+4+i*16)=*(ch+1); *(dataOUT+5+i*16)=*(ch+2); *(dataOUT+6+i*16)=*(ch+3);

        ch=(unsigned char *) &trajectory[i].elevatorPos;
        *(dataOUT+7+i*16)=*ch; *(dataOUT+8+i*16)=*(ch+1); *(dataOUT+9+i*16)=*(ch+2); *(dataOUT+10+i*16)=*(ch+3);

        ch=(unsigned char *) &trajectory[i].aileronPos;
        *(dataOUT+11+i*16)=*ch; *(dataOUT+12+i*16)=*(ch+1); *(dataOUT+13+i*16)=*(ch+2); *(dataOUT+14+i*16)=*(ch+3);

        ch=(unsigned char *) &trajectory[i].throttlePos;
        *(dataOUT+15+i*16)=*ch; *(dataOUT+16+i*16)=*(ch+1); *(dataOUT+17+i*16)=*(ch+2); *(dataOUT+18+i*16)=*(ch+3);
        usartBufferPutByte(usart_buffer_3,(uint8_t)(trajectory[i].elevatorPos), 10);
        usartBufferPutByte(usart_buffer_3,(uint8_t)(trajectory[i].aileronPos), 10);
        usartBufferPutByte(usart_buffer_3,(uint8_t)(trajectory[i].throttlePos), 10);
        usartBufferPutByte(usart_buffer_3,0xFF, 10);
    }
    makeTRPacket(address64,address16,0x00,frameID,dataOUT,(trajMaxIndex+1)*16+3);
}*/
void kopterTrajectorySetReportReceived(unsigned char *address64,unsigned char *address16,unsigned char index,int time,float elevatorPos,float aileronPos,float throttlePos){

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
/*void kopterPositionSlaveSet(unsigned char *address64,unsigned char *address16,unsigned char *slaveAddr){
    char i;
    for(i=0;i<8;i++){
        *(posSlave+i)=*(slaveAddr+i);
    }
}*/
void kopterPositionSlaveSetStatusRequest(unsigned char *address64,unsigned char *address16,unsigned char frameID){
    *(dataOUT)='c';
    *(dataOUT+1)=COMMANDS.POSITION_SLAVE_SET;
    *(dataOUT+2)=GET_STATUS;
    makeTRPacket(address64,address16,0x00,frameID,dataOUT,3);
}
/*void kopterPositionSlaveSetReport(unsigned char *address64,unsigned char *address16,unsigned char frameID){
    char i;
    *(dataOUT)='r';
    *(dataOUT+1)=COMMANDS.POSITION_SLAVE_SET;
    for (i=0;i<8;i++){
        *(dataOUT+2+i)=*(posSlave+i);
    }
    makeTRPacket(address64,address16,0x00,frameID,dataOUT,10);
}*/
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
void kopterTimeRequest(unsigned char *address64,unsigned char *address16,int time,unsigned char frameID){
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
/*void kopterTime(unsigned char *address64,unsigned char *address16,uint32_t time){
    secondsTimer=time;
    milisecondsTimer=0;
}*/
void kopterTimeStatusRequest(unsigned char *address64,unsigned char *address16,unsigned char frameID){
    *(dataOUT)='c';
    *(dataOUT+1)=COMMANDS.TIME;
    *(dataOUT+2)=GET_STATUS;
    makeTRPacket(address64,address16,0x00,frameID,dataOUT,3);
}
/*void kopterTimeReport(unsigned char *address64,unsigned char *address16,unsigned char frameID){
    unsigned char *ch;

    *(dataOUT)='r';
    *(dataOUT+1)=COMMANDS.TIME;

    ch=(unsigned char *) &secondsTimer;
    *(dataOUT+2)=*ch;
    *(dataOUT+3)=*(ch+1);
    *(dataOUT+4)=*(ch+2);
    *(dataOUT+5)=*(ch+3);
    makeTRPacket(address64,address16,0x00,frameID,dataOUT,6);
}*/
void kopterTimeReportReceived(unsigned char *address64,unsigned char *address16,int time){

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
/*void kopterPositionSet(unsigned char *address64,unsigned char *address16,float elevator,float aileron){
    positionShift.elevator=elevator-kalmanStates.elevator.position;
    positionShift.aileron=aileron-kalmanStates.aileron.position;
}*/

