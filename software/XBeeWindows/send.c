#include <stdio.h>
#include "commands.h"
#include "const.h"



void getAddress(unsigned char addr[8],unsigned char kopter){
    if(kopter==KOPTERS.K1){
        addr=ADDRESS.K1;
    }else if(kopter==KOPTERS.K2){
        addr=ADDRESS.K2;
    }else if(kopter==KOPTERS.K3){
        addr=ADDRESS.K3;
    }else if(kopter==KOPTERS.KC1){
        addr=ADDRESS.KC1;
    }else {
        addr=NULL;
    }
}

void telemetryGet(unsigned char kopter,unsigned char type,unsigned char on){
    unsigned char addr[8]={0};
    getAddress(addr,kopter);
    if(addr!=NULL){
        telemetryToCoordinatorSet(addr,ADDRESS.UNKNOWN16,type,on,0x00);
    }
}

void telemetryStatus(unsigned char kopter,unsigned char type){
    unsigned char addr[8]={0};
    getAddress(addr,kopter);
    if(addr!=NULL){
        telemetryToCoordinatorStatusRequest(addr,ADDRESS.UNKNOWN16,type,0x00);
    }
}

void land(unsigned char kopter,unsigned char on){
    unsigned char addr[8]={0};
    getAddress(addr,kopter);
    if(addr!=NULL){
        kopterLandRequest(addr,ADDRESS.UNKNOWN16,on,0x00);
    }
}

void landStatus(unsigned char kopter){
    unsigned char addr[8]={0};
    getAddress(addr,kopter);
    if(addr!=NULL){
        kopterLandStatusRequest(addr,ADDRESS.UNKNOWN16,0x00);
    }
}

void trajectoryFollow(unsigned char kopter,unsigned char on){
    unsigned char addr[8]={0};
    getAddress(addr,kopter);
    if(addr!=NULL){
        kopterTrajectoryFollowRequest(addr,ADDRESS.UNKNOWN16,on,0x00);
    }
}

void trajectoryFollowStatus(unsigned char kopter){
    unsigned char addr[8]={0};
    getAddress(addr,kopter);
    if(addr!=NULL){
        kopterTrajectoryFollowStatusRequest(addr,ADDRESS.UNKNOWN16,0x00);
    }
}

void trajectoryAddPoint(unsigned char kopter,unsigned char index,float time,float elevatorPos,float aileronPos,float throttlePos){
    unsigned char addr[8]={0};
    getAddress(addr,kopter);
    if(addr!=NULL){
        kopterTrajectoryAddPointRequest(addr,ADDRESS.UNKNOWN16,index,time,elevatorPos,aileronPos,throttlePos,0x00);
    }
}

void trajectoryPointStatus(unsigned char kopter){
    unsigned char addr[8]={0};
    getAddress(addr,kopter);
    if(addr!=NULL){
        kopterTrajectoryPointStatusRequest(addr,ADDRESS.UNKNOWN16,0x00);
    }
}

void setSetpoint(unsigned char kopter,unsigned char type,unsigned char incType,float value){
    unsigned char addr[8]={0};
    getAddress(addr,kopter);
    if(addr!=NULL){
        kopterSetpointsSetRequest(addr,ADDRESS.UNKNOWN16,type,incType,value,0x00);
    }
}

void setpointValue(unsigned char kopter,unsigned char type){
    unsigned char addr[8]={0};
    getAddress(addr,kopter);
    if(addr!=NULL){
        kopterSetpointStatusRequest(addr,ADDRESS.UNKNOWN16,type,0x00);
    }
}

void setController(unsigned char kopter,unsigned char option){
    unsigned char addr[8]={0};
    getAddress(addr,kopter);
    if(addr!=NULL){
        kopterControllersRequest(addr,ADDRESS.UNKNOWN16,option,0x00);
    }
}

void controllerStatus(unsigned char kopter){
    unsigned char addr[8]={0};
    getAddress(addr,kopter);
    if(addr!=NULL){
        kopterControllersStatusRequest(addr,ADDRESS.UNKNOWN16,0x00);
    }
}

void setGumstix(unsigned char kopter,unsigned char on){
    unsigned char addr[8]={0};
    getAddress(addr,kopter);
    if(addr!=NULL){
        kopterGumstixRequest(addr,ADDRESS.UNKNOWN16,on,0x00);
    }
}

void gumstixStatus(unsigned char kopter){
    unsigned char addr[8]={0};
    getAddress(addr,kopter);
    if(addr!=NULL){
        kopterGumstixStatusRequest(addr,ADDRESS.UNKNOWN16,0x00);
    }
}

void XBeeMessageSend(unsigned char kopter,char * message){
    unsigned char addr[8]={0};
    getAddress(addr,kopter);
    if(addr!=NULL){
        sendXBeeMessage(addr,ADDRESS.UNKNOWN16,message,0x00);
    }
}
