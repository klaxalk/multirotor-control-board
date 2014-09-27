#include <stdio.h>
#include "packets.h"
#include "commands.h"

extern volatile float groundDistance;

char *data[20];

char groundDistanceTelemetry(char *address64, char *address16,char options){
    *data="t";
    *(data+1)=0x01;
    *(data+2)=(char*) &groundDistance;
return makeTRPacket(address64,address16,options,0x01,data,2+sizeof(float));
}

char groundDistanceRequest(char *address64, char *address16){
    *data="c";
    *(data+1)=0x01;
return makeTRPacket(address64,address16,0x00,0x01,data,2+sizeof(float));
}
