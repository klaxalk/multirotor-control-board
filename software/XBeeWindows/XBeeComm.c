#include <stdio.h>
#include "serialLink.h"
#include "defines.h"

HANDLE XBeeH;
unsigned char packet[60];

void openXBeeComm(int port){
    XBeeH=openSerialLine(port);
}

void closeXBeeComm(){
    closeSerialLine(XBeeH);
}


unsigned char* readPacket(){
    DWORD bytesReaded;
    int i;
    packet[0]=0x00;
    packet[3]=0x00;
    if(!ReadFile(XBeeH,packet,1,&bytesReaded,NULL)){
        packet[0]=0x00;
    }
    if(packet[0]==0x7E){
        while(!ReadFile(XBeeH,packet+1,2,&bytesReaded,NULL));
        ReadFile(XBeeH,packet+3,packet[2]+1,&bytesReaded,NULL);
    }
    return packet;
}


void sendPacket(unsigned char *packeto){
    DWORD bytesWrited;
    int i;

    WriteFile(XBeeH,packeto,*(packeto+2)+4,&bytesWrited,NULL);
    FlushFileBuffers(XBeeH);
}
