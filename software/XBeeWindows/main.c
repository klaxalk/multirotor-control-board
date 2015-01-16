#include <stdio.h>
#include "XBeeComm.h"
#include "packets.h"
#include "serialLink.h"
#include "const.h"


void readSerial(int com){
    constInit();
    unsigned char *inPacket=packet;
    openXBeeComm(com);
    while(1){
        inPacket=readPacket();
        if(*inPacket==0x7E){packetHandler(inPacket);}
    }
    closeXBeeComm(com);
}

int main()
{
   readSerial(3);
}
