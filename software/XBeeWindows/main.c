#include <stdio.h>
#include "XBeeComm.h"
#include "packets.h"
#include "commands.h"
#include "serialLink.h"

int main()
{
    unsigned char *inPacket=packet;
    constInit();
    openXBeeComm(3);

    while(1){
        inPacket=readPacket();
        if(*inPacket==0x7E){packetHandler(inPacket);}
    }

    closeXBeeComm();
    return 0;
}
