#include <stdio.h>
#include "XBeeComm.h"
#include "packets.h"
#include "serialLink.h"
#include "const.h"
#include "serial.h"

unsigned char *inPacket=packet;

void startSerial(int com)
{
        constInit();
        openXBeeComm(com);

}
void checkSerial()
{
        inPacket=readPacket();
      if(*inPacket==0x7E){packetHandler(inPacket);}
}
