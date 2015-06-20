#ifndef PACKETS_H
#define PACKETS_H

#include "const.h"
extern unsigned char packet[255];
extern volatile unsigned char telemetryToCoordinatorArr[TELEMETRY_VARIABLES];

void packetHandler(unsigned char *inPacket);
void makeTRPacket(unsigned char *adr64,unsigned char *adr16,unsigned char options,unsigned char frameID,unsigned char *data, unsigned char dataLength);

void adr64Setter(unsigned char * adr,unsigned char b1,unsigned char b2,unsigned char b3,unsigned char b4,unsigned char b5,unsigned char b6,unsigned char b7,unsigned char b8);
void adr64Setter2(unsigned char * adr,unsigned char * adr2);

#endif /*PACKETS_H*/
