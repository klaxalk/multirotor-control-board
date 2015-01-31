#ifndef PACKETS_H
#define PACKETS_H

#include "constants.h"

extern volatile unsigned char telemetryToCoordinatorArr[TELEMETRY_VARIABLES];
extern volatile unsigned char leadingDataReceived;


void packetHandler(unsigned char *inPacket);
void makeTRPacket(unsigned char *adr64,unsigned char *adr16,unsigned char options,unsigned char frameID,unsigned char *data, unsigned char dataLength);

void adr64Setter2(unsigned char * adr,unsigned char * adr2);

#endif /*PACKETS_H*/
