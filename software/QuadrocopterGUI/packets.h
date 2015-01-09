#ifndef PACKETS_H
#define PACKETS_H

extern unsigned char packet[255];

void packetHandler(unsigned char *inPacket);
//create Transmit Request Packet 0x10
void makeTRPacket(unsigned char *adr64,unsigned char *adr16,unsigned char options,unsigned char frameID,unsigned char *data, unsigned char dataLength);
void parReceivePacket(unsigned char *inPacket,unsigned char *address64,unsigned char *address16,unsigned char *receiveOptions,unsigned char *dataIN);
void parTSPacket(unsigned char *inPacket,unsigned char *frameID,unsigned char *address16,unsigned char *TrRetryCount,unsigned char *deliveryStatus,unsigned char *discoveryStatus);
void parMSPacket(unsigned char *inPacket,unsigned char *status);

#endif /*PACKETS_H*/
