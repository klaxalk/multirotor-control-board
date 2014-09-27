#ifndef PACKETS_H
#define PACKETS_H

const char COORDINATOR64[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
const char BROADCAST64[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF};
const char UNKNOWN16[2] = {0xFF,0xFE};

extern volatile char outPacket[60];
extern volatile char inPacket[60];

extern volatile char sendPacket;

//create Transmit Request Packet 0x10
char makeTRPacket(char *adr64, char *adr16, char options, char frameID, char *data, unsigned char dataLength);
//parse Modem Status 0x8A
void parMSPacket(char *status);
//parse Transmit Status 0x8B
void parTSPacket(char *frameID, char *address16,char *TrRetryCount, char *deliveryStatus, char *discoveryStatus);
//parse Receive Packet 0x90
void parReceivePacket(char *address64, char *address16, char *receiveOptions, char *data);

#endif /*PACKETS_H*/
