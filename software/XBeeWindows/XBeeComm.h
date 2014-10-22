#ifndef XBeeComm_H
#define XBeeComm_H

void openXBeeComm(int port);
void closeXBeeComm();
unsigned char *readPacket();
void sendPacket(unsigned char *packeto);

#endif // XBeeComm_H
