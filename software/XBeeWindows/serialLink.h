#ifndef serialLink_H
#define serialLink_H
#include <windows.h>

HANDLE openSerialLine(int port);
void closeSerialLine(HANDLE hSerial);
DWORD readBytes(HANDLE hSerial,unsigned char *bytes,int length);
DWORD sendBytes(HANDLE hSerial,unsigned char *bytes,int length);

#endif // serialLink_H
