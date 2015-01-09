#ifndef SERIAL_H
#define SERIAL_H
#include <windows.h>

#ifdef __cplusplus
//extern "C" {
#endif
void startSerial(int com);
void checkSerial();
#ifdef __cplusplus
//}
#endif
extern HANDLE matlabH;
#endif
