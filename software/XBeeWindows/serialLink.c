#include <stdio.h>
#include <windows.h>
#include "packets.h"


HANDLE hSerial;
char inPacket[60];

void openSerialLine(){
    DCB dcbSerialParams = {0};

    serialHandle = CreateFile("COM1", GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
    if(hSerial==INVALID_HANDLE_VALUE){
        if(GetLastError()==ERROR_FILE_NOT_FOUND){
            printf("SERIAL PORT NOT FOUND /n");
        }
        printf("ERROR in serial port /n");
    }

    printf("SERIAL PORT OPEN");


    dcbSerial.DCBlength=sizeof(dcbSerialParams);
    if (!GetCommState(hSerial, &dcbSerialParams)) {
        printf("ERROR in getting state /n");
    }
    dcbSerialParams.BaudRate=CBR_19200;
    dcbSerialParams.ByteSize=8;
    dcbSerialParams.StopBits=ONESTOPBIT;
    dcbSerialParams.Parity=NOPARITY;
    if(!SetCommState(hSerial, &dcbSerialParams)){
        printf("error setting serial port state /n");
    }

    printf("SERIAL PARAMS SETTED");

    COMMTIMEOUTS timeouts={0};
    timeouts.ReadIntervalTimeout=50;
    timeouts.ReadTotalTimeoutConstant=50;
    timeouts.ReadTotalTimeoutMultiplier=10;
    timeouts.WriteTotalTimeoutConstant=50;
    timeouts.WriteTotalTimeoutMultiplier=10;
    if(!SetCommTimeouts(hSerial, &timeouts)){
        printf("ERROR timeouts setting /n");
    }

    printf("TIMEOUTS SETTED");

}

void closeSerialLine(){
    CloseHandle(hSerial);
    printf("SERIAL PORT CLOSE");
}

void readPacketSerialLine(){
    DWORD bytesReaded;
    do{
        if(!ReadFile(hSerial,inPacket,1,&bytesReaded,NULL)){
            *inPacket=0x00;
        }
    }while(*inPacket!=0x7E)
    ReadFile(hSerial,inPacket+1,2,&bytesReaded,NULL);
    ReadFile(hSerial,inPacket+3,*(inPacket+2)+1,&bytesReaded,NULL);
}


DWORD writeSerialLine(char *out,int length){
    DWORD bytesWrited;
    if(!WriteFile(hSerial,out,length,&bytesWrited,NULL)){
        return -1;
    }
    FlushFileBuffers(hSerial);
    return bytesWrited;
}


void main(){


}
