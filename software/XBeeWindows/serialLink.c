#include <stdio.h>
#include <windows.h>


HANDLE openSerialLine(int port){
    HANDLE hSerial;
    DCB dcbSerialParams = {0};

    char str[15];
    sprintf(str, "COM%d", port);
    hSerial = CreateFile(str, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
    if(hSerial==INVALID_HANDLE_VALUE){
        printf("ERROR in serial port \n");
        if(GetLastError()==ERROR_FILE_NOT_FOUND){
            printf("SERIAL PORT COM%i NOT FOUND \n",port);
        }
    }else{

    printf("SERIAL PORT COM%i OPEN\n",port);

        dcbSerialParams.DCBlength=sizeof(DCB);
        if (!GetCommState(hSerial, &dcbSerialParams)) {
            printf("ERROR in getting state \n");
        }
        dcbSerialParams.BaudRate=CBR_19200;
        dcbSerialParams.ByteSize=8;
        dcbSerialParams.StopBits=ONESTOPBIT;
        dcbSerialParams.Parity=NOPARITY;
        if(!SetCommState(hSerial, &dcbSerialParams)){
            printf("error setting serial port state \n");
        }

        COMMTIMEOUTS timeouts={0};
        timeouts.ReadIntervalTimeout=50;
        timeouts.ReadTotalTimeoutConstant=50;
        timeouts.ReadTotalTimeoutMultiplier=10;
        timeouts.WriteTotalTimeoutConstant=50;
        timeouts.WriteTotalTimeoutMultiplier=10;
        if(!SetCommTimeouts(hSerial, &timeouts)){
            printf("ERROR timeouts setting \n");
        }
    }
    return hSerial;
}

void closeSerialLine(HANDLE hSerial){
    CloseHandle(hSerial);
    printf("SERIAL PORT COM CLOSE");
}

DWORD readBytes(HANDLE hSerial,unsigned char *bytes,int length){
    DWORD bytesReaded;

    if(!ReadFile(hSerial,bytes,length,&bytesReaded,NULL)){
            return 0;
    }
    return bytesReaded;
}


DWORD sendBytes(HANDLE hSerial,unsigned char *bytes,int length){
    DWORD bytesWrited;

    if(!WriteFile(hSerial,bytes,length,&bytesWrited,NULL)){
        return 0;
    }
    FlushFileBuffers(hSerial);
    return bytesWrited;
}


