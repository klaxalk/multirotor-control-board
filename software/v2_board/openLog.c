/*
 * openLog.c
 *
 * Created: 30. 10. 2014 16:38:11
 *  Author: Martin
 */ 

#include <stdio.h>
#include "system.h"
#include "usart_driver_RTOS.h"
#include <string.h>
#include "ioport.h"
#include "packets.h"
#include "openLog.h"


extern volatile float groundDistance;			
extern volatile float estimatedAileronVel;			
extern volatile float estimatedElevatorVel;
extern volatile float estimatedAileronPos;
extern volatile float estimatedElevatorPos;
extern volatile float aileronSpeed;
extern volatile float elevatorSpeed;
extern volatile float opticalFlowData;
extern volatile uint16_t outputChannels[6];


extern UsartBuffer * usart_buffer_log;
extern UsartBuffer * usart_buffer_4;
char fileName[12]; 
int i=0;

void openLogRequest(unsigned char *address64,unsigned char *address16,unsigned char *data,unsigned char frameID){
		unsigned char dataLength = *(data);
		*(data)='o';
		makeTRPacket(address64,address16,0x00,frameID,data,dataLength+1);
}

void openLogReceive(unsigned char *address64,unsigned char *address16,unsigned char *data){
	
	usartBufferPutByte(usart_buffer_4,*data,0);
	
		//1st byte - data length
		//2nd byte - 'o' (ignore)
		char fName[32];
		int i;
		sprintf(fName,"%c",*(data+2));
		for (i=0;i<=(*(data)-3);i++)
		{sprintf(fName,"%s%c",fName,*(data+3+i));
		usartBufferPutByte(usart_buffer_4,*(data+3+i),0);
		}
		
		startLogging(fName);
}

void stopLogging(){
	usartBufferPutString(usart_buffer_log,"$$$",0);
	// 
	vTaskDelay(200);
}

void startLogging(char * newFileName){
	stopLogging();
	i=0;
	//
	if(strlen(newFileName)>12) strcpy (fileName,"log.txt");
	else strcpy (fileName,newFileName);
	strcpy (fileName,strupr(fileName));
	char str[64];
	sprintf(str, "\rappend %s\rStart logging:\n",fileName);
	usartBufferPutString(usart_buffer_log,str,0);
}

void loggingData(){
	i++;
	char str[128];
	sprintf(str, "%d,%f,%f,%f,%d,%d,%d,%d\n", i,groundDistance,elevatorSpeed,aileronSpeed,outputChannels[0],outputChannels[1],outputChannels[2],outputChannels[3]);
	usartBufferPutString(usart_buffer_log,str,0);
	}
	
void setTelemetry(){
	stopLogging();
	
		//prikazy pro read
		//nedodelany, zalezi, co bude potreba
		
	startLogging(fileName);
	}
	


