/*
 * openLog.c
 *
 * Created: 30. 10. 2014 16:38:11
 *  Author: Martin
 */ 

#include <stdio.h>
#include "system.h"
#include "usart_driver_RTOS.h"
#include <stdio.h> 
#include <stdlib.h> 
#include <string.h>
#include "ioport.h"

extern volatile float groundDistance;			
extern volatile float elevatorSpeed;			
extern volatile float aileronSpeed;
extern volatile float estimatedAileronPos;
extern volatile float estimatedElevatorPos;


extern UsartBuffer * usart_buffer_log;
char fileName[12];
int i=0;

void stopLogging(){
	usartBufferPutString(usart_buffer_log,"$$$",0);
	// waiting for 
	vTaskDelay(200);
}

void startLogging(char * newfileName){
	stopLogging();
	
	// exception length of fileName
	if(strlen(newfileName)>12) sprintf(fileName, "LOG.TXT");
	else sprintf(fileName, "%s",newfileName);
	
	// 
	sprintf(fileName, "%s",strupr(fileName));
	char str[64];
	sprintf(str, "\rappend %s\rLogging started:\n",fileName);
	usartBufferPutString(usart_buffer_log,str,0);
}

void loggingData(){
	i++;
	char str[128];
	sprintf(str, "%d,%f,%f,%f,%f,%f\n", i,groundDistance,elevatorSpeed,aileronSpeed,estimatedAileronPos,estimatedElevatorPos);
	usartBufferPutString(usart_buffer_log,str,0);
	}
	
void setTelemetry(){
	stopLogging();
	
		//prikazy pro read
		//nedodelany, zalezi, co bude potreba
		
	startLogging(fileName);
	}
	


