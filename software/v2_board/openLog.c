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
	// 
	vTaskDelay(200);
}

void startLogging(char * newFileName){
	stopLogging();
	i=0;
	if(strlen(newFileName)>12) strcpy (fileName,"log.txt");
	else strcpy (fileName,newFileName);
	strcpy (fileName,strupr(fileName));
	char str[64];
	sprintf(str, "\rappend %s\rStart:\n",fileName);
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
	


