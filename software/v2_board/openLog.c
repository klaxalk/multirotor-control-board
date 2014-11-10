/*
 * openLog.c
 *
 * Created: 30. 10. 2014 16:38:11
 *  Author: Martin
 */ 

#include <stdio.h>
#include "system.h"
#include "usart_driver_RTOS.h"
//#include <stdio.h> // sprintf
//#include <stdlib.h> // abs
#include <string.h>
#include "ioport.h"

extern volatile float groundDistance;
extern volatile float elevatorSpeed;
extern volatile float aileronSpeed;

extern UsartBuffer * usart_buffer_log;
extern UsartBuffer * usart_buffer_4;
char fileName[12];
int i=0;

void stopLogging(){
	usartBufferPutString(usart_buffer_log,"$$$",0);
	delay_us(400000);
}

void startLogging(char newfileName[]){
	stopLogging();
	if(strlen(newfileName)>12) sprintf(fileName, "LOG.TXT");
	else sprintf(fileName, "%s",newfileName);
	sprintf(fileName, "%s",strupr(fileName));
	usartBufferPutString(usart_buffer_4,fileName,0);
	char string[64];
	sprintf(string, "\rappend %s\rLogging started:\n",fileName);
	usartBufferPutString(usart_buffer_log,string,0);
	usartBufferPutString(usart_buffer_4,string,0);
}

void loggingData(){
	i++;
	char string[64];
	sprintf(string, "%d,%f,%f,%f\n", i,groundDistance,elevatorSpeed,aileronSpeed);
	usartBufferPutString(usart_buffer_log,string,0);
	}
	
void setTelemetry(){
	stopLogging();
	
		//prikazy pro read
		//nedodelany, zalezi, co bude potreba
		
	startLogging(fileName);
	}
	


