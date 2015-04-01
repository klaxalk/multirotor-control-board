#include <stdio.h>
#include "logTask.h"
#include "packets.h"
#include "controllers.h"
#include "communication.h"
#include "system.h"
#include "commTask.h"

char str[10];
char *strBin;

void logValueBin(float lValue){
	strBin=(char *) &lValue;
	*(strBin+4)='\0';
	usartBufferPutString(usart_buffer_log,str,10);
}

void logValueStr(float lValue){
	sprintf(str,"%.2f,",lValue);    
	usartBufferPutString(usart_buffer_log,str,10);		
}

void logTaskReadable(void *p){
	
	usartBufferPutString(usart_buffer_log,"Start Logging - String\r\n",10);		
	usartBufferPutString(usart_buffer_log,"Altitude,Elevator,Aileron,Blob,Setpoints,Outputs,RC IN,Time\r\n",10);
	vTaskDelay(100);	
	
	while(1){	
		logValueStr((float) (pitchAngle/10.0));
		logValueStr((float) (rollAngle/10.0));
		logValueStr(position.altitude); //altitude
		logValueStr(speed.altitude);
		logValueStr(position.elevator); //elevator
		logValueStr(speed.elevator);
		logValueStr(acceleration.elevator);
		logValueStr(position.aileron); //aileron
		logValueStr(speed.aileron);
		logValueStr(acceleration.aileron);
		logValueStr(validGumstix); //blob detector
		logValueStr(blob.elevator);
		logValueStr(blob.aileron);
		logValueStr(blob.altitude); 
		logValueStr((float)controllerActive); //controller		
		logValueStr(positionSetpoint.altitude);  //setpoints
		logValueStr(positionSetpoint.elevator);
		logValueStr(positionSetpoint.aileron);
		logValueStr((float)outputThrottle); //outputs
		logValueStr((float)outputElevator);
		logValueStr((float)outputAileron);
		logValueStr((float)outputRudder);
		logValueStr((float)RCchannel[THROTTLE]); //RC IN
		logValueStr((float)RCchannel[ELEVATOR]);
		logValueStr((float)RCchannel[AILERON]);
		logValueStr((float)RCchannel[RUDDER]);
		logValueStr((float)milisecondsTimer); //time	
		sprintf(str,"%d,",secondsTimer);    
		usartBufferPutString(usart_buffer_log,str,10);
				
		usartBufferPutString(usart_buffer_log,"\r\n",10);
		vTaskDelay(40);	
	}
}

void logTaskBinary(void *p){
		vTaskDelay(1000);	
	    usartBufferPutString(usart_buffer_log,"Start Logging - Binary Float\r\n",10);	
		usartBufferPutString(usart_buffer_log,"Angles,Altitude,Elevator,Aileron,Blob,Setpoints,Outputs,RC IN,Time\r\n",10);	
		strBin=str;			
	while(1){			
		logValueBin((float) (pitchAngle/10.0)); //angles
		logValueBin((float) (rollAngle/10.0));
		logValueBin(position.altitude); //altitude
		logValueBin(speed.altitude);
		logValueBin(position.elevator); //elevator
		logValueBin(speed.elevator);
		logValueBin(acceleration.elevator);
		logValueBin(position.aileron); //aileron
		logValueBin(speed.aileron);
		logValueBin(acceleration.aileron);
		logValueBin(validGumstix); //blob detector
		logValueBin(blob.elevator);
		logValueBin(blob.aileron);
		logValueBin(blob.altitude); 
		logValueBin((float)controllerActive); //controller		
		logValueBin(positionSetpoint.altitude); //setpoints
		logValueBin(positionSetpoint.elevator);
		logValueBin(positionSetpoint.aileron);
		logValueBin((float)outputThrottle); //outputs
		logValueBin((float)outputElevator);
		logValueBin((float)outputAileron);
		logValueBin((float)outputRudder);
		logValueBin((float)RCchannel[THROTTLE]); //RC IN
		logValueBin((float)RCchannel[ELEVATOR]);
		logValueBin((float)RCchannel[AILERON]);		
		logValueBin((float)RCchannel[RUDDER]);	
		logValueBin((float)milisecondsTimer); //time	
		logValueBin((float)secondsTimer); 					
		usartBufferPutString(usart_buffer_log,"\r\n",10);				
	vTaskDelay(14);		
	}
}