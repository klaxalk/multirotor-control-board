#include <stdio.h>
#include "openLog.h"
#include "packets.h"
#include "controllers.h"
#include "communication.h"
#include "system.h"

void stopLogging(){
	usartBufferPutString(usart_buffer_log,"$$$",0);	//$$$ - characters for stop logging
}

void startLogging(){
	stopLogging();			
	usartBufferPutString(usart_buffer_log,"append LOG.TXT\n",0);		
	usartBufferPutString(usart_buffer_log,"Start Logging\n",0);				
}

void loggingData(){
	char str[10];
	sprintf(str,"%.3f,",estimatedThrottlePos);    //altitude
	usartBufferPutString(usart_buffer_log,str,0);
	sprintf(str,"%.3f,",estimatedThrottleVel);    
	usartBufferPutString(usart_buffer_log,str,0);
	
	sprintf(str,"%.3f,",estimatedElevatorPos);   //elevator
	usartBufferPutString(usart_buffer_log,str,0);
	sprintf(str,"%.3f,",estimatedElevatorVel);
	usartBufferPutString(usart_buffer_log,str,0);
	sprintf(str,"%.3f,",estimatedElevatorAcc);
	usartBufferPutString(usart_buffer_log,str,0);
	
	sprintf(str,"%.3f,",estimatedAileronPos);    //aileron
	usartBufferPutString(usart_buffer_log,str,0);
	sprintf(str,"%.3f,",estimatedAileronVel);
	usartBufferPutString(usart_buffer_log,str,0);
	sprintf(str,"%.3f,",estimatedAileronAcc);
	usartBufferPutString(usart_buffer_log,str,0);
	
	sprintf(str,"%d,",validGumstix);			 //Blob Detector
	usartBufferPutString(usart_buffer_log,str,0);
	sprintf(str,"%.3f,",estimatedBlobDistance);
	usartBufferPutString(usart_buffer_log,str,0);
	sprintf(str,"%.3f,",estimatedBlobHorizontal);
	usartBufferPutString(usart_buffer_log,str,0);
	sprintf(str,"%.3f,",estimatedBlobVertical);
	usartBufferPutString(usart_buffer_log,str,0);
			
	sprintf(str,"%.3f,",throttleSetpoint);		 //Setpoints
	usartBufferPutString(usart_buffer_log,str,0);
	sprintf(str,"%.3f,",elevatorPositionSetpoint);
	usartBufferPutString(usart_buffer_log,str,0);
	sprintf(str,"%.3f,",aileronPositionSetpoint);
	usartBufferPutString(usart_buffer_log,str,0);
	sprintf(str,"%.3f,",elevatorVelocitySetpoint);
	usartBufferPutString(usart_buffer_log,str,0);
	sprintf(str,"%.3f,",aileronVelocitySetpoint);
	usartBufferPutString(usart_buffer_log,str,0);
	
	sprintf(str,"%d,",outputThrottle);			 //OUTPUTS
	usartBufferPutString(usart_buffer_log,str,0);
	sprintf(str,"%d,",outputElevator);
	usartBufferPutString(usart_buffer_log,str,0);		
	sprintf(str,"%d,",outputAileron);
	usartBufferPutString(usart_buffer_log,str,0);
	sprintf(str,"%d,",outputRudder);
	usartBufferPutString(usart_buffer_log,str,0);
	
	sprintf(str,"%.3f,",RCchannel[THROTTLE]);    //RC IN
	usartBufferPutString(usart_buffer_log,str,0);
	sprintf(str,"%.3f,",RCchannel[ELEVATOR]);
	usartBufferPutString(usart_buffer_log,str,0);
	sprintf(str,"%.3f,",RCchannel[AILERON]);
	usartBufferPutString(usart_buffer_log,str,0);
	sprintf(str,"%.3f,",RCchannel[RUDDER]);
	usartBufferPutString(usart_buffer_log,str,0);
	
	usartBufferPutString(usart_buffer_log,"\n",0);
}
