#include <stdio.h>
#include "openLog.h"
#include "packets.h"
#include "controllers.h"
#include "communication.h"
#include "system.h"

void stopLogging(){
	usartBufferPutString(usart_buffer_log,"$$$",10);	//$$$ - characters for stop logging
}

void startLogging(){
	//stopLogging();			
	//usartBufferPutString(usart_buffer_log,"append LOG.TXT\n",10);		
	usartBufferPutString(usart_buffer_log,"Start Logging\n",10);				
}

void loggingData(){
	led_blue_toggle();
	char str[10];
	sprintf(str,"%.3f,",estimatedThrottlePos);    //altitude
	usartBufferPutString(usart_buffer_log,str,10);
	sprintf(str,"%.3f,",estimatedThrottleVel);    
	usartBufferPutString(usart_buffer_log,str,10);
	
	sprintf(str,"%.3f,",estimatedElevatorPos);   //elevator
	usartBufferPutString(usart_buffer_log,str,10);
	sprintf(str,"%.3f,",estimatedElevatorVel);
	usartBufferPutString(usart_buffer_log,str,10);
	sprintf(str,"%.3f,",estimatedElevatorAcc);
	usartBufferPutString(usart_buffer_log,str,10);
	
	sprintf(str,"%.3f,",estimatedAileronPos);    //aileron
	usartBufferPutString(usart_buffer_log,str,10);
	sprintf(str,"%.3f,",estimatedAileronVel);
	usartBufferPutString(usart_buffer_log,str,10);
	sprintf(str,"%.3f,",estimatedAileronAcc);
	usartBufferPutString(usart_buffer_log,str,10);
	
	sprintf(str,"%d,",validGumstix);			 //Blob Detector
	usartBufferPutString(usart_buffer_log,str,10);
	sprintf(str,"%.3f,",estimatedBlobDistance);
	usartBufferPutString(usart_buffer_log,str,10);
	sprintf(str,"%.3f,",estimatedBlobHorizontal);
	usartBufferPutString(usart_buffer_log,str,10);
	sprintf(str,"%.3f,",estimatedBlobVertical);
	usartBufferPutString(usart_buffer_log,str,10);
			
	sprintf(str,"%.3f,",throttleSetpoint);		 //Setpoints
	usartBufferPutString(usart_buffer_log,str,10);
	sprintf(str,"%.3f,",elevatorPositionSetpoint);
	usartBufferPutString(usart_buffer_log,str,10);
	sprintf(str,"%.3f,",aileronPositionSetpoint);
	usartBufferPutString(usart_buffer_log,str,10);
	sprintf(str,"%.3f,",elevatorVelocitySetpoint);
	usartBufferPutString(usart_buffer_log,str,10);
	sprintf(str,"%.3f,",aileronVelocitySetpoint);
	usartBufferPutString(usart_buffer_log,str,10);
	
	sprintf(str,"%d,",outputThrottle);			 //OUTPUTS
	usartBufferPutString(usart_buffer_log,str,10);
	sprintf(str,"%d,",outputElevator);
	usartBufferPutString(usart_buffer_log,str,10);		
	sprintf(str,"%d,",outputAileron);
	usartBufferPutString(usart_buffer_log,str,10);
	sprintf(str,"%d,",outputRudder);
	usartBufferPutString(usart_buffer_log,str,10);
	
	sprintf(str,"%d,",RCchannel[THROTTLE]);    //RC IN
	usartBufferPutString(usart_buffer_log,str,10);
	sprintf(str,"%d,",RCchannel[ELEVATOR]);
	usartBufferPutString(usart_buffer_log,str,10);
	sprintf(str,"%d,",RCchannel[AILERON]);
	usartBufferPutString(usart_buffer_log,str,10);
	sprintf(str,"%d,",RCchannel[RUDDER]);
	usartBufferPutString(usart_buffer_log,str,10);
	
	usartBufferPutString(usart_buffer_log,"-OK-\n\r",10);
}
