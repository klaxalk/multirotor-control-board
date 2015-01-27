#include <stdio.h>
#include "commTask.h"
#include "usart_driver_RTOS.h"
#include "communication.h"
#include "controllers.h"
#include "system.h"
#include "packets.h"
#include "commands.h"



void commTask(void *p) {	
	//cca 40kHz	
	unsigned char inChar;
	unsigned char packet[60];	
	int8_t i;
	int16_t counter40Hz=500;
	int16_t counter20Hz=0;
	
	//wait for XBee	
	vTaskDelay(1000);
	

		
	while (1) {				
		stateChecker();
		
		if (counter40Hz++>1000){
			counter40Hz=0;
		}
		
		if (counter20Hz++>2000){
			counter20Hz=0;
			telemetryToCoordinatorSend();
			if(positionControllerEnabled && leadKopter[7]!=0x00){
				kopterLeadDataSend(leadKopter,ADDRESS.UNKNOWN16,estimatedThrottlePos,elevatorDesiredSpeedPosController,aileronDesiredSpeedPosController,elevatorPosContError,aileronPosContError,0x00);			
				led_blue_toggle();
			}
		}		
		
		
		// XBee
		if (usartBufferGetByte(usart_buffer_xbee, &inChar, 0)) {					
			 //packet received
			 if (inChar == 0x7E){
				 *packet=inChar;
				 while(!usartBufferGetByte(usart_buffer_xbee, packet+1, 0));
				 while(!usartBufferGetByte(usart_buffer_xbee, packet+2, 0));
				 for (i=0;i<*(packet+2)+1;i++){
					while(!usartBufferGetByte(usart_buffer_xbee, packet+3+i, 0));
				 }
				 packetHandler(packet);
			 }
		}
		

		//Gumstix
		if (usartBufferGetByte(usart_buffer_4, &inChar, 0)) {
			gumstixParseChar(inChar);
		}
		
		if (gumstixDataFlag == 1) {
			if (validGumstix == 1) {
				//Gumstix returns position of the blob relative to camera
				//in mm, we want position of the drone relative to the blob in m.
				// +elevator = front
				// +aileron  = left
				// +throttle = up

				//saturation
				if(xPosGumstixNew > 2*POSITION_MAXIMUM) xPosGumstixNew = 2*POSITION_MAXIMUM;
				if(xPosGumstixNew < 0) xPosGumstixNew = 0; //distance from the blob (positive)

				if(yPosGumstixNew > +POSITION_MAXIMUM) yPosGumstixNew = +POSITION_MAXIMUM;
				if(yPosGumstixNew < -POSITION_MAXIMUM) yPosGumstixNew = -POSITION_MAXIMUM;

				if(zPosGumstixNew > +POSITION_MAXIMUM) zPosGumstixNew = +POSITION_MAXIMUM;
				if(zPosGumstixNew < -POSITION_MAXIMUM) zPosGumstixNew = -POSITION_MAXIMUM;

				#if GUMSTIX_CAMERA_POINTING == FORWARD //camera led on up side
					//Camera pointing forward and being LANDSCAPE oriented
					elevatorGumstix = - (float) xPosGumstixNew / 1000;
					aileronGumstix  = - (float) yPosGumstixNew / 1000;
					throttleGumstix = - (float) zPosGumstixNew / 1000;
				#elif GUMSTIX_CAMERA_POINTING == DOWNWARD //camera led on front side
					elevatorGumstix = + (float) yPosGumstixNew / 1000;
					aileronGumstix  = - (float) zPosGumstixNew / 1000;
					throttleGumstix = + (float) xPosGumstixNew / 1000;
				#endif
			}
			gumstixDataFlag = 0;
		}

		//PX4Flow
		if (usartBufferGetByte(usart_buffer_1, &inChar, 0)) {
			px4flowParseChar((uint8_t) inChar);
		}

		if (opticalFlowDataFlag == 1) {
			// decode the message (there will be new values in opticalFlowData...)
			mavlink_msg_optical_flow_decode(&mavlinkMessage, &opticalFlowData);
			
			//px4flow returns velocities in m/s and ground distance in m
			// +elevator = front
			// +aileron  = left
			// +throttle = up
				elevatorSpeed = - opticalFlowData.flow_comp_m_x;
				aileronSpeed  = + opticalFlowData.flow_comp_m_y;

			
			if (opticalFlowData.ground_distance < ALTITUDE_MAXIMUM 
			&& opticalFlowData.ground_distance >= 0.2 ) {
				groundDistance = opticalFlowData.ground_distance;
			}
			px4Confidence = opticalFlowData.quality;
			opticalFlowDataFlag = 0;
		}
	}	
}