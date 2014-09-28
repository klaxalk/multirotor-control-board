/*
 * communicationTask.c
 *
 * Created: 11.9.2014 11:17:03
 *  Author: Tomas Baca
 */ 

#include "commTask.h"
#include "system.h"
#include "ioport.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usart_driver_RTOS.h"
#include "communication.h"
#include <stdio.h>

extern volatile uint16_t RCchannel[9];
extern UsartBuffer * usart_buffer_xbee;
extern UsartBuffer * usart_buffer_1;
extern UsartBuffer * usart_buffer_4;

extern volatile float groundDistance;
extern volatile float elevatorSpeed;
extern volatile float aileronSpeed;
extern volatile uint8_t px4Confidence;

extern volatile float estimatedElevatorPos;
extern volatile float estimatedAileronPos;

// variables used by the mavlink library
extern mavlink_message_t mavlinkMessage;
extern mavlink_status_t mavlinkStatus;
extern int8_t mavlinkFlag;
extern mavlink_optical_flow_t opticalFlowData;
extern int8_t opticalFlowDataFlag;

void commTask(void *p) {
	
	unsigned char inChar;

	char* ukazatel;
	
	while (1) {

		// xbee received
		if (usartBufferGetByte(usart_buffer_xbee, &inChar, 0)) {
					
			if (inChar == 'x') {
				
				int i;
				for (i = 0; i < 4; i++) {
										
					usartBufferPutInt(usart_buffer_xbee, RCchannel[i], 10, 10);
					usartBufferPutString(usart_buffer_xbee, ", ", 10);
				}
				usartBufferPutString(usart_buffer_xbee, "\r\n", 10);
			}
			
			if (inChar == 'v') {
				
				char buffer[20];
				sprintf(buffer, "%2.2f %2.2f %2.2f\r\n", elevatorSpeed, aileronSpeed, groundDistance);
				usartBufferPutString(usart_buffer_xbee, buffer, 10);
			}

			if (inChar == 'b') {

				ukazatel = (char*) &estimatedElevatorPos;
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel), 10);
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel+1), 10);
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel+2), 10);
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel+3), 10);

				ukazatel = (char*) &estimatedAileronPos;
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel), 10);
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel+1), 10);
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel+2), 10);
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel+3), 10);

				ukazatel = (char*) &estimatedThrottlePos;
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel), 10);
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel+1), 10);
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel+2), 10);
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel+3), 10);

				ukazatel = (char*) &elevatorSetpoint;
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel), 10);
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel+1), 10);
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel+2), 10);
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel+3), 10);

				ukazatel = (char*) &aileronSetpoint;
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel), 10);
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel+1), 10);
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel+2), 10);
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel+3), 10);
				
				usartBufferPutByte(usart_buffer_xbee, controllerEnabled, 10);
				usartBufferPutByte(usart_buffer_xbee, positionControllerEnabled, 10);
				usartBufferPutByte(usart_buffer_xbee, landingRequest, 10);
				usartBufferPutByte(usart_buffer_xbee, trajectoryEnabled, 10);

				usartBufferPutByte(usart_buffer_xbee, validGumstix, 10);
				
				ukazatel = (char*) &elevatorSpeed;
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel), 10);
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel+1), 10);
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel+2), 10);
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel+3), 10);

				ukazatel = (char*) &aileronSpeed;
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel), 10);
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel+1), 10);
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel+2), 10);
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel+3), 10);

				usartBufferPutString(usart_buffer_xbee, "\r\n", 10);
			}
		}
		
		if (usartBufferGetByte(usart_buffer_4, &inChar, 0)) {

			gumstixParseChar(inChar);
		}
		
		if (gumstixDataFlag == 1) {
			
			if (validGumstix == 1) {

				//Gumstix returns position of the blob relative to camera
				//in milimeters, we want position of the drone relative
				//to the blob in meters.
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

				//~ Camera pointing forward and being PORTRAIT oriented
				//~ elevatorGumstix = - (float)xPosGumstixNew / 1000;
				//~ aileronGumstix  = - (float)zPosGumstixNew / 1000;
				//~ throttleGumstix = + (float)yPosGumstixNew / 1000;

				//~ Camera pointing forward and being LANDSCAPE oriented
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

		if (usartBufferGetByte(usart_buffer_1, &inChar, 0)) {

			px4flowParseChar((uint8_t) inChar);
		}

		if (opticalFlowDataFlag == 1) {

			led_blue_toggle();

			// decode the message (there will be new values in opticalFlowData...
			mavlink_msg_optical_flow_decode(&mavlinkMessage, &opticalFlowData);

			//px4flow returns velocities in m/s and gnd. distance in m
			// +elevator = front
			// +aileron  = left
			// +throttle = up

			#if PX4_CAMERA_ORIENTATION == FORWARD

				elevatorSpeed = + opticalFlowData.flow_comp_m_x;
				aileronSpeed  = - opticalFlowData.flow_comp_m_y;

			#elif PX4_CAMERA_ORIENTATION == BACKWARD

				elevatorSpeed = - opticalFlowData.flow_comp_m_x;
				aileronSpeed  = + opticalFlowData.flow_comp_m_y;

			#endif

			if (opticalFlowData.ground_distance < ALTITUDE_MAXIMUM &&
			opticalFlowData.ground_distance > 0.3) {

				groundDistance = opticalFlowData.ground_distance;
			}

			px4Confidence = opticalFlowData.quality;

			opticalFlowDataFlag = 0;
		}
	}
}