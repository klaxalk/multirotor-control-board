/*
 * communicationTask.c
 *
 * Created: 11.9.2014 11:17:03
 *  Author: Tomas Baca
 */ 

#include "system.h"
#include "commTask.h"
#include "communication.h"
#include <stdio.h>
#include "controllers.h"

void commTask(void *p) {
	
	unsigned char inChar;
	char* ukazatel;
	
	while (1) {
		
		if (usartBufferGetByte(usart_buffer_4, &inChar, 0)) {

			usartBufferPutByte(usart_buffer_stm, inChar, 0);
		}
		
		if (usartBufferGetByte(usart_buffer_stm, &inChar, 0)) {

			usartBufferPutByte(usart_buffer_4, inChar, 0);
		}
		
		// xBee received
		if (usartBufferGetByte(usart_buffer_xbee, &inChar, 0)) {

			// prototype of answering with log to xBee
			if (inChar == 'b') {

				ukazatel = (char*) &groundDistance;
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel), 10);
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel+1), 10);
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel+2), 10);
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel+3), 10);

				usartBufferPutString(usart_buffer_xbee, "\r\n", 10);
			}
		}
	
		if (usartBufferGetByte(usart_buffer_1, &inChar, 0)) {

			px4flowParseChar((uint8_t) inChar);
		}

		if (opticalFlowDataFlag == 1) {

			// decode the message (there will be new values in opticalFlowData...
			mavlink_msg_optical_flow_decode(&mavlinkMessage, &opticalFlowData);

			// px4flow returns velocities in m/s and gnd. distance in m
			// +elevator = front
			// +aileron  = left
			// +throttle = up

			#if PX4_CAMERA_ORIENTATION == FORWARD

				elevatorSpeed = +opticalFlowData.flow_comp_m_x;
				aileronSpeed  = -opticalFlowData.flow_comp_m_y;

			#elif PX4_CAMERA_ORIENTATION == BACKWARD

				elevatorSpeed = -opticalFlowData.flow_comp_m_x;
				aileronSpeed  = +opticalFlowData.flow_comp_m_y;

			#endif

			if (opticalFlowData.ground_distance < ALTITUDE_MAXIMUM && opticalFlowData.ground_distance > 0.3) {

				groundDistance = opticalFlowData.ground_distance;
			}

			px4Confidence = opticalFlowData.quality;
			
			led_yellow_toggle();

			opticalFlowDataFlag = 0;
		}
	}
}