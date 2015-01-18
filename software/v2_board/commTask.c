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

void sendFloat(UsartBuffer * usartBuffer, const float var, char * crc) {
	
	char * ukazatel = (char*) &var;
	
	usartBufferPutByte(usartBuffer, *(ukazatel), 10);
	*crc += *(ukazatel);
	
	usartBufferPutByte(usartBuffer, *(ukazatel+1), 10);
	*crc += *(ukazatel+1);
	
	usartBufferPutByte(usartBuffer, *(ukazatel+2), 10);
	*crc += *(ukazatel+2);
	
	usartBufferPutByte(usartBuffer, *(ukazatel+3), 10);
	*crc += *(ukazatel+3);
}

void sendInt16(UsartBuffer * usartBuffer, const int16_t var, char * crc) {
	
	char * ukazatel = (char*) &var;
	
	usartBufferPutByte(usartBuffer, *(ukazatel), 10);
	*crc += *(ukazatel);
	
	usartBufferPutByte(usartBuffer, *(ukazatel+1), 10);
	*crc += *(ukazatel+1);
}

void sendChar(UsartBuffer * usartBuffer, const char var, char * crc) {
	
	usartBufferPutByte(usartBuffer, var, 10);
	*crc += var;
}

void commTask(void *p) {
	
	unsigned char inChar;
	char* ukazatel;
	
	int16_t lastMiliseconds = 0;
	float dt;
	
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
			
			// rotate px4flow data
			elevatorSpeed = -opticalFlowData.flow_comp_m_x;
			aileronSpeed  = +opticalFlowData.flow_comp_m_y;

			// saturate the ground distance
			if (opticalFlowData.ground_distance < ALTITUDE_MAXIMUM && opticalFlowData.ground_distance > 0.3) {

				groundDistance = opticalFlowData.ground_distance;
			}

			px4Confidence = opticalFlowData.quality;
			
			led_yellow_toggle();

			opticalFlowDataFlag = 0;
			
			/* -------------------------------------------------------------------- */
			/*	Send data to ARM													*/
			/* -------------------------------------------------------------------- */
			
			// compute the dt
			portENTER_CRITICAL();
			
			if ((milisecondsTimer - lastMiliseconds) > 0) {
				
				dt = (float) ((float) (milisecondsTimer - lastMiliseconds) * 0.001);
			} else {
				
				dt = (float) ((float) (milisecondsTimer - lastMiliseconds + 1000) * 0.001);
			}
			
			lastMiliseconds = milisecondsTimer;
		
			portEXIT_CRITICAL();
			
			// send data to ARM
			
			// clear the crc
			char crc = 0;
			sendChar(usart_buffer_stm, 'a', &crc);		// this character initiates the transmition
			sendChar(usart_buffer_stm, 1+20, &crc);		// this will be the size of the message
			
			sendChar(usart_buffer_stm, '1', &crc);		// id of the message
			sendFloat(usart_buffer_stm, dt, &crc);
			sendFloat(usart_buffer_stm, elevatorSpeed, &crc);
			sendFloat(usart_buffer_stm, aileronSpeed, &crc);
			sendFloat(usart_buffer_stm, (float) ((float) outputChannels[ELEVATOR]/2 - PPM_IN_MIDDLE_LENGTH)/10, &crc);
			sendFloat(usart_buffer_stm, (float) ((float) outputChannels[AILERON]/2 - PPM_IN_MIDDLE_LENGTH)/10, &crc);
			
			sendChar(usart_buffer_stm, crc, &crc);
		}
	}
}