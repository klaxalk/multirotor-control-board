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
	
	main2commMessage_t main2commMessage;
	
	/* -------------------------------------------------------------------- */
	/*	Needed for receiving from STM										*/
	/* -------------------------------------------------------------------- */
	char payloadSize = 0;
	char stmRxBuffer[STM_BUFFER_SIZE];
	int bytesReceived;
	char stmMessageReceived = 0;
	char receivingMessage = 0;
	int receiverState = 0;
	char crcIn = 0;
	
	/* -------------------------------------------------------------------- */
	/*	For computing dt for STM											*/
	/* -------------------------------------------------------------------- */
	int16_t lastMiliseconds = 0;
	float dt;
	
	/* -------------------------------------------------------------------- */
	/*	initialize kalmanStates 											*/
	/* -------------------------------------------------------------------- */
	kalmanStates.elevator.position = 0;
	kalmanStates.elevator.velocity = 0;
	kalmanStates.elevator.acceleration = 0;
	
	kalmanStates.aileron.position = 0;
	kalmanStates.aileron.velocity = 0;
	kalmanStates.aileron.acceleration = 0;
	
	while (1) {
		
		/* -------------------------------------------------------------------- */
		/*	A character received from STM										*/
		/* -------------------------------------------------------------------- */
		if (usartBufferGetByte(usart_buffer_stm, &inChar, 0)) {

			if (receivingMessage) {

				// expecting to receive the payload size
				if (receiverState == 0) {

					// check the message length
					if (inChar >= 0 && inChar < STM_BUFFER_SIZE) {

						payloadSize = inChar;
						receiverState = 1;
						crcIn += inChar;
					
					// the receiving message is over the buffer size	
					} else {

						receivingMessage = 0;
						receiverState = 0;
					}

				// expecting to receive the payload
				} else if (receiverState == 1) {

					// put the char in the buffer
					stmRxBuffer[bytesReceived++] = inChar;
					// add crc
					crcIn += inChar;

					// if the message should end, change state
					if (bytesReceived >= payloadSize)
						receiverState = 2;

				// expecting to receive the crc
				} else if (receiverState == 2) {

					if (crcIn == inChar) {

						stmMessageReceived = 1;
						receivingMessage = 0;
						} else {

						receivingMessage = 0;
						receiverState = 0;
					}
				}

			} else {

				// this character precedes every message
				if (inChar == 'a') {

					crcIn = inChar;

					receivingMessage = 1;
					receiverState = 0;
					bytesReceived = 0;
				}
			}
		}
		
		/* -------------------------------------------------------------------- */
		/*	A message received from STM											*/
		/* -------------------------------------------------------------------- */
		if (stmMessageReceived) {
			
			int idx = 0;

			//  read the message ID
			char messageId = readChar(stmRxBuffer, &idx);
			
			/* -------------------------------------------------------------------- */
			/*	If the message contains output of MPC								*/
			/* -------------------------------------------------------------------- */
			if (messageId == '1') {
				
				int16_t tempInt;
				
				// saturate and save incoming data
				
				portENTER_CRITICAL();
				
				tempInt = readInt16(stmRxBuffer, &idx);
				if (tempInt > MPC_SATURATION) {
					mpcElevatorOutput = MPC_SATURATION;
				} else if (tempInt < -MPC_SATURATION) {
					mpcElevatorOutput = -MPC_SATURATION;
				} else {
					mpcElevatorOutput = tempInt;
				}
				
				tempInt = readInt16(stmRxBuffer, &idx);
				if (tempInt > MPC_SATURATION) {
					mpcAileronOutput = MPC_SATURATION;
				} else if (tempInt < -MPC_SATURATION) {
					mpcAileronOutput = -MPC_SATURATION;
				} else {
					mpcAileronOutput = tempInt;
				}
				
				// convert degrees to the time units of PPM
				controllerElevatorOutput = mpcElevatorOutput;
				controllerAileronOutput = mpcAileronOutput;
				
				portEXIT_CRITICAL();
				
			/* -------------------------------------------------------------------- */
			/*	If the message contains output of kalman filter						*/
			/* -------------------------------------------------------------------- */
			} else if (messageId == '2') {
				
				kalmanStates.elevator.position = readFloat(stmRxBuffer, &idx);
				kalmanStates.elevator.velocity = readFloat(stmRxBuffer, &idx);
				kalmanStates.elevator.acceleration = readFloat(stmRxBuffer, &idx);
				
				kalmanStates.aileron.position = readFloat(stmRxBuffer, &idx);
				kalmanStates.aileron.velocity = readFloat(stmRxBuffer, &idx);
				kalmanStates.aileron.acceleration = readFloat(stmRxBuffer, &idx);
			}
			
			stmMessageReceived = 0;
		}
		
		/* -------------------------------------------------------------------- */
		/*	A character received from XBee										*/
		/* -------------------------------------------------------------------- */
		if (usartBufferGetByte(usart_buffer_xbee, &inChar, 0)) {

			// send a simple telemetry
			if (inChar == 'b') {

				char crc = 0;

				sendFloat(usart_buffer_xbee, kalmanStates.elevator.position, &crc);
				sendFloat(usart_buffer_xbee, kalmanStates.aileron.position, &crc);
			
				sendInt16(usart_buffer_xbee, controllerElevatorOutput, &crc);
				sendInt16(usart_buffer_xbee, controllerAileronOutput, &crc);

				usartBufferPutString(usart_buffer_xbee, "\r\n", 10);
			}
		}
	
		/* -------------------------------------------------------------------- */
		/*	A character received from px4flow									*/
		/* -------------------------------------------------------------------- */
		if (usartBufferGetByte(usart_buffer_1, &inChar, 0)) {

			px4flowParseChar((uint8_t) inChar);
		}
		
		/* -------------------------------------------------------------------- */
		/*	A message received from px4flow										*/
		/* -------------------------------------------------------------------- */
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
		
		/* -------------------------------------------------------------------- */
		/*	A message received from the main Task								*/
		/* -------------------------------------------------------------------- */
		if (xQueueReceive(main2commsQueue, &main2commMessage, 0)) {
						
			// send message to STM to reset its Kalman filter			
			if (main2commMessage.messageType == CLEAR_STATES) {
	
				char crc = 0;
				sendChar(usart_buffer_stm, 'a', &crc);		// this character initiates the transmition
				sendChar(usart_buffer_stm, 1, &crc);		// this will be the size of the message
			
				sendChar(usart_buffer_stm, '2', &crc);		// id of the message
			
				sendChar(usart_buffer_stm, crc, &crc);		// isend crc
			}
		}
	}
}