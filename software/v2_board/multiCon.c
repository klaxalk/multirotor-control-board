/*
 * multiCon.c
 *
 * Created: 24.8.2015 18:53:33
 *  Author: klaxalk
 */ 

#include "system.h"
#include "ioport.h"
#include "multiCon.h"
#include "communication.h"

uint8_t numberOfDetectedBlobs = 0;
uint8_t multiconErrorState = 0;
blob_t blobs[NUMBER_OF_BLOBS];

/* -------------------------------------------------------------------- */
/*	Variables for data reception from multicon								*/
/* -------------------------------------------------------------------- */

char multiconRxBuffer[MULTICON_BUFFER_SIZE];

int16_t multiconPayloadSize = 0;
int16_t multiconBytesReceived;

uint8_t multiconMessageReceived = 0;
uint8_t multiconReceivingMessage = 0;
uint8_t multiconReceiverState = 0;
uint8_t multiconCrcIn = 0;

uint8_t temp[20];

/* -------------------------------------------------------------------- */
/*	Fetch the incoming char into a buffer, completes the message		*/
/* -------------------------------------------------------------------- */
int8_t multiconParseChar(char inChar, multiconMessageHandler_t * messageHandler) {
	
	uint8_t i;

	if (multiconReceivingMessage) {

		// expecting to receive the payload size
		if (multiconReceiverState == 0) {

			multiconRxBuffer[0] = inChar;
			multiconReceiverState = 1;
			multiconCrcIn += inChar;
			
		} else if (multiconReceiverState == 1) {
			
			multiconRxBuffer[1] = inChar;
			
			multiconPayloadSize = hex2bin((uint8_t *) &multiconRxBuffer);		
			
			multiconCrcIn += inChar;
			
			if (multiconPayloadSize < MULTICON_BUFFER_SIZE) {
				
				multiconReceiverState = 2;
			} else {
				
				multiconReceiverState = 0;
				multiconReceivingMessage = 0;
			}
			
		// expecting to receive the payload
		} else if (multiconReceiverState == 2) {

			// put the char in the buffer
			multiconRxBuffer[multiconBytesReceived++] = inChar;
			
			// add crc
			if (multiconBytesReceived <= multiconPayloadSize-2)
				multiconCrcIn += inChar;
			
			// if the message should end, change state
			if (multiconBytesReceived >= multiconPayloadSize)
				multiconReceiverState = 3;

		} else if (multiconReceiverState == 3) {
		
			/*
			sprintf(temp, "%d %d\n\r", hex2bin((uint8_t *) (multiconRxBuffer + multiconBytesReceived - 2)), multiconCrcIn);
			usartBufferPutString(usart_buffer_4, temp, 10);
			*/
			
			// check the crc
			if (true || multiconCrcIn == hex2bin((uint8_t *) (multiconRxBuffer + multiconBytesReceived - 2))) {
				
				multiconMessageReceived = 1;
				multiconReceivingMessage = 0;
				
				uint8_t i;
				for (i = 0; i < multiconPayloadSize/2; i++) {
					
					multiconRxBuffer[i] = hex2bin((uint8_t *) (multiconRxBuffer + i*2));
				}
				
			} else {

				multiconReceivingMessage = 0;
				multiconReceiverState = 0;
			}
		}

	} else {

		// this character precedes every message
		if (inChar == 'A') {

			multiconCrcIn = 0;

			multiconReceivingMessage = 1;
			multiconReceiverState = 0;
			multiconBytesReceived = 0;
		}
	}
	
	if (multiconMessageReceived) {
		
		messageHandler->messageBuffer = multiconRxBuffer+1;
		messageHandler->messageId = multiconRxBuffer[0];
		multiconMessageReceived = 0;
		return 1;
	}
	
	return 0;
}