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
#include "mpcHandler.h"

uint8_t numberOfDetectedBlobs = 0;
uint8_t multiconErrorState = 0;
blob_t blobs[NUMBER_OF_BLOBS];

/* -------------------------------------------------------------------- */
/*	Variables for data reception from multicon							*/
/* -------------------------------------------------------------------- */

char multiconRxBuffer[MULTICON_BUFFER_SIZE];

int16_t multiconPayloadSize = 0;
int16_t multiconBytesReceived;

uint8_t multiconMessageReceived = 0;
uint8_t multiconReceivingMessage = 0;
uint8_t multiconReceiverState = 0;
uint8_t multiconCrcIn = 0;

/* -------------------------------------------------------------------- */
/*	Variables for bluetooth RSSI										*/
/* -------------------------------------------------------------------- */

volatile float bluetooth_RSSI = 0;

/* -------------------------------------------------------------------- */
/*	Fetch the incoming char into a buffer, completes the message		*/
/* -------------------------------------------------------------------- */
int8_t multiconParseChar(char inChar, multiconMessageHandler_t * messageHandler) {

	if (multiconReceivingMessage) {

		// expecting to receive the payload size
		if (multiconReceiverState == 0) {

			multiconRxBuffer[multiconBytesReceived++] = inChar;
			multiconReceiverState = 1;
			multiconCrcIn += inChar;
			
		} else if (multiconReceiverState == 1) {
			
			multiconRxBuffer[multiconBytesReceived++] = inChar;
			
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
			if (multiconBytesReceived <= multiconPayloadSize)
				multiconCrcIn += inChar;
			
			// if the message should end, change state
			if (multiconBytesReceived >= multiconPayloadSize + 2) // the payload size itself is not included in the payload size
				multiconReceiverState = 3;

		} else if (multiconReceiverState == 3) {
			
			// check the crc
			if (multiconCrcIn == hex2bin((uint8_t *) (multiconRxBuffer + multiconBytesReceived - 2))) {
				
				multiconMessageReceived = 1;
				multiconReceivingMessage = 0;
				
				uint8_t i;
				for (i = 0; i < multiconBytesReceived/2; i++) {
					
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
		
		messageHandler->messageBuffer = multiconRxBuffer+2;
		messageHandler->messageId = multiconRxBuffer[1];
		multiconMessageReceived = 0;
		return 1;
	}
	
	return 0;
}

/* -------------------------------------------------------------------- */
/*	Create a message with all blobs to send								*/
/* -------------------------------------------------------------------- */

void sendBlobs(uint64_t address) {
	
	uint8_t buffer[64];
	
	uint8_t idx = 0;
	
	buffer[idx++] = 'N';
	buffer[idx++] = numberOfDetectedBlobs;
	
	writeFloatToBuffer(buffer, kalmanStates.elevator.position, idx);
	idx += 4;
	writeFloatToBuffer(buffer, kalmanStates.aileron.position, idx);
	idx += 4;
	
	writeFloatToBuffer(buffer, mpcSetpoints.elevator, idx);
	idx += 4;
	writeFloatToBuffer(buffer, mpcSetpoints.aileron, idx);
	idx += 4;
	
	uint8_t i, j;
	for (i = 0; i < numberOfDetectedBlobs; i++) {
		
		writeFloatToBuffer(buffer, blobs[i].x, idx);
		idx += 4;
		
		writeFloatToBuffer(buffer, blobs[i].y, idx);
		idx += 4;
	}
	
	xbeeSendMessageTo(&buffer, idx, address);
}