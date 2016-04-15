/*
 * raspberryPi.c
 *
 * Created: 8.10.2015 16:31:27
 *  Author: klaxalk
 */ 

#include "system.h"
#include "ioport.h"
#include "raspberryPi.h"
#include "communication.h"
#include "mpcHandler.h"
#include "xbee.h"

/* -------------------------------------------------------------------- */
/*	Variables for data reception from RPi								*/
/* -------------------------------------------------------------------- */

char rpiRxBuffer[RPI_BUFFER_SIZE];

int16_t rpiPayloadSize = 0;
int16_t rpiBytesReceived;

char rpiMessageReceived = 0;
char rpiReceivingMessage = 0;
char rpiReceiverState = 0;
char rpiCrcIn = 0;

volatile float rpix = 0;
volatile float rpiy = 0;
volatile float rpiz = 0;
volatile char rpiOk = 0;

/* -------------------------------------------------------------------- */
/*	Fetch the incoming char into a buffer, completes the message		*/
/* -------------------------------------------------------------------- */
int8_t rpiParseChar(char inChar, rpiMessageHandler_t * messageHandler) {

	if (rpiReceivingMessage) {

		// expecting to receive the payload size
		if (rpiReceiverState == 0) {

			// check the message length
			if (inChar >= 0 && inChar < RPI_BUFFER_SIZE) {

				rpiPayloadSize = inChar;
				rpiReceiverState = 1;
				rpiCrcIn += inChar;
				
				// the receiving message is over the buffer size
			} else {

				rpiReceivingMessage = 0;
				rpiReceiverState = 0;
			}

			// expecting to receive the payload
		} else if (rpiReceiverState == 1) {

			// put the char in the buffer
			rpiRxBuffer[rpiBytesReceived++] = inChar;
			// add crc
			rpiCrcIn += inChar;

			// if the message should end, change state
			if (rpiBytesReceived >= rpiPayloadSize)
			rpiReceiverState = 2;

			// expecting to receive the crc
		} else if (rpiReceiverState == 2) {

			if (rpiCrcIn == inChar) {

				rpiMessageReceived = 1;
				rpiReceivingMessage = 0;
			} else {

				rpiReceivingMessage = 0;
				rpiReceiverState = 0;
			}
		}

		} else {

		// this character precedes every message
		if (inChar == 'a') {

			rpiCrcIn = inChar;

			rpiReceivingMessage = 1;
			rpiReceiverState = 0;
			rpiBytesReceived = 0;
		}
	}
	
	if (rpiMessageReceived) {
		
		messageHandler->messageBuffer = rpiRxBuffer+1;
		messageHandler->messageId = rpiRxBuffer[0];
		rpiMessageReceived = 0;
		return 1;
	}
	
	return 0;
}

/* -------------------------------------------------------------------- */
/*	Create a message with the blob										*/
/* -------------------------------------------------------------------- */

void sendPiBlob(uint64_t address) {
	
	uint8_t buffer[64];
	
	uint8_t idx = 0;
	
	buffer[idx++] = 'N';
	buffer[idx++] = rpiOk;
	
	writeFloatToBuffer(buffer, kalmanStates.elevator.position, idx);
	idx += 4;
	writeFloatToBuffer(buffer, kalmanStates.aileron.position, idx);
	idx += 4;
	
	writeFloatToBuffer(buffer, mpcSetpoints.elevator, idx);
	idx += 4;
	writeFloatToBuffer(buffer, mpcSetpoints.aileron, idx);
	idx += 4;

	writeFloatToBuffer(buffer, rpix, idx);
	idx += 4;
	writeFloatToBuffer(buffer, rpiy, idx);
	idx += 4;
	
	xbeeSendMessageTo(&buffer, idx, address);
}