#include "system.h"
#include "ioport.h"
#include "argos3D.h"
#include "communication.h"

uint8_t numberOfObstacles = 0;
float obstaclesX1[MAX_NUMBER_OF_OBSTACLES];
float obstaclesX2[MAX_NUMBER_OF_OBSTACLES];
float obstaclesY1[MAX_NUMBER_OF_OBSTACLES];
float obstaclesY2[MAX_NUMBER_OF_OBSTACLES];

/* -------------------------------------------------------------------- */
/*	Variables for data reception from argos								*/
/* -------------------------------------------------------------------- */

char argosRxBuffer[ARGOS_BUFFER_SIZE];

int16_t argosPayloadSize = 0;
int16_t argosBytesReceived;

uint8_t argosMessageReceived = 0;
uint8_t argosReceivingMessage = 0;
uint8_t argosReceiverState = 0;
uint8_t argosCrcIn = 0;

/* -------------------------------------------------------------------- */
/*	Fetch the incoming char into a buffer, completes the message		*/
/* -------------------------------------------------------------------- */
int8_t argosParseChar(char inChar, argosMessageHandler_t * messageHandler) {

	if (argosReceivingMessage) {

		// expecting to receive the payload size
		if (argosReceiverState == 0) {

			argosRxBuffer[argosBytesReceived++] = inChar;
			argosReceiverState = 1;
			argosCrcIn += inChar;
			
			} else if (argosReceiverState == 1) {
			
			argosRxBuffer[argosBytesReceived++] = inChar;
			
			argosPayloadSize = hex2bin((uint8_t *) &argosRxBuffer);
			
			argosCrcIn += inChar;
			
			if (argosPayloadSize < ARGOS_BUFFER_SIZE) {
				
				argosReceiverState = 2;
				} else {
				
				argosReceiverState = 0;
				argosReceivingMessage = 0;
			}
			
			// expecting to receive the payload
			} else if (argosReceiverState == 2) {

			// put the char in the buffer
			argosRxBuffer[argosBytesReceived++] = inChar;
			
			// add crc
			if (argosBytesReceived <= argosPayloadSize)
			argosCrcIn += inChar;
			
			// if the message should end, change state
			if (argosBytesReceived >= argosPayloadSize + 2) // the payload size itself is not included in the payload size
			argosReceiverState = 3;

			} else if (argosReceiverState == 3) {
			
			// check the crc
			if (argosCrcIn == hex2bin((uint8_t *) (argosRxBuffer + argosBytesReceived - 2))) {
				
				argosMessageReceived = 1;
				argosReceivingMessage = 0;
				
				uint8_t i;
				for (i = 0; i < argosBytesReceived/2; i++) {
					
					argosRxBuffer[i] = hex2bin((uint8_t *) (argosRxBuffer + i*2));
				}
				
				} else {

				argosReceivingMessage = 0;
				argosReceiverState = 0;
			}
		}

		} else {

		// this character precedes every message
		if (inChar == 'A') {

			argosCrcIn = 0;
			argosReceivingMessage = 1;
			argosReceiverState = 0;
			argosBytesReceived = 0;
		}
	}
	
	if (argosMessageReceived) {
		
		messageHandler->messageBuffer = argosRxBuffer+2;
		messageHandler->messageId = argosRxBuffer[1];
		argosMessageReceived = 0;
		return 1;
	}
	
	return 0;
}