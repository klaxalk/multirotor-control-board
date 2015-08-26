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
/*	Variables for data reception from Argos								*/
/* -------------------------------------------------------------------- */

char argosRxBuffer[ARGOS_BUFFER_SIZE];

int16_t argosPayloadSize = 0;
int16_t argosBytesReceived;

char argosMessageReceived = 0;
char argosReceivingMessage = 0;
char argosReceiverState = 0;
char argosCrcIn = 0;

/* -------------------------------------------------------------------- */
/*	Fetch the incoming char into a buffer, completes the message		*/
/* -------------------------------------------------------------------- */
int8_t argosParseChar(char inChar, argosMessageHandler_t * messageHandler) {

	if (argosReceivingMessage) {

		// expecting to receive the payload size
		if (argosReceiverState == 0) {

			argosRxBuffer[0] = inChar;
			
		} else if (argosReceiverState == 1) {
			
			argosRxBuffer[1] = inChar;
			
			argosPayloadSize = hex2bin((uint8_t *) &argosRxBuffer);
			
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
				argosCrcIn += inChar;

				// if the message should end, change state
				if (argosBytesReceived >= argosPayloadSize)
					argosReceiverState = 3;

		} else if (argosReceiverState == 3) {
			
			// check the crc
			if (argosCrcIn == hex2bin((uint8_t *) (&argosRxBuffer + argosBytesReceived - 2))) {
				
				argosMessageReceived = 1;
				argosReceivingMessage = 0;
				
				uint8_t i;
				for (i = 0; i < argosPayloadSize/2; i++) {
					
					argosRxBuffer[i] = hex2bin((uint8_t *) (&argosRxBuffer + (i-1)*2));
				}
				
			} else {

				argosReceivingMessage = 0;
				argosReceiverState = 0;
			}
		}

	} else {

		// this character precedes every message
		if (inChar == 'A') {

			argosCrcIn = inChar;

			argosReceivingMessage = 1;
			argosReceiverState = 0;
			argosBytesReceived = 0;
		}
	}
	
	if (argosMessageReceived) {
		
		messageHandler->messageBuffer = argosRxBuffer+1;
		messageHandler->messageId = argosRxBuffer[0];
		argosMessageReceived = 0;
		return 1;
	}
	
	return 0;
}