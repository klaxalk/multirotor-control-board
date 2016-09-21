/*
 * raspberryPi.c
 *
 * Created: 8.10.2015 16:31:27
 *  Author: klaxalk
 */ 

#include "system.h"
#include "ioport.h"
#include "gumstix.h"
#include "communication.h"
#include "mpcHandler.h"
#include "xbee.h"

/* -------------------------------------------------------------------- */
/*	Variables for data reception from RPi								*/
/* -------------------------------------------------------------------- */

char gumstixRxBuffer[GUMSTIX_BUFFER_SIZE];

int16_t gumstixPayloadSize = 0;
int16_t gumstixBytesReceived;

char gumstixMessageReceived = 0;
char gumstixReceivingMessage = 0;
char gumstixReceiverState = 0;
char gumstixCrcIn = 0;

volatile float gumstix_x = 0;
volatile float gumstix_y = 0;
volatile float gumstix_z = 0;
volatile char gumstix_ok = 0;

volatile uint8_t message_size = 0;

/* -------------------------------------------------------------------- */
/*	Fetch the incoming char into a buffer, completes the message		*/
/* -------------------------------------------------------------------- */
int8_t gumstixParseChar(char inChar, gumstixMessageHandler_t * messageHandler) {

	if (gumstixReceivingMessage) {

		if (gumstixReceiverState == 1) {

			// put the char in the buffer
			gumstixRxBuffer[gumstixBytesReceived++] = inChar;
			
			// add crc
			gumstixCrcIn += inChar;

			// if the message should end, change state
			if (gumstixBytesReceived >= message_size)
				gumstixReceiverState = 2;

			// expecting to receive the crc
		} else if (gumstixReceiverState == 2) {

			if (gumstixCrcIn == inChar) {

				gumstixMessageReceived = 1;
				gumstixReceivingMessage = 0;
			} else {

				gumstixReceivingMessage = 0;
				gumstixReceiverState = 0;
			}
		}

	} else {

		// this character precedes every message
		if (inChar == 'x' || inChar == 'y' || inChar == 'z' || inChar == 'v') {

			if (inChar == 'v')
				message_size = 3;
			else
				message_size = 3;	

			gumstixCrcIn = inChar;

			gumstixReceivingMessage = 1;
			gumstixReceiverState = 1;
			gumstixBytesReceived = 0;
			
			gumstixRxBuffer[gumstixBytesReceived++] = inChar;
		}
	}
	
	if (gumstixMessageReceived) {
		
		messageHandler->messageBuffer = gumstixRxBuffer+1;
		messageHandler->messageId = gumstixRxBuffer[0];
		gumstixMessageReceived = 0;
		return 1;
	}
	
	return 0;
}