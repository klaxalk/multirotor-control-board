/*
 * communication.c
 *
 * Created: 11.9.2014 13:22:24
 *  Author: Tomas Baca
 */ 

#include "communication.h"
#include "system.h"
#include "ioport.h"

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

//~ --------------------------------------------------------------------
//~ Processes message from px4flow
//~ --------------------------------------------------------------------

// data from the sensor
volatile float groundDistance = 0;
volatile float elevatorSpeed = 0;
volatile float aileronSpeed = 0;
volatile uint8_t px4Confidence = 0;

// variables used by the mavlink library
mavlink_message_t mavlinkMessage;
mavlink_status_t mavlinkStatus;
int8_t mavlinkFlag = 0;
mavlink_optical_flow_t opticalFlowData;
int8_t opticalFlowDataFlag = 0;

int8_t px4flowParseChar(uint8_t incomingChar) {

	if (mavlink_parse_char(MAVLINK_COMM_0, incomingChar, &mavlinkMessage, &mavlinkStatus)) {

		switch (mavlinkMessage.msgid) {
		case MAVLINK_MSG_ID_OPTICAL_FLOW: {

			opticalFlowDataFlag = 1;
			return 1;

		}
		break;
		default:
			//Do nothing
			break;
		}
	}
	return 0;
}

