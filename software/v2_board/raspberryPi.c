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

/* -------------------------------------------------------------------- */
/*	Variables for data reception from RPi								*/
/* -------------------------------------------------------------------- */

receiverStateMachine_t rpiReceiver;

volatile float rpix = 0;
volatile float rpiy = 0;
volatile float rpiz = 0;
volatile char rpiOk = 0;

/* -------------------------------------------------------------------- */
/*	Fetch the incoming char into a buffer, completes the message		*/
/* -------------------------------------------------------------------- */
int8_t rpiParseChar(char inChar, rpiMessageHandler_t * messageHandler, receiverStateMachine_t * receiver) {
	
	// following byte should contain the payload size
	if (receiver->receiverState == RECEIVING_PAYLOAD_SIZE) {
		
		// does the message fit into our buffer?
		if (inChar >= 0 && inChar < RPI_BUFFER_SIZE) {
			
			receiver->payloadSize = inChar;
			receiver->receiverState = RECEIVING_PAYLOAD;
			receiver->checksum += inChar;
			
		} else {
			
			// restart the state machine
			receiver->receiverState = NOT_RECEIVING;
		}
	}
	
	// the new byte is the payload byte
	if (receiver->receiverState == RECEIVING_PAYLOAD) {
		
		receiver->rxBuffer[receiver->bytesReceived++] = inChar;
		receiver->checksum += inChar;
		
		// the payload is successfully received
		if (receiver->bytesReceived >= receiver->payloadSize) {
			
			receiver->receiverState = RECEIVING_CHECKSUM;
		}
	}
	
	if (receiver->receiverState == RECEIVING_CHECKSUM) {
		
		// if the checksum is correct
		if (receiver->checksum == inChar) {
			
			messageHandler->messageBuffer = receiver->rxBuffer+1;
			messageHandler->messageId = receiver->rxBuffer[0];
			receiver->receiverState = MESSAGE_RECEIVED;
			return 1;

		// if it is not correct, restart the receiver			
		} else {
			
			receiver->receiverState = NOT_RECEIVING;
		}
	}
	
	// waiting for the first byte of the message
	if (receiver->receiverState == NOT_RECEIVING) {
		
		// we received the starting byte
		if (inChar == RPI_INIT_BYTE) {

			receiver->checksum = inChar;
			receiver->bytesReceived = 0;
			receiver->receiverState = RECEIVING_PAYLOAD_SIZE;
		}
	}
	
	return 0;
}