/*
 * communication.c
 *
 * Created: 11.9.2014 13:22:24
 *  Author: Tomas Baca
 */ 

#include "communication.h"
#include "system.h"
#include "ioport.h"

// converts two characters in hex to one byte in binary
uint8_t hex2bin(const uint8_t * ptr) {
	
	uint8_t value = 0;
	uint8_t ch = *ptr;
	
	int i;
	for (i = 0; i < 2; i++) {
		
		if (ch >= '0' && ch <= '9')
		value = (value << 4) + (ch - '0');
		else if (ch >= 'A' && ch <= 'F')
		value = (value << 4) + (ch - 'A' + 10);
		else if (ch >= 'a' && ch <= 'f')
		value = (value << 4) + (ch - 'a' + 10);
		else
		return value;
		ch = *(++ptr);
	}
	
	return value;
}

// parses a single uint8_t from the buffer
uint8_t readUint8(char * message, int * indexFrom) {

	char tempChar = message[(*indexFrom)++];

	return tempChar;
}

// write 4 bytes of float number to the buffer on position
void writeFloatToBuffer(char * buffer, const float input, uint16_t position) {
	
	uint8_t * tempPtr = (uint8_t *) &input;
	
	*(buffer+position) = *tempPtr;
	*(buffer+position+1) = *(tempPtr+1);
	*(buffer+position+2) = *(tempPtr+2);
	*(buffer+position+3) = *(tempPtr+3);
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

