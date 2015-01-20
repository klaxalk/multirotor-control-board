/*
 * communication.c
 *
 * Created: 11.9.2014 13:22:24
 *  Author: Tomas Baca
 */ 

#include "communication.h"
#include "system.h"
#include "ioport.h"

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

float readFloat(char * message, int * indexFrom) {

	float tempFloat;

	char * ukazatel = (char*) &tempFloat;
	*(ukazatel++) = message[(*indexFrom)++];
	*(ukazatel++) = message[(*indexFrom)++];
	*(ukazatel++) = message[(*indexFrom)++];
	*(ukazatel++) = message[(*indexFrom)++];

	return tempFloat;
}

int16_t readInt16(char * message, int * indexFrom) {

	int16_t tempInt;

	char * ukazatel = (char*) &tempInt;
	*(ukazatel++) = message[(*indexFrom)++];
	*(ukazatel++) = message[(*indexFrom)++];

	return tempInt;
}

char readChar(char * message, int * indexFrom) {

	char tempChar = message[(*indexFrom)++];

	return tempChar;
}

void sendFloat(UsartBuffer * usartBuffer, const float var, char * crc) {
	
	char * ukazatel = (char*) &var;
	
	usartBufferPutByte(usartBuffer, *(ukazatel), 10);
	*crc += *(ukazatel);
	
	usartBufferPutByte(usartBuffer, *(ukazatel+1), 10);
	*crc += *(ukazatel+1);
	
	usartBufferPutByte(usartBuffer, *(ukazatel+2), 10);
	*crc += *(ukazatel+2);
	
	usartBufferPutByte(usartBuffer, *(ukazatel+3), 10);
	*crc += *(ukazatel+3);
}

void sendInt16(UsartBuffer * usartBuffer, const int16_t var, char * crc) {
	
	char * ukazatel = (char*) &var;
	
	usartBufferPutByte(usartBuffer, *(ukazatel), 10);
	*crc += *(ukazatel);
	
	usartBufferPutByte(usartBuffer, *(ukazatel+1), 10);
	*crc += *(ukazatel+1);
}

void sendChar(UsartBuffer * usartBuffer, const char var, char * crc) {
	
	usartBufferPutByte(usartBuffer, var, 10);
	*crc += var;
}

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