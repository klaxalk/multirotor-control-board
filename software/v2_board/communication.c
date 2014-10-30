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

#if PX4FLOW_DATA_RECEIVE == ENABLED

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

#endif // PX4FLOW_DATA_RECEIVE

#if GUMSTIX_DATA_RECEIVE == ENABLED

/* -------------------------------------------------------------------- */
/*	Variables for gumstix												*/
/* -------------------------------------------------------------------- */
volatile unsigned char gumstixParseCharState = 0;
volatile unsigned char gumstixParseCharByte = 0;
volatile int16_t gumstixParseTempInt;
volatile int16_t xPosGumstixNew = 0;
volatile int16_t yPosGumstixNew = 0;
volatile int16_t zPosGumstixNew = 0;
volatile float elevatorGumstix = 0;
volatile float aileronGumstix = 0;
volatile float throttleGumstix = 0;
volatile int8_t validGumstix = 0;
volatile int8_t gumstixDataFlag = 0;
volatile unsigned char gumstixParseCharCrc = 0;

void gumstixParseChar(unsigned char incomingChar) {

	if (gumstixParseCharByte == 2) {
		gumstixParseCharByte++;
	} else {
		gumstixParseCharCrc += incomingChar;
	}

	if (gumstixParseCharState == 0) {

		switch (incomingChar) {

		case 'x':
			gumstixParseCharState = 1;
			break;
		case 'y':
			gumstixParseCharState = 2;
			break;
		case 'z':
			gumstixParseCharState = 3;
			break;
		case 'v':
			gumstixParseCharState = 4;
			break;
		}

		gumstixParseCharByte = 0;
		gumstixParseTempInt = 0;
		gumstixParseCharCrc = incomingChar;

	} else if (gumstixParseCharByte < 2) {

		char* gumstixParseTempIntPointer = (char*) &gumstixParseTempInt;
		*(gumstixParseTempIntPointer+gumstixParseCharByte) = incomingChar;

		gumstixParseCharByte++;
	}

	if ((gumstixParseCharByte == 3) && (gumstixParseCharCrc == incomingChar)) { // we have the whole int red

		switch (gumstixParseCharState) {

		case 1:
			xPosGumstixNew = gumstixParseTempInt;
			break;
		case 2:
			yPosGumstixNew = gumstixParseTempInt;
			break;
		case 3:
			zPosGumstixNew = gumstixParseTempInt;
			break;
		case 4:
			validGumstix = gumstixParseTempInt;

			if (validGumstix == 1) {
				
				gumstixDataFlag = 1;
				led_blue_on();
			} else {
				
				led_blue_off();
			}
			break;
		}

		gumstixParseCharState = 0;
	}
}

#endif // GUMSTIX_DATA_RECEIVE == ENABLED
