/*
 * communication.c
 *
 * Created: 11.9.2014 13:22:24
 *  Author: Tomas Baca
 */ 

#include "communication.h"
#include "system.h"
#include "ioport.h"

#if FLIGHTCTRL_DATA_RECEIVE == ENABLED

// parse the whole message from the FlightCtrl (angles)
void parseFlightCtrlMessage() {

	Decode64();

	char* dummy;
	int16_t tempAngle;

	dummy = (char*)(&tempAngle);

	dummy[0] = pRxData[0];
	dummy[1] = pRxData[1];

	if (RxdBuffer[2] == 'X') {

		pitchBoardAngle = tempAngle;

	} else if (RxdBuffer[2] == 'Y') {

		rollBoardAngle = tempAngle;
	}

#if FRAME_ORIENTATION == X_COPTER

	// rotate the angles to the xCopter reference frame
	pitchAngle = (pitchBoardAngle-rollBoardAngle)/2;
	rollAngle = (pitchBoardAngle+rollBoardAngle)/2;

#elif FRAME_ORIENTATION == PLUS_COPTER

	pitchAngle = pitchBoardAngle;
	rollAngle = rollBoardAngle;

#endif

}

#endif // FLIGHTCTRL_DATA_RECEIVE == ENABLED

//~ --------------------------------------------------------------------
//~ Processes message from px4flow
//~ --------------------------------------------------------------------

#if PX4FLOW_DATA_RECEIVE == ENABLED

//px4flow values
volatile float groundDistance = 0.3;
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

			if (mavlinkStatus.parse_error == 0) {

				opticalFlowDataFlag = 1;
			}

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

//~ --------------------------------------------------------------------
//~ Processes message from flightCtrl
//~ --------------------------------------------------------------------

#if (FLIGHTCTRL_DATA_RECEIVE == ENABLED) || (ATOM_DATA_RECEIVE == ENABLED)

void flightCtrlParseChar(char incomingChar) {

	static unsigned int crc;
	static unsigned char crc1,crc2,buf_ptr;
	static unsigned char UartState = 0;
	unsigned char CrcOkay = 0;

	if (buf_ptr >= MAX_SENDE_BUFF) {
		UartState = 0;
	}

	if (incomingChar == '\r' && UartState == 2) {
		UartState = 0;
		crc -= RxdBuffer[buf_ptr-2];
		crc -= RxdBuffer[buf_ptr-1];
		crc %= 4096;
		crc1 = '=' + crc / 64;
		crc2 = '=' + crc % 64;
		CrcOkay = 0;
		if ((crc1 == RxdBuffer[buf_ptr-2]) && (crc2 == RxdBuffer[buf_ptr-1])) {
			CrcOkay = 1;
		} else {
			CrcOkay = 0;
			CntCrcError++;
		};
		if (!flightCtrlDataFlag && CrcOkay) { // Celá správa pøijata
			flightCtrlDataFlag = 1;
			flightCtrlDataBeingReceived = 0;
			flightCtrlByteReceive = 0;
			BytesReceiving = buf_ptr + 1;
			RxdBuffer[buf_ptr] = '\r';
		}
	} else
		switch (UartState) {
		case 0:
			if (incomingChar == '#' && !flightCtrlDataFlag) {
				UartState = 1;    // Start char
			}
			buf_ptr = 0;
			RxdBuffer[buf_ptr++] = incomingChar;
			crc = incomingChar;
			break;
		case 1: // Vyhonocení adresy
			UartState++;
			RxdBuffer[buf_ptr++] = incomingChar;
			crc += incomingChar;
			break;
		case 2: //  Bere data
			RxdBuffer[buf_ptr] = incomingChar;
			if (buf_ptr < MAX_EMPFANGS_BUFF) {
				buf_ptr++;
			} else {
				UartState = 0;
			}
			crc += incomingChar;
			break;
		default:
			UartState = 0;
			break;
		}
}

// decode the base64 encoded data
void Decode64(void) {

	unsigned char a,b,c,d;
	unsigned char x,y,z;
	unsigned char ptrIn = 3; // start at begin of data block
	unsigned char ptrOut = 3;
	unsigned char len = BytesReceiving - 6;

	while (len) {

		a = RxdBuffer[ptrIn++] - '=';
		b = RxdBuffer[ptrIn++] - '=';
		c = RxdBuffer[ptrIn++] - '=';
		d = RxdBuffer[ptrIn++] - '=';

		x = (a << 2) | (b >> 4);
		y = ((b & 0x0f) << 4) | (c >> 2);
		z = ((c & 0x03) << 6) | d;

		if (len--) {
			RxdBuffer[ptrOut++] = x;
		} else {
			break;
		}
		if (len--) {
			RxdBuffer[ptrOut++] = y;
		} else {
			break;
		}
		if (len--) {
			RxdBuffer[ptrOut++] = z;
		}	else {
			break;
		}
	}

	pRxData = (signed char*) &RxdBuffer[3]; // dekodování zaèíná 4. bytem
	RxDataLen = ptrOut - 3;  // kolik bylo dekodováno bytù?
}

#endif

#if GUMSTIX_DATA_RECEIVE == ENABLED

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

	if ((gumstixParseCharByte == 3) && (gumstixParseCharCrc == incomingChar)) { // we have the whole int ret

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
				// led_control_on();
				gumstixDataFlag = 1;
			} else {
				// led_control_off();
			}
			break;
		}

		gumstixParseCharState = 0;
	}
}

#endif // GUMSTIX_DATA_RECEIVE == ENABLED
