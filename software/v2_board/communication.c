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
volatile float groundDistance = 3.1415;
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

	if (my_mavlink_parse_char(MAVLINK_COMM_0, incomingChar, &mavlinkMessage, &mavlinkStatus)) {

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

// decode the Mavlink incomming message from the px4flow sensor
uint8_t my_mavlink_parse_char(uint8_t chan, uint8_t c, mavlink_message_t* r_message, mavlink_status_t* r_mavlink_status) {

	mavlink_message_t* rxmsg = mavlink_get_channel_buffer(chan); ///< The currently decoded message
	mavlink_status_t* status = mavlink_get_channel_status(chan); ///< The current decode status
	int bufferIndex = 0;

	status->msg_received = 0;

	switch (status->parse_state) {
	case MAVLINK_PARSE_STATE_UNINIT:
	case MAVLINK_PARSE_STATE_IDLE:
		if (c == MAVLINK_STX) {
			status->parse_state = MAVLINK_PARSE_STATE_GOT_STX;
			rxmsg->len = 0;
			rxmsg->magic = c;
			mavlink_start_checksum(rxmsg);
		}
		break;

	case MAVLINK_PARSE_STATE_GOT_STX:
		if (status->msg_received
		        /* Support shorter buffers than the
		           default maximum packet size */
#if (MAVLINK_MAX_PAYLOAD_LEN < 255)
		        || c > MAVLINK_MAX_PAYLOAD_LEN
#endif
		   ) {
			status->buffer_overrun++;
			status->parse_error++;
			status->msg_received = 0;
			status->parse_state = MAVLINK_PARSE_STATE_IDLE;
		} else {
			// NOT counting STX, LENGTH, SEQ, SYSID, COMPID, MSGID, CRC1 and CRC2
			rxmsg->len = c;
			status->packet_idx = 0;
			mavlink_update_checksum(rxmsg, c);
			status->parse_state = MAVLINK_PARSE_STATE_GOT_LENGTH;
		}
		break;

	case MAVLINK_PARSE_STATE_GOT_LENGTH:
		rxmsg->seq = c;
		mavlink_update_checksum(rxmsg, c);
		status->parse_state = MAVLINK_PARSE_STATE_GOT_SEQ;
		break;

	case MAVLINK_PARSE_STATE_GOT_SEQ:
		rxmsg->sysid = c;
		mavlink_update_checksum(rxmsg, c);
		status->parse_state = MAVLINK_PARSE_STATE_GOT_SYSID;
		break;

	case MAVLINK_PARSE_STATE_GOT_SYSID:
		rxmsg->compid = c;
		mavlink_update_checksum(rxmsg, c);
		status->parse_state = MAVLINK_PARSE_STATE_GOT_COMPID;
		break;

	case MAVLINK_PARSE_STATE_GOT_COMPID:
		rxmsg->msgid = c;
		mavlink_update_checksum(rxmsg, c);
		if (rxmsg->len == 0) {
			status->parse_state = MAVLINK_PARSE_STATE_GOT_PAYLOAD;
		} else {
			status->parse_state = MAVLINK_PARSE_STATE_GOT_MSGID;
		}
		break;

	case MAVLINK_PARSE_STATE_GOT_MSGID:
		_MAV_PAYLOAD_NON_CONST(rxmsg)[status->packet_idx++] = (char)c;
		mavlink_update_checksum(rxmsg, c);
		if (status->packet_idx == rxmsg->len) {
			status->parse_state = MAVLINK_PARSE_STATE_GOT_PAYLOAD;
		}
		break;

	case MAVLINK_PARSE_STATE_GOT_PAYLOAD:
		status->parse_state = MAVLINK_PARSE_STATE_GOT_CRC1;
		_MAV_PAYLOAD_NON_CONST(rxmsg)[status->packet_idx] = (char) c;
		break;

	case MAVLINK_PARSE_STATE_GOT_CRC1:

		// Successfully got message
		status->msg_received = 1;
		status->parse_state = MAVLINK_PARSE_STATE_IDLE;
		_MAV_PAYLOAD_NON_CONST(rxmsg)[status->packet_idx+1] = (char) c;
		memcpy(r_message, rxmsg, sizeof(mavlink_message_t));

		break;
	}

	bufferIndex++;
	// If a message has been sucessfully decoded, check index
	if (status->msg_received == 1) {
		//while(status->current_seq != rxmsg->seq)
		//{
		//	status->packet_rx_drop_count++;
		//               status->current_seq++;
		//}
		status->current_rx_seq = rxmsg->seq;
		// Initial condition: If no packet has been received so far, drop count is undefined
		if (status->packet_rx_success_count == 0) {
			status->packet_rx_drop_count = 0;
		}
		// Count this packet as received
		status->packet_rx_success_count++;
	}

	r_mavlink_status->current_rx_seq = status->current_rx_seq+1;
	r_mavlink_status->packet_rx_success_count = status->packet_rx_success_count;
	r_mavlink_status->packet_rx_drop_count = status->parse_error;
	status->parse_error = 0;

	return status->msg_received;
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
