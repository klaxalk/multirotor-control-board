/*
 * File name: pc_communication.c
 * Date:      2014/11/04 15:21
 * Author:    Jan Chudoba
 */

#include "pc_communication.h"
#include "debugcomm.h"

// communication protocol:
// - start character '^'
// - message type (1-3 letter identifier)
// - comma-separated list of parameters
// - end of line chaaracter '\n'
//

// recognized commands (PC->MAV)
// V,0.000,0.000,0.000,0.000 - velocity control

#define PC_COMM_HEADER_CHARACTER '^'
#define PC_PARSE_MESSAGE_TYPE_MAX_LENGTH 3
#define PARAMETER_PARSE_INIT 0xFF

static volatile uint8_t pcParseState = 0;
static volatile uint8_t pcParsePtr = 0;
static volatile char pcParseMessageTypeBuffer[PC_PARSE_MESSAGE_TYPE_MAX_LENGTH+1];
static volatile TPcMessageType pcParseMessageType = PC_MSG_NONE;
static volatile uint8_t pcParseMessageComplete = 0;
static volatile uint8_t pcParseParameter = PARAMETER_PARSE_INIT;

TPcMessageType pcParseVelocityMessage(uint8_t incomingChar);

TPcMessageType pcParseChar(uint8_t incomingChar)
{
	if (incomingChar == ' ') return 0;
	TPcMessageType result = PC_MSG_NONE;
	switch (pcParseState) {
	case 0: // wait for incoming character
		if (incomingChar == PC_COMM_HEADER_CHARACTER) {
			pcParsePtr = 0;
			pcParseState = 1;
			//debugMessage("PC parse 0->1\r\n");
		}
		break;
	case 1: // parse message type
		if (incomingChar != ',') {
			if (incomingChar == PC_COMM_HEADER_CHARACTER) {
				pcParsePtr = 0;
			} else {
				if (pcParsePtr < PC_PARSE_MESSAGE_TYPE_MAX_LENGTH) {
					pcParseMessageTypeBuffer[pcParsePtr++] = incomingChar;
				}
			}
		} else {
			pcParseMessageType = PC_MSG_NONE;
			if (pcParsePtr==1 && pcParseMessageTypeBuffer[0] == 'V') {
				pcParseMessageType = PC_MSG_VELOCITY;
				pcParseParameter = PARAMETER_PARSE_INIT;
				//debugMessage("PC parse MSG V\r\n");
			} else {
				// unknown message
				pcParseMessageTypeBuffer[pcParsePtr] = 0;
				debugMessageF("PC parse MSG UNKN '%s'\r\n", pcParseMessageTypeBuffer);
				pcParseState = 0;
			}
			if (pcParseMessageType != PC_MSG_NONE) pcParseState = 2;
		}
		break;
	case 2: // parse parameters
		result = PC_MSG_NUMBER;
		switch (pcParseMessageType) {
		case PC_MSG_VELOCITY:
			result = pcParseVelocityMessage(incomingChar);
			break;
		case PC_MSG_NONE:
		case PC_MSG_NUMBER:
			;
		}
		if (result == PC_MSG_NUMBER) {
			// error in parameters parsing
			pcParseState = 0;
			result = PC_MSG_NONE;
			debugMessage("PC parse MSG ARGERR\r\n");
		} else if (result != PC_MSG_NONE) {
			// received whole message
			pcParseState = 0;
		}
	}
	return result;
}

static volatile float float_value = 0;
static volatile uint8_t float_dot = 0;
static volatile float float_divider = 1.0f;
static volatile uint8_t float_sign = 0;

volatile float pcVelocityMessageValue[4];

TPcMessageType pcParseVelocityMessage(uint8_t incomingChar)
{
	if (pcParseParameter == PARAMETER_PARSE_INIT) {
		pcParseParameter = 0;
		float_value = 0;
		float_dot = 0;
		float_divider = 1.0f;
		float_sign = 0;
	}
	if (incomingChar == ' ') return PC_MSG_NONE;
	if (incomingChar=='\n' || incomingChar=='\r') {
		if (pcParseParameter == 3) {
			pcVelocityMessageValue[3] = (float_sign?(-1):1) * float_value / float_divider;
			return PC_MSG_VELOCITY;
		}
		// error
		return PC_MSG_NUMBER;
	} else
	if (incomingChar==',') {
		if (pcParseParameter < 4) {
			debugMessageF("PC parse num %d\r\n", (int)float_value);
			pcVelocityMessageValue[pcParseParameter++] = (float_sign?(-1):1) * float_value / float_divider;
			float_value = 0;
			float_dot = 0;
			float_divider = 1.0f;
			float_sign = 0;
		} else {
			// error
			return PC_MSG_NUMBER;
		}
	} else {
		if (incomingChar >= '0' && incomingChar <= '9') {
			uint8_t n = incomingChar - '0';
			float_value = 10.0f * float_value + (float)n;
			if (float_dot) float_divider *= 10.0f;
		} else if (incomingChar == '.') {
			float_dot = 1;
		} else if (incomingChar == '-') {
			float_sign = 1;
		} else {
			// error
			return PC_MSG_NUMBER;
		}
	}
	return PC_MSG_NONE;
}

void pcSendMessage(char * message) {
	// TODO ...
}

void pcResetAll()
{
	pcSendMessage((char*)"^R\n");
}

void pcNewTrajectoryPoint()
{
	pcSendMessage((char*)"^N\n");
}

void pcGotoHome()
{
	pcSendMessage((char*)"^G\n");
}

void pcFollowTrajectory()
{
	pcSendMessage((char*)"^T\n");
}

void pcStay()
{
	pcSendMessage((char*)"^S\n");
}

void pcSetLogging(uint8_t enable)
{
	char * message = (char*) "^L0\n";
	if (enable) message[2] = '1';
	pcSendMessage(message);
}



/* end of pc_communication.c */
