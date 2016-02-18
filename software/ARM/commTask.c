/*
 * commTask.c
 *
 *  Author: Tomas Baca
 */

#include "commTask.h"
#include "kalmanTask.h"
#include "mpcTask.h"
#include "uart_driver.h"
#include <stdlib.h>
#include "kalman/aileron/aileronKalman.h"
#include "kalman/elevator/elevatorKalman.h"
#include "kalman/throttle/throttleKalman.h"

float readFloat(char * message, int * indexFrom) {

	float tempFloat;

	char * ukazatel = (char*) &tempFloat;
	*(ukazatel++) = message[(*indexFrom)++];
	*(ukazatel++) = message[(*indexFrom)++];
	*(ukazatel++) = message[(*indexFrom)++];
	*(ukazatel++) = message[(*indexFrom)++];

	return tempFloat;
}

int_least16_t readInt16(char * message, int * indexFrom) {

	int_least16_t tempInt;

	char * ukazatel = (char*) &tempInt;
	*(ukazatel++) = message[(*indexFrom)++];
	*(ukazatel++) = message[(*indexFrom)++];

	return tempInt;
}

char readChar(char * message, int * indexFrom) {

	char tempChar = message[(*indexFrom)++];

	return tempChar;
}

void sendFloat(const float var, char * crc) {

	char * ukazatel = (char*) &var;

	usart4PutChar(*(ukazatel));
	*crc += *(ukazatel);

	usart4PutChar(*(ukazatel+1));
	*crc += *(ukazatel+1);

	usart4PutChar(*(ukazatel+2));
	*crc += *(ukazatel+2);

	usart4PutChar(*(ukazatel+3));
	*crc += *(ukazatel+3);
}

void sendInt16(const int16_t var, char * crc) {

	char * ukazatel = (char*) &var;

	usart4PutChar(*(ukazatel));
	*crc += *(ukazatel);

	usart4PutChar(*(ukazatel+1));
	*crc += *(ukazatel+1);
}

void sendChar(const char var, char * crc) {

	usart4PutChar(var);
	*crc += var;
}

void commTask(void *p) {

	char crcOut = 0;

	// message from mpcTask
	mpc2commMessage_t mpcMessage;

	// message from kalmanTask
	kalman2commMessage_t kalmanMessage;

	// message from kalmanTask
	kalman2commThrottleMessage_t kalmanThrottleMessage;

	float tempFloat;
	int16_t tempInt;

	/* -------------------------------------------------------------------- */
	/*	Needed for receiving from xMega										*/
	/* -------------------------------------------------------------------- */
	char payloadSize = 0;
	char messageBuffer[XMEGA_BUFFER_SIZE];
	char inChar;
	int bytesReceived;
	char messageReceived = 0;
	char receivingMessage = 0;
	int receiverState = 0;
	char crcIn = 0;

	while (1) {

		/* -------------------------------------------------------------------- */
		/*	Receive char from usart												*/
		/* -------------------------------------------------------------------- */
		if (xQueueReceive(usartRxQueue, &inChar, 0)) {

			if (receivingMessage) {

				// expecting to receive the payload size
				if (receiverState == 0) {

					// check the message length
					if (inChar >= 0 && inChar < XMEGA_BUFFER_SIZE) {

						payloadSize = inChar;
						receiverState = 1;
						crcIn += inChar;

					// the receiving message is over the buffer size
					} else {

						receivingMessage = 0;
						receiverState = 0;
					}

				// expecting to receive the payload
				} else if (receiverState == 1) {

					// put the char in the buffer
					messageBuffer[bytesReceived++] = inChar;
					// add crc
					crcIn += inChar;

					// if the message should end, change state
					if (bytesReceived >= payloadSize)
						receiverState = 2;

				// expecting to receive the crc
				} else if (receiverState == 2) {

					if (crcIn == inChar) {

						messageReceived = 1;
						receivingMessage = 0;
						} else {

						receivingMessage = 0;
						receiverState = 0;
					}
				}

			} else {

				// this character precedes every message
				if (inChar == 'a') {

					crcIn = inChar;

					receivingMessage = 1;
					receiverState = 0;
					bytesReceived = 0;
				}
			}
		}

		/* -------------------------------------------------------------------- */
		/*	If there is a message from uart										*/
		/* -------------------------------------------------------------------- */
		if (messageReceived) {

			int idx = 0;

			//  read the message ID
			char messageId = readChar(messageBuffer, &idx);

			if (messageId == '1') {

				comm2kalmanMessage_t mes;

				tempFloat = readFloat(messageBuffer, &idx);
				if (fabs(tempFloat) < 1)
					mes.elevatorSpeed = tempFloat;

				tempFloat = readFloat(messageBuffer, &idx);
				if (fabs(tempFloat) < 1)
					mes.aileronSpeed = tempFloat;

				tempInt = readInt16(messageBuffer, &idx);
				if (abs(tempInt) < 300)
					mes.elevatorInput = (float) tempInt;

				tempInt = readInt16(messageBuffer, &idx);
				if (abs(tempInt) < 300)
					mes.aileronInput = (float) tempInt;

				xQueueSend(comm2kalmanQueue, &mes, 0);

			} else if (messageId == '2') {

				resetKalmanMessage_t mes;

				mes.elevatorPosition = 0;

				tempFloat = readFloat(messageBuffer, &idx);
				if (fabs(tempFloat) < 5)
					mes.elevatorPosition = tempFloat;

				mes.aileronPosition = 0;

				tempFloat = readFloat(messageBuffer, &idx);
				if (fabs(tempFloat) < 5)
					mes.aileronPosition = tempFloat;

				xQueueSend(resetKalmanQueue, &mes, 0);

			} else if (messageId == '3') {

				resetKalmanMessage_t mes;

				tempFloat = readFloat(messageBuffer, &idx);
				if (fabs(tempFloat) < 200)
					mes.elevatorPosition = tempFloat;
				else
					continue;

				tempFloat = readFloat(messageBuffer, &idx);
				if (fabs(tempFloat) < 200)
					mes.aileronPosition = tempFloat;
				else
					continue;

				xQueueSend(setKalmanQueue, &mes, 0);

			} else if (messageId == '4') {

				comm2kalmanThrottleMessage_t mes;

				tempFloat = readFloat(messageBuffer, &idx);
				if (fabs(tempFloat) < 10)
					mes.groundDistance = tempFloat;

				tempInt = readInt16(messageBuffer, &idx);
				if (abs(tempInt) < 10000)
					mes.batteryVoltage = (((float) (tempInt - 185)) / 220.5);

				tempInt = readInt16(messageBuffer, &idx);
				if (abs(tempInt) < 10000)
					mes.throttleInput = (float) tempInt;

				tempFloat = readFloat(messageBuffer, &idx);
				if (fabs(tempFloat) <= 1)
					mes.signalConfidence = tempFloat;

				xQueueSend(comm2kalmanThrottleQueue, &mes, 0);

			} else if (messageId == '5') {

				resetThrottleKalmanMessage_t mes;

				mes.throttlePosition = 0;

				tempFloat = readFloat(messageBuffer, &idx);
				if (fabs(tempFloat) < 5)
					mes.throttlePosition = tempFloat;

				xQueueSend(resetThrottleKalmanQueue, &mes, 0);

			} else if (messageId == '6') {

				resetThrottleKalmanMessage_t mes;

				tempFloat = readFloat(messageBuffer, &idx);
				if (fabs(tempFloat) < 200)
					mes.throttlePosition = tempFloat;
				else
					continue;

				xQueueSend(setThrottleKalmanQueue, &mes, 0);

			} else if (messageId == 's') {

				comm2mpcMessage_t comm2mpcMessage;

				comm2mpcMessage.messageType = SETPOINT;

				tempFloat = readFloat(messageBuffer, &idx);
				if (fabs(tempFloat) < 25)
					comm2mpcMessage.elevatorReference[0] = tempFloat;

				tempFloat = readFloat(messageBuffer, &idx);
				if (fabs(tempFloat) < 25)
					comm2mpcMessage.aileronReference[0] = tempFloat;

				xQueueSend(comm2mpcQueue, &comm2mpcMessage, 0);

			} else if (messageId == 't') {

				comm2mpcMessage_t comm2mpcMessage;

				comm2mpcMessage.messageType = TRAJECTORY;

				// receive the elevator trajectory key-point
				int i;
				for (i = 0; i < 5; i++) {

					tempFloat = readFloat(messageBuffer, &idx);
					if (fabs(tempFloat) < 25)
						comm2mpcMessage.elevatorReference[i] = tempFloat;
				}

				// receive the aileron trajectory key-point
				for (i = 0; i < 5; i++) {

					tempFloat = readFloat(messageBuffer, &idx);
					if (fabs(tempFloat) < 25)
						comm2mpcMessage.aileronReference[i] = tempFloat;
				}

				xQueueSend(comm2mpcQueue, &comm2mpcMessage, 0);

			}

			messageReceived = 0;
		}

		/* -------------------------------------------------------------------- */
		/*	Receive message from MPC task										*/
		/* -------------------------------------------------------------------- */
		if (xQueueReceive(mpc2commQueue, &mpcMessage, 0)) {

			/* -------------------------------------------------------------------- */
			/*	Send message to xMega												*/
			/* -------------------------------------------------------------------- */

			// clear the crc
			crcOut = 0;
			sendChar('a', &crcOut);			// this character initiates the transmission
			sendChar(1 + 4 + 2*4, &crcOut);			// this will be the size of the message

			sendChar('1', &crcOut);			// id of the message
			sendInt16((int16_t) mpcMessage.elevatorOutput, &crcOut);
			sendInt16((int16_t) mpcMessage.aileronOutput, &crcOut);

			sendFloat(mpcMessage.elevatorSetpoint, &crcOut);
			sendFloat(mpcMessage.aileronSetpoint, &crcOut);

			sendChar(crcOut, &crcOut);
		}

		/* -------------------------------------------------------------------- */
		/*	Receive message from kalman task with aileron and elevator			*/
		/* -------------------------------------------------------------------- */
		if (xQueueReceive(kalman2commQueue, &kalmanMessage, 0)) {

			/* -------------------------------------------------------------------- */
			/*	Send message to xMega												*/
			/* -------------------------------------------------------------------- */

			// clear the crc
			crcOut = 0;
			sendChar('a', &crcOut);			// this character initiates the transmission
			sendChar(1+2*5*4, &crcOut);		// this will be the size of the message

			sendChar('2', &crcOut);			// id of the message

			sendFloat(kalmanMessage.elevatorData[0], &crcOut);
			sendFloat(kalmanMessage.elevatorData[1], &crcOut);
			sendFloat(kalmanMessage.elevatorData[2], &crcOut);
			sendFloat(kalmanMessage.elevatorData[3], &crcOut);
			sendFloat(kalmanMessage.elevatorData[4], &crcOut);

			sendFloat(kalmanMessage.aileronData[0], &crcOut);
			sendFloat(kalmanMessage.aileronData[1], &crcOut);
			sendFloat(kalmanMessage.aileronData[2], &crcOut);
			sendFloat(kalmanMessage.aileronData[3], &crcOut);
			sendFloat(kalmanMessage.aileronData[4], &crcOut);

			sendChar(crcOut, &crcOut);
		}

		/* -------------------------------------------------------------------- */
		/*	Receive message from kalman task with throttle						*/
		/* -------------------------------------------------------------------- */
		if (xQueueReceive(kalman2commThrottleQueue, &kalmanThrottleMessage, 0)) {

			/* -------------------------------------------------------------------- */
			/*	Send message to xMega												*/
			/* -------------------------------------------------------------------- */

			// clear the crc
					crcOut = 0;
			sendChar('a', &crcOut);			// this character initiates the transmission
			sendChar(1+2*5*4, &crcOut);		// this will be the size of the message

			sendChar('3', &crcOut);			// id of the message

			sendFloat(kalmanThrottleMessage.throttleData[0], &crcOut);
			sendFloat(kalmanThrottleMessage.throttleData[1], &crcOut);
			sendFloat(kalmanThrottleMessage.throttleData[2], &crcOut);
			sendFloat(kalmanThrottleMessage.throttleData[3], &crcOut);

			sendChar(crcOut, &crcOut);
		}
	}
}
