/*
 * commTask.c
 *
 *  Author: Tomas Baca
 */

#include "commTask.h"
#include "kalmanTask.h"
#include "mpcTask.h"
#include "uart_driver.h"

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
	mpcOutputMessage mpcMessage;

	float tempFloat;

	/* -------------------------------------------------------------------- */
	/*	Needed for receiving from xMega										*/
	/* -------------------------------------------------------------------- */
	char payloadSize = 0;
	char messageBuffer[64];
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

					if (inChar >= 0 && inChar <= 63) {

						payloadSize = inChar;
						receiverState = 1;
						crcIn += inChar;
					} else {

						receivingMessage = 0;
						receiverState = 0;
					}

				// expecting to receive the payload
				} else if (receiverState == 1) {

					messageBuffer[bytesReceived++] = inChar;
					crcIn += inChar;

					if (bytesReceived >= payloadSize) {

						receiverState = 2;
					}

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

				px4flowMessage mes;

				tempFloat = readFloat(messageBuffer, &idx);
				if (fabs(tempFloat) < 1)
					mes.dt = tempFloat;

				tempFloat = readFloat(messageBuffer, &idx);
				if (fabs(tempFloat) < 1)
					mes.elevatorSpeed = tempFloat;

				tempFloat = readFloat(messageBuffer, &idx);
				if (fabs(tempFloat) < 1)
					mes.aileronSpeed = tempFloat;

				tempFloat = readFloat(messageBuffer, &idx);
				if (fabs(tempFloat) < 100)
					mes.elevatorInput = tempFloat;

				tempFloat = readFloat(messageBuffer, &idx);
				if (fabs(tempFloat) < 100)
					mes.aileronInput = tempFloat;

				// send human readable values of
				/*
				char tempString[60];
				sprintf(tempString, "%4.1f %4.1f %4.1f %4.1f\n\r", mes.elevatorSpeed, mes.aileronSpeed, mes.elevatorInput, mes.aileronInput);
				Usart4PutString(tempString);
				*/

				xQueueSend(comm2kalmanQueue, &mes, 0);

			} else if (messageId == '2') {

				vector_float_set_zero(elevatorHandler.states);
				vector_float_set_zero(aileronHandler.states);
			}

			messageReceived = 0;
		}

		/* -------------------------------------------------------------------- */
		/*	Receive message from MPC											*/
		/* -------------------------------------------------------------------- */
		if (xQueueReceive(mpc2commQueue, &mpcMessage, 0)) {

			/*
			char tempString[60];
			sprintf(tempString, "%2.3f %2.3f %d %d\n\r", vector_float_get(elevatorHandler.states, 1), vector_float_get(aileronHandler.states, 1), mpcMessage.elevatorOutput, mpcMessage.aileronOutput);
			Usart4PutString(tempString);
			*/

			/* -------------------------------------------------------------------- */
			/*	Send message to xMega												*/
			/* -------------------------------------------------------------------- */

			// clear the crc
			crcOut = 0;
			sendChar('a', &crcOut);		// this character initiates the transmition
			sendChar(1+4, &crcOut);		// this will be the size of the message

			sendChar('1', &crcOut);		// id of the message
			sendInt16(mpcMessage.elevatorOutput, &crcOut);
			sendInt16(mpcMessage.aileronOutput, &crcOut);

			sendChar(crcOut, &crcOut);
		}
	}
}
