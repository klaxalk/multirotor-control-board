/*
 * commTask.c
 *
 *  Author: Tomas Baca
 */

#include "commTask.h"
#include "kalmanTask.h"

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

void commTask(void *p) {

	char inChar;
	char messageBuffer[20];
	int bytesReceived;
	char payloadSize = 0;
	char messageReceived = 0;
	char receivingMessage = 0;
	int receiverState = 0;

	char crc = 0;

	while (1) {

		if (xQueueReceive(usartRxQueue, &inChar, 100)) {

			if (receivingMessage) {

				// expecting to receive the payload size
				if (receiverState == 0) {

					if (inChar >= 0 && inChar <= 32) {

						payloadSize = inChar;
						receiverState = 1;
						crc += inChar;
					} else {

						receivingMessage = 0;
						receiverState = 0;
					}

				// expecting to receive the payload
				} else if (receiverState == 1) {

					messageBuffer[bytesReceived++] = inChar;
					crc += inChar;

					if (bytesReceived >= payloadSize) {

						receiverState = 2;
					}

				// expecting to receive the crc
				} else if (receiverState == 2) {

					if (crc == inChar) {

						messageReceived = 1;
						receivingMessage = 0;
					} else {

						receivingMessage = 0;
						receiverState = 0;
					}
				}

			} else {

				if (inChar == 'a') {

					crc = inChar;

					receivingMessage = 1;
					receiverState = 0;
					bytesReceived = 0;
				}
			}
		}

		if (messageReceived) {

			int idx = 0;

			//  read the message ID
			char messageId = readChar(messageBuffer, &idx);

			if (messageId == '1') {

				px4flowMessage mes;

				mes.dt = readFloat(messageBuffer, &idx);
				mes.elevatorSpeed = readFloat(messageBuffer, &idx);
				mes.aileronSpeed = readFloat(messageBuffer, &idx);
				mes.elevatorInput = readFloat(messageBuffer, &idx);
				mes.aileronInput = readFloat(messageBuffer, &idx);

				// send human readable values of¨
				/*
				char tempString[60];
				sprintf(tempString, "%8.6f %8.6f\n\r", vector_float_get(elevatorHandler.states, 1), vector_float_get(aileronHandler.states, 1));
				Usart4PutString(tempString);
				*/

				xQueueSend(comm2kalmanQueue, &mes, 100);

			}

			messageReceived = 0;
		}
	}
}
