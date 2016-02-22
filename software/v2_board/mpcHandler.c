/*
 * mpcHandler.c
 *
 * Created: 10.2.2015 10:46:19
 *  Author: Tomas Baca
 */ 

#include "mpcHandler.h"

/* -------------------------------------------------------------------- */
/*	Variables for data reception from STM u-controller					*/
/* -------------------------------------------------------------------- */

char stmRxBuffer[STM_BUFFER_SIZE];

int16_t payloadSize = 0;
int16_t bytesReceived;

char stmMessageReceived = 0;
char receivingMessage = 0;
char receiverState = 0;
char crcIn = 0;

/* -------------------------------------------------------------------- */
/*	This structure holds all states estimated by the kalman filter		*/
/* -------------------------------------------------------------------- */

volatile kalmanStates_t kalmanStates;

/* -------------------------------------------------------------------- */
/*	This structure holds setpoints for MPC								*/
/* -------------------------------------------------------------------- */

volatile mpcSetpoints_t mpcSetpoints;

/* -------------------------------------------------------------------- */
/*	These variables hold actions from MPC								*/
/* -------------------------------------------------------------------- */

volatile int16_t mpcElevatorOutput = 0;
volatile int16_t mpcAileronOutput = 0;
volatile int16_t mpcThrottleOutput = 0;

/* -------------------------------------------------------------------- */
/*	Initialize the local copy of kalman states to ZERO					*/
/* -------------------------------------------------------------------- */
void initializeKalmanStates() {
	
	kalmanStates.elevator.position = 0;
	kalmanStates.elevator.velocity = 0;
	kalmanStates.elevator.acceleration = 0;
	kalmanStates.elevator.acceleration_input = 0;
	kalmanStates.elevator.acceleration_error = 0;
	
	kalmanStates.aileron.position = 0;
	kalmanStates.aileron.velocity = 0;
	kalmanStates.aileron.acceleration = 0;
	kalmanStates.aileron.acceleration_input = 0;
	kalmanStates.aileron.acceleration_error = 0;
	
	kalmanStates.throttle.position = 0;
	kalmanStates.throttle.velocity = 0;
	kalmanStates.throttle.acceleration = 0;
	kalmanStates.throttle.omega = 0;
}

/* -------------------------------------------------------------------- */
/*	Fetch the incoming char into a buffer, completes the message		*/
/* -------------------------------------------------------------------- */
int8_t stmParseChar(char inChar, stmMessageHandler_t * messageHandler) {

	if (receivingMessage) {

		// expecting to receive the payload size
		if (receiverState == 0) {

			// check the message length
			if (inChar >= 0 && inChar < STM_BUFFER_SIZE) {

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
			stmRxBuffer[bytesReceived++] = inChar;
			// add crc
			crcIn += inChar;

			// if the message should end, change state
			if (bytesReceived >= payloadSize)
			receiverState = 2;

		// expecting to receive the crc
		} else if (receiverState == 2) {

			if (crcIn == inChar) {

				stmMessageReceived = 1;
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
	
	if (stmMessageReceived) {
		
		messageHandler->messageBuffer = stmRxBuffer+1;
		messageHandler->messageId = stmRxBuffer[0];
		stmMessageReceived = 0;
		return 1;
	}
	
	return 0;
}

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

/* ----------------------------------------------------------------------- */
/* Send measurement and system input values of aileron and elevator to STM */
/* ----------------------------------------------------------------------- */
void stmSendMeasurement(float elevSpeed, float aileSpeed, int16_t elevInput, int16_t aileInput) {
	
	char crc = 0;
	
	sendChar(usart_buffer_stm, 'a', &crc);		// this character initiates the transmission
	
	sendChar(usart_buffer_stm, 1+12, &crc);		// this will be the size of the message
	sendChar(usart_buffer_stm, '1', &crc);		// id of the message
	
	// sends the payload
	sendFloat(usart_buffer_stm, elevSpeed, &crc);
	sendFloat(usart_buffer_stm, aileSpeed, &crc);
	sendInt16(usart_buffer_stm, elevInput, &crc);
	sendInt16(usart_buffer_stm, aileInput, &crc);	
	
	
	// at last send the crc, ends the transmission
	sendChar(usart_buffer_stm, crc, &crc);
}


/* -------------------------------------------------------------------- */
/*	Send request for reset the kalman states							*/
/* -------------------------------------------------------------------- */
void stmResetKalman(float initElevator, float initAileron) {
	
	char crc = 0;
	
	sendChar(usart_buffer_stm, 'a', &crc);		// this character initiates the transmission
	sendChar(usart_buffer_stm, 1 + 2*4, &crc);		// this will be the size of the message
	
	sendChar(usart_buffer_stm, '2', &crc);		// id of the message
	
	sendFloat(usart_buffer_stm, initElevator, &crc);
	sendFloat(usart_buffer_stm, initAileron, &crc);

	// at last send the crc, ends the transmission
	sendChar(usart_buffer_stm, crc, &crc);
}

/* -------------------------------------------------------------------- */
/*	Override the current kalman's integrated position					*/
/* -------------------------------------------------------------------- */
void stmSetKalmanPosition(float elevatorPos, float aileronPos) {
	
	char crc = 0;
	
	sendChar(usart_buffer_stm, 'a', &crc);		// this character initiates the transmission
	sendChar(usart_buffer_stm, 1 + 2*4, &crc);		// this will be the size of the message
	
	sendChar(usart_buffer_stm, '3', &crc);		// id of the message
	
	sendFloat(usart_buffer_stm, elevatorPos, &crc);
	sendFloat(usart_buffer_stm, aileronPos, &crc);
	
	// at last send the crc, ends the transmission
	sendChar(usart_buffer_stm, crc, &crc);
}

/* --------------------------------------------------------------------- */
/* Send measurement and system input values of throttle to STM			 */
/* --------------------------------------------------------------------- */
void stmSendThrottleMeasurement(float groundDist, int16_t battLvl, int16_t throInput, float groundDistConf) {
	
	char crc = 0;
	
	sendChar(usart_buffer_stm, 'a', &crc);		// this character initiates the transmission
	
	sendChar(usart_buffer_stm, 1+12, &crc);		// this will be the size of the message
	sendChar(usart_buffer_stm, '4', &crc);		// id of the message
	
	// sends the payload
	sendFloat(usart_buffer_stm, groundDist, &crc);
	sendInt16(usart_buffer_stm, battLvl, &crc);
	sendInt16(usart_buffer_stm, throInput, &crc);
	sendFloat(usart_buffer_stm, groundDistConf, &crc);
	
	// at last send the crc, ends the transmission
	sendChar(usart_buffer_stm, crc, &crc);
}


/* -------------------------------------------------------------------- */
/*	Send request for reset the throttle kalman states					*/
/* -------------------------------------------------------------------- */
void stmResetThrottleKalman(float initThrottle) {
	
	char crc = 0;
	
	sendChar(usart_buffer_stm, 'a', &crc);		// this character initiates the transmission
	sendChar(usart_buffer_stm, 1 + 2*4, &crc);		// this will be the size of the message
	
	sendChar(usart_buffer_stm, '5', &crc);		// id of the message
	
	sendFloat(usart_buffer_stm, initThrottle, &crc);

	// at last send the crc, ends the transmission
	sendChar(usart_buffer_stm, crc, &crc);
}

/* -------------------------------------------------------------------- */
/*	Override the current kalman's integrated altitude					*/
/* -------------------------------------------------------------------- */
void stmSetThrottleKalmanPosition(float throttlePos) {
	
	char crc = 0;
	
	sendChar(usart_buffer_stm, 'a', &crc);		// this character initiates the transmission
	sendChar(usart_buffer_stm, 1 + 2*4, &crc);		// this will be the size of the message
	
	sendChar(usart_buffer_stm, '6', &crc);		// id of the message
	
	sendFloat(usart_buffer_stm, throttlePos, &crc);

	// at last send the crc, ends the transmission
	sendChar(usart_buffer_stm, crc, &crc);
}

/* -------------------------------------------------------------------- */
/*	Send point setpoint to STM											*/
/* -------------------------------------------------------------------- */
void stmSendSetpoint(float elevatorSetpoint, float aileronSetpoint) {
	
	char crc = 0;
	
	sendChar(usart_buffer_stm, 'a', &crc);		// this character initiates the transmission
	
	sendChar(usart_buffer_stm, 1 + 2*4, &crc);		// this will be the size of the message
	sendChar(usart_buffer_stm, 's', &crc);		// id of the message
	
	// sends the payload
	sendFloat(usart_buffer_stm, elevatorSetpoint, &crc);
	sendFloat(usart_buffer_stm, aileronSetpoint, &crc);
	
	// at last send the crc, ends the transmission
	sendChar(usart_buffer_stm, crc, &crc);
}

/* -------------------------------------------------------------------- */
/*	Send dual setpoints to STM (start and end of horizon)				*/
/* -------------------------------------------------------------------- */
void stmSendTrajectory(float elevatorTrajectory[5], float aileronTrajectory[5]) {
	
	char crc = 0;
	
	sendChar(usart_buffer_stm, 'a', &crc);		// this character initiates the transmission
	
	sendChar(usart_buffer_stm, 1 + 5*4 + 5*4, &crc);	// this will be the size of the message
	sendChar(usart_buffer_stm, 't', &crc);		// id of the message
	
	int i;
	
	// send the elevator trajectory
	for (i = 0; i < 5; i++) {
		
		sendFloat(usart_buffer_stm, elevatorTrajectory[i], &crc);	
	}
	
	// send the aileron trajectory
	for (i = 0; i < 5; i++) {
		
		sendFloat(usart_buffer_stm, aileronTrajectory[i], &crc);
	}
	
	// at last send the crc, ends the transmission
	sendChar(usart_buffer_stm, crc, &crc);	
}
