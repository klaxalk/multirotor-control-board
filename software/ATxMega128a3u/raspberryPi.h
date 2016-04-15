/*
 * raspberryPi.h
 *
 * Created: 8.10.2015 16:31:38
 *  Author: klaxalk
 */ 


#ifndef RASPBERRYPI_H_
#define RASPBERRYPI_H_

volatile float rpix;
volatile float rpiy;
volatile float rpiz;
volatile char rpiOk;

#define RPI_BUFFER_SIZE 24

// this structure holds a message received from RPI
typedef struct {

	char messageId;
	char * messageBuffer;
} rpiMessageHandler_t;

int8_t rpiParseChar(char inChar, rpiMessageHandler_t * messageHandler);

void sendPiBlob(uint64_t address);

#endif /* RASPBERRYPI_H_ */