/*
 * gumstix.h
 *
 *  Author: klaxalk
 */ 

#ifndef GUMSTIX_H_
#define GUMSTIX_H_

volatile float gumstix_x;
volatile float gumstix_y;
volatile float gumstix_z;
volatile char gumstix_v;
volatile char gumstix_ok;

#define GUMSTIX_BUFFER_SIZE 24

// this structure holds a message received from RPI
typedef struct {

	char messageId;
	char * messageBuffer;
	
} gumstixMessageHandler_t;

int8_t gumstixParseChar(char inChar, gumstixMessageHandler_t * messageHandler);

#endif /* GUMSTIX_H_ */