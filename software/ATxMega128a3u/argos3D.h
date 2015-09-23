/*
 * argos3D.h
 *
 * Created: 24.8.2015 16:59:32
 *  Author: klaxalk
 */ 


#ifndef ARGOS3D_H_
#define ARGOS3D_H_

#define MAX_NUMBER_OF_OBSTACLES	10

#define ARGOS_BUFFER_SIZE 60

// this structure holds a message received from STM
typedef struct {

	char messageId;
	char * messageBuffer;
} argosMessageHandler_t;

int8_t argosParseChar(char inChar, argosMessageHandler_t * messageHandler);

#endif /* ARGOS3D_H_ */