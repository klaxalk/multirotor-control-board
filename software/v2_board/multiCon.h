/*
 * multiCon.h
 *
 * Created: 24.8.2015 18:53:00
 *  Author: klaxalk
 */ 


#ifndef MULTICON_H_
#define MULTICON_H_

#define MULTICON_BUFFER_SIZE 80

#define NUMBER_OF_BLOBS 4

uint8_t numberOfDetectedBlobs;
uint8_t multiconErrorState;

// this structure holds a message received from STM
typedef struct {

	char messageId;
	char * messageBuffer;
} multiconMessageHandler_t;

// this structure holds a message received from STM
typedef struct {

	float x, y, z;
} blob_t;

#ifdef MATOUS

#define NUMBER_OF_RADIOS 3

// this structure holds a message received from STM
typedef struct {

	float RSSI;
	uint16_t timer;
} btradio_t;

#endif

blob_t blobs[NUMBER_OF_BLOBS];

#ifdef MATOUS

btradio_t radios[NUMBER_OF_RADIOS];

#endif

int8_t multiconParseChar(char inChar, multiconMessageHandler_t * messageHandler);
void sendBlobs(uint64_t address);

#endif /* MULTICON_H_ */