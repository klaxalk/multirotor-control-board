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

#define RPI_INIT_BYTE	0x61

typedef enum {
	
	NOT_RECEIVING,				// waiting for the first byte
	RECEIVING_PAYLOAD_SIZE,		// expecting to receive the payload size	
	RECEIVING_PAYLOAD,			// the payload is being received
	RECEIVING_CHECKSUM,			// expecting to receive the checksum byte
	MESSAGE_RECEIVED,			// signalizes that the message is complete and ready to proceess
	
} receiver_states_t;

typedef struct {
	
	// storing checksum while the message is received 
	uint8_t checksum;
	
	// state of the receiver states machine
	receiver_states_t receiverState;
	
	// the payload size of the packet
	uint8_t payloadSize;
	
	// how many bytes have been already received
	uint16_t bytesReceived;
	
	// buffer for received message
	uint8_t rxBuffer[RPI_BUFFER_SIZE];
	
} receiverStateMachine_t;

// this structure holds a message received from RPI
typedef struct {

	char messageId;
	char * messageBuffer;
	
} rpiMessageHandler_t;

receiverStateMachine_t rpiReceiver;

int8_t rpiParseChar(char inChar, rpiMessageHandler_t * messageHandler, receiverStateMachine_t * receiver);

#endif /* RASPBERRYPI_H_ */