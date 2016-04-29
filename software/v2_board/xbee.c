/*
 * xbee.c
 *
 * Created: 8.1.2016 10:42:24
 *  Author: Tomas Baca
 */ 

#include "xbee.h"

xbeeReceiverStateMachine_t xbeeReceiver;

void xbeeSendFloat(UsartBuffer * usartBuffer, const float var, uint8_t * checkSum) {
	
	char * ukazatel = (char*) &var;
	
	usartBufferPutByte(usartBuffer, *(ukazatel+3), XBEE_SERIAL_TICKS_TIMEOUT);
	*checkSum -= *(ukazatel+3);
	
	usartBufferPutByte(usartBuffer, *(ukazatel+2), XBEE_SERIAL_TICKS_TIMEOUT);
	*checkSum -= *(ukazatel+2);
	
	usartBufferPutByte(usartBuffer, *(ukazatel+1), XBEE_SERIAL_TICKS_TIMEOUT);
	*checkSum -= *(ukazatel+1);
	
	usartBufferPutByte(usartBuffer, *(ukazatel), XBEE_SERIAL_TICKS_TIMEOUT);
	*checkSum -= *(ukazatel);
}

void xbeeSendInt16(UsartBuffer * usartBuffer, const int16_t var, uint8_t * checkSum) {
	
	char * ukazatel = (char*) &var;
	
	usartBufferPutByte(usartBuffer, *(ukazatel+1), XBEE_SERIAL_TICKS_TIMEOUT);
	*checkSum -= *(ukazatel+1);
	
	usartBufferPutByte(usartBuffer, *(ukazatel), XBEE_SERIAL_TICKS_TIMEOUT);
	*checkSum -= *(ukazatel);
}

void xbeeSendUint16(UsartBuffer * usartBuffer, const uint16_t var, uint8_t * checkSum) {
	
	char * ukazatel = (char*) &var;
	
	usartBufferPutByte(usartBuffer, *(ukazatel+1), XBEE_SERIAL_TICKS_TIMEOUT);
	*checkSum -= *(ukazatel+1);
	
	usartBufferPutByte(usartBuffer, *(ukazatel), XBEE_SERIAL_TICKS_TIMEOUT);
	*checkSum -= *(ukazatel);
}

void xbeeSendChar(UsartBuffer * usartBuffer, const char var, uint8_t * checkSum) {
	
	usartBufferPutByte(usartBuffer, var, XBEE_SERIAL_TICKS_TIMEOUT);
	*checkSum -= var;
}

void xbeeSendUint8(UsartBuffer * usartBuffer, const uint8_t var, uint8_t * checkSum) {
	
	usartBufferPutByte(usartBuffer, var, XBEE_SERIAL_TICKS_TIMEOUT);
	*checkSum -= var;
}

// iterates through the data blob and sends it byte by byte
void xbeeSendDataBlob(uint8_t * pointer, uint8_t length, uint8_t * checkSum) {
	
	uint8_t i;
	
	// just precautions
	if (length < 63) {
		
		for (i = length; i > 0; i--) {
			
			usartBufferPutByte(usart_buffer_xbee, *(pointer + i - 1), XBEE_SERIAL_TICKS_TIMEOUT);
			*checkSum -= *(pointer + i - 1);
		}
	}
}

// iterates through the data blob and sends it byte by byte
uint64_t xbeeReadUint64(const uint8_t * pointer) {
	
	uint8_t i;
	uint64_t output;
	uint8_t * tempPtr = (uint8_t *) &output;
	
	// swaps the bytes
	for (i = 0; i < 8; i++) {
		
		*(tempPtr + 7 - i) = *(pointer + i);
	}
	
	return output;
}


// iterates through the data blob and sends it byte by byte
uint16_t xbeeReadUint16(const uint8_t * pointer) {
	
	uint64_t output;
	uint8_t * tempPtr = (uint8_t *) &output;
	
	*(tempPtr) = *(pointer+1);
	*(tempPtr+1) = *(pointer);
	
	return output;
}

/* -------------------------------------------------------------------- */
/*	Parsing state machine												*/
/* -------------------------------------------------------------------- */

void sendHex(uint8_t zn) {
	
	// creates the first hex character
	uint8_t ch = zn;
	ch = ch & 240;
	ch = ch >> 4;
	if (ch >= 0 && ch <= 9)
	ch = ch + '0';
	else
	ch = ch + 'A' - 10;
	usartBufferPutByte(usart_buffer_4, ch, 100);

	// creates the second hex character
	ch = zn;
	ch = ch & 15;
	if (ch >= 0 && ch <= 9)
	ch = ch + '0';
	else
	ch = ch + 'A' - 10;
	usartBufferPutByte(usart_buffer_4, ch, 100);
	usartBufferPutByte(usart_buffer_4, ' ', 100);
}

uint8_t xbeeParseChar(uint8_t inChar, xbeeMessageHandler_t * messageHandler, xbeeReceiverStateMachine_t * receiver) {

	uint8_t * tempPtr;

	switch (receiver->receiverState) {
		
		// waiting for the first byte of the message
		case XBEE_NOT_RECEIVING:
	
			// we received the starting byte
			if (inChar == XBEE_INIT_BYTE) {

				receiver->bytesReceived = 0;
				receiver->checksum = 0xFF;
				receiver->receiverState = XBEE_RECEIVING_PAYLOAD_SIZE_1;
			}
		break;
		
		// first byte of the payload size
		case XBEE_RECEIVING_PAYLOAD_SIZE_1:
		
			tempPtr = (uint8_t *) &(receiver->payloadSize);
			*(tempPtr+1) = inChar;
			receiver->receiverState = XBEE_RECEIVING_PAYLOAD_SIZE_2;
			receiver->checksum -= inChar;
		break;
		
		// following byte should contain the payload size
		case XBEE_RECEIVING_PAYLOAD_SIZE_2:
		
			tempPtr = (uint8_t *) &(receiver->payloadSize);
			*(tempPtr) = inChar;
	
			// does the message fit into our buffer?
			if (receiver->payloadSize <= (uint8_t) XBEE_BUFFER_SIZE) {
			
				receiver->receiverState = XBEE_RECEIVING_API_TYPE;
				receiver->checksum = 0xFF;
			
			} else {
			
				// restart the state machine
				receiver->receiverState = XBEE_NOT_RECEIVING;
			}
		break;
		
		// API type (0x90 for receive packet)
		case XBEE_RECEIVING_API_TYPE:
		
			receiver->apiId = inChar;
			
			switch (receiver->apiId) {
				
				case XBEE_API_PACKET_RECEIVE:
					receiver->receiverState = XBEE_RECEIVING_PAYLOAD;
					receiver->checksum -= inChar;
				break;
				
				case XBEE_API_PACKET_AT_RESPONSE:
					receiver->receiverState = XBEE_RECEIVING_PAYLOAD;
					receiver->checksum -= inChar;
				break;
				
				default:
					// restart the state machine
					receiver->receiverState = XBEE_NOT_RECEIVING;
				break;
				
			}
		
		break;
	
		// the new byte is the payload byte
		case XBEE_RECEIVING_PAYLOAD:
		
			receiver->rxBuffer[receiver->bytesReceived++] = inChar;
			receiver->checksum -= inChar;
		
			// the payload is successfully received
			if (receiver->bytesReceived >= (receiver->payloadSize-1)) {
			
				receiver->receiverState = XBEE_RECEIVING_CHECKSUM;
			}
		break;
	
		// receiving the last checksum byte
		case XBEE_RECEIVING_CHECKSUM:
		
			// if the checksum is correct
			if (receiver->checksum == inChar) {
				
				switch (receiver->apiId) {
					
					case XBEE_API_PACKET_RECEIVE:
					
						messageHandler->apiId = receiver->apiId;
						messageHandler->content.receiveResponse.address64 = xbeeReadUint64((uint8_t *) &receiver->rxBuffer);
						messageHandler->content.receiveResponse.address16 = xbeeReadUint16((uint8_t *) &receiver->rxBuffer + 8);
						messageHandler->content.receiveResponse.receiveOptions = xbeeReadUint16((uint8_t *) &receiver->rxBuffer + 9);
						messageHandler->content.receiveResponse.messageId = (uint8_t) *(receiver->rxBuffer + 11);
						messageHandler->content.receiveResponse.payload = (uint8_t *) (&receiver->rxBuffer) + 12;
						messageHandler->content.receiveResponse.payloadSize = receiver->payloadSize - 13;
						receiver->receiverState = XBEE_MESSAGE_RECEIVED;
						
					break;
					
					case XBEE_API_PACKET_AT_RESPONSE:
						
						messageHandler->apiId = receiver->apiId;
						messageHandler->content.atReponse.frameID = receiver->rxBuffer[0];
						messageHandler->content.atReponse.ATcmd[0] = receiver->rxBuffer[1];
						messageHandler->content.atReponse.ATcmd[1] = receiver->rxBuffer[2];
						messageHandler->content.atReponse.cmdStatus = receiver->rxBuffer[3];
						messageHandler->content.atReponse.cmdData = receiver->rxBuffer[4];
						receiver->receiverState = XBEE_MESSAGE_RECEIVED;
						
					break;
					
					case XBEE_MESSAGE_RECEIVED:
					break;
					
				}
				
				return 1;

			// if it is not correct, restart the receiver
			} else {
			
				receiver->receiverState = XBEE_NOT_RECEIVING;
			}
		break;
		
		case XBEE_MESSAGE_RECEIVED:
		break;
	}
		
	return 0;
}

void xbeeSendMessageTo(const uint8_t * message, const uint16_t length, const uint64_t address) {
	
	uint16_t i;
	
	uint8_t checkSum = 0;
	
	// initiates the message
	xbeeSendUint8(usart_buffer_xbee, XBEE_INIT_BYTE, &checkSum);
	
	// send the packet length, 14 is the overhead of the xbee transmit packet
	xbeeSendUint16(usart_buffer_xbee, length + 14, &checkSum);
	
	checkSum = 0xFF;
	
	// send the api type
	xbeeSendUint8(usart_buffer_xbee, XBEE_API_PACKET_TRANSMIT, &checkSum);
	
	// send the frameID
	xbeeSendUint8(usart_buffer_xbee, 0x00, &checkSum);
	
	// send the 64bit address
	xbeeSendDataBlob((uint8_t *) &address, 8, &checkSum);
	
	// send the 16bit address
	xbeeSendUint16(usart_buffer_xbee, XBEE_16BIT_ADDRESS_UNKNOWN, &checkSum);
	
	// send the radius
	xbeeSendUint8(usart_buffer_xbee, XBEE_RADIUS_MAX, &checkSum);
	
	// send transmit options
	xbeeSendUint8(usart_buffer_xbee, 0, &checkSum);
	
	// send the payload
	for (i = 0; i < length; i++) {
		
		xbeeSendUint8(usart_buffer_xbee, *(message+i), &checkSum);
	}
	
	// send the checksum
	xbeeSendUint8(usart_buffer_xbee, checkSum, &checkSum);
}

void xbeeGetRSSI(void) {
	
	uint8_t checkSum = 0;
	
	// initiates the message
	xbeeSendUint8(usart_buffer_xbee, XBEE_INIT_BYTE, &checkSum);
	
	// send the packet length, 8 for RSSI AT command
	xbeeSendUint16(usart_buffer_xbee, 4, &checkSum);
	
	checkSum = 0xFF;

	// send the api type
	xbeeSendUint8(usart_buffer_xbee, XBEE_API_AT_COMMAND, &checkSum);	
	
	// send the frameID, for EG 0x09
	xbeeSendUint8(usart_buffer_xbee, 0x09, &checkSum);
	
	// what AT command? DB, two ascii characters
	xbeeSendUint8(usart_buffer_xbee, 'D', &checkSum);
	xbeeSendUint8(usart_buffer_xbee, 'B', &checkSum);
	
	// send the checksum
	xbeeSendUint8(usart_buffer_xbee, checkSum, &checkSum);
}