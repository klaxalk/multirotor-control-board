/*
 * communication.h
 *
 * Created: 11.9.2014 13:22:37
 *  Author: Tomas Baca
 */ 

#ifndef _COMMUNICATION_H
#define _COMMUNICATION_H

#include "mavlink/common/mavlink.h"
#include "system.h"

/* -------------------------------------------------------------------- */
/*	px4flow support														*/
/* -------------------------------------------------------------------- */

mavlink_message_t mavlinkMessage;
mavlink_status_t mavlinkStatus;

mavlink_optical_flow_t opticalFlowData;
volatile float groundDistance;
volatile float elevatorSpeed;
volatile float aileronSpeed;
volatile uint8_t px4Confidence;

extern int8_t opticalFlowDataFlag;

int8_t px4flowParseChar(uint8_t incomingChar);

/* -------------------------------------------------------------------- */
/*	functions for sending and parsing raw variables to uart				*/
/* -------------------------------------------------------------------- */

// functions for parsing a variable from a buffer
float readFloat(char * message, int * indexFrom);
int16_t readInt16(char * message, int * indexFrom);
char readChar(char * message, int * indexFrom);

// methods for sending variables in a binary form
void sendFloat(UsartBuffer * usartBuffer, const float var, char * crc);
void sendInt16(UsartBuffer * usartBuffer, const int16_t var, char * crc);
void sendChar(UsartBuffer * usartBuffer, const char var, char * crc);

#endif // COMMUNICATION_H
