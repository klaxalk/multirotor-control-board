/*
 * communication.h
 *
 * Created: 11.9.2014 13:22:37
 *  Author: Tomas Baca
 */ 

#ifndef _COMMUNICATION_H
#define _COMMUNICATION_H

#include "mavlink/common/mavlink.h"
#include <avr/io.h>
#include "config.h"
#include "controllers.h"

// merge RC channels with controller output
// the most important function, do not modify
// unless you know what you are doing!
void mergeSignalsToOutput();

//~ --------------------------------------------------------------------
//~ if data processing from px4flow computer is on
//~ --------------------------------------------------------------------

extern mavlink_message_t mavlinkMessage;
extern mavlink_status_t mavlinkStatus;

extern mavlink_optical_flow_t opticalFlowData;
extern volatile float groundDistance;
extern volatile float elevatorSpeed;
extern volatile float aileronSpeed;
extern volatile uint8_t px4Confidence;

extern int8_t opticalFlowDataFlag;

int8_t px4flowParseChar(uint8_t incomingChar);

// functions for parsing a variable from a buffer
float readFloat(char * message, int * indexFrom);
int16_t readInt16(char * message, int * indexFrom);
char readChar(char * message, int * indexFrom);

// methods for sending variables in a binary form
void sendFloat(UsartBuffer * usartBuffer, const float var, char * crc);
void sendInt16(UsartBuffer * usartBuffer, const int16_t var, char * crc);
void sendChar(UsartBuffer * usartBuffer, const char var, char * crc);

#endif // COMMUNICATION_H
