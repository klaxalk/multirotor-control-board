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

uint8_t hex2bin(const uint8_t * ptr);

/* -------------------------------------------------------------------- */
/*	px4flow receiver support											*/
/* -------------------------------------------------------------------- */

mavlink_message_t mavlinkMessage;
mavlink_status_t mavlinkStatus;

mavlink_optical_flow_t opticalFlowData;
volatile float groundDistance;
volatile float elevatorSpeed;
volatile float aileronSpeed;
volatile uint8_t px4Confidence;

extern int8_t opticalFlowDataFlag;

uint8_t readUint8(char * message, int * indexFrom);
void writeFloatToBuffer(char * buffer, const float input, uint16_t position);

int8_t px4flowParseChar(uint8_t incomingChar);
uint8_t readUint8(char * message, int * indexFrom);
void writeUint64ToBuffer(char * buffer, const uint64_t input, uint16_t position);

#endif // COMMUNICATION_H
