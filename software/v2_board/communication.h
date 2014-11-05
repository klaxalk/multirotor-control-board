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

#if PX4FLOW_DATA_RECEIVE == ENABLED

extern mavlink_message_t mavlinkMessage;
extern mavlink_status_t mavlinkStatus;

extern mavlink_optical_flow_t opticalFlowData;
extern volatile float groundDistance;
extern volatile float elevatorSpeed;
extern volatile float aileronSpeed;
extern volatile uint8_t px4Confidence;

extern int8_t opticalFlowDataFlag;

int8_t px4flowParseChar(uint8_t incomingChar);

#endif // PX4FLOW_DATA_RECEIVE == ENABLED

//~ --------------------------------------------------------------------
//~ if data processing from gumstix is on
//~ --------------------------------------------------------------------

#if GUMSTIX_DATA_RECEIVE == ENABLED

//extern volatile unsigned char gumstixParseCharState;
//extern volatile unsigned char gumstixParseCharByte;
//extern volatile unsigned char gumstixParseCharCrc;
//extern volatile int16_t gumstixParseTempInt;
//extern volatile int8_t validGumstix;
extern volatile int8_t gumstixDataFlag;

extern volatile int16_t xPosGumstixNew;
extern volatile int16_t yPosGumstixNew;
extern volatile int16_t zPosGumstixNew;

extern volatile float elevatorGumstix;
extern volatile float aileronGumstix;
extern volatile float throttleGumstix;
extern volatile int8_t validGumstix;

void gumstixParseChar(unsigned char incomingChar);

#endif // GUMSTIX_DATA_RECEIVE == ENABLED

#endif // COMMUNICATION_H
