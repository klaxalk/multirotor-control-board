/*
 * This file contains functions and variables that take care of communication
 */

#ifndef _COMMUNICATION_H
#define _COMMUNICATION_H

#include "mavlink/v1.0/common/mavlink.h"
#include <avr/io.h>
#include "config.h"
#include "controllers.h"

// define basic macros
#define pulse1_on()  PORTA |= _BV(4) // set output1 to 1
#define pulse1_off()  PORTA &= ~_BV(4) // set output1 to 0

// define mavlink buffers lengths
#define MAX_SENDE_BUFF 20
#define MAX_EMPFANGS_BUFF 20

// defining UART speeds
#define FOSC 18432000UL
#define MYUBRR0 FOSC/16/BAUD0-1
#define MYUBRR1 FOSC/16/BAUD1-1

extern volatile unsigned char controllerEnabled;
extern volatile uint16_t outputChannels[6];
extern volatile uint8_t pulseFlag[9];
extern volatile uint16_t pulseStart[9];
extern volatile uint16_t pulseEnd[9];

// controllers output variables
extern volatile int16_t controllerElevatorOutput;
extern volatile int16_t controllerAileronOutput;
extern volatile int16_t controllerThrottleOutput;
extern volatile int16_t controllerRudderOutput;

// variables for mavlink
extern unsigned volatile char BytesReceiving;
extern signed volatile char TxdBuffer[MAX_SENDE_BUFF];
extern signed volatile char RxdBuffer[MAX_EMPFANGS_BUFF];
extern signed char *pRxData;
extern unsigned char RxDataLen;

// angles with respect to the board
extern volatile int16_t pitchBoardAngle;
extern volatile int16_t rollBoardAngle;
// angles with respect to the "front"
extern volatile int16_t pitchAngle;
extern volatile int16_t rollAngle;

// variables for PPM and PWM communication
extern volatile uint16_t RCchannel[9];

// initialize the serial line 0
void USART0_init(unsigned int ubrr);

// initialize the serial line 1
void USART1_init(unsigned int ubrr);

// write byte to Uart0 (debug) serial output
void Uart0_write_char(unsigned char c);

// write string to Uart0 (debug) serial output
void Uart0_write_string(char* array, int len);

#if FLIGHTCTRL_DATA_RECEIVE == ENABLED

// decode the base64 encoded data from the Flight-CTRL
void Decode64();

#endif

#if PX4FLOW_DATA_RECEIVE == ENABLED

// decode the Mavlink incomming message from the px4flow sensor
uint8_t my_mavlink_parse_char(uint8_t chan, uint8_t c, mavlink_message_t* r_message, mavlink_status_t* r_mavlink_status);

#endif // PX4FLOW_DATA_RECEIVE == ENABLED

// merge RC channels with controller output
// the most important function, do not modify
// unless you know what you are doing!
void mergeSignalsToOutput();

// parse the message from the FlightCtrl (angles)
void parseFlightCtrlMessage();

// PWM input capture
void capturePWMInput();

//~ --------------------------------------------------------------------
//~ if data processing from Atom computer is on
//~ --------------------------------------------------------------------

#if ATOM_DATA_RECEIVE == ENABLED

extern unsigned volatile char atomParseCharState;
extern unsigned volatile char atomParseCharByte;
extern volatile int16_t atomParseTempInt;
extern volatile int16_t xPosSurf;
extern volatile int16_t yPosSurf;
extern volatile int16_t headingSurf;
extern volatile int16_t scaleSurf;

extern unsigned volatile char Uart0state;
extern unsigned volatile char Uart0byte;

// processes message from Atom computer
void atomParseChar(char incomingChar);

#endif // ATOM_DATA_RECEIVE == ENABLED

//~ --------------------------------------------------------------------
//~ if data processing from px4flow computer is on
//~ --------------------------------------------------------------------

#if PX4FLOW_DATA_RECEIVE == ENABLED

extern mavlink_message_t mavlinkMessage;
extern mavlink_status_t mavlinkStatus;
extern int8_t opticalFlowDataFlag;

int8_t px4flowParseChar(uint8_t incomingChar);

#endif // PX4FLOW_DATA_RECEIVE == ENABLED

//~ --------------------------------------------------------------------
//~ process data from flight ctrl
//~ --------------------------------------------------------------------

#if (FLIGHTCTRL_DATA_RECEIVE == ENABLED) || (ATOM_DATA_RECEIVE == ENABLED)

extern unsigned volatile char flightCtrlDataFlag;
extern unsigned volatile char CntCrcError;
extern unsigned volatile char BytesReceiving;
extern signed volatile char TxdBuffer[MAX_SENDE_BUFF];
extern signed volatile char RxdBuffer[MAX_EMPFANGS_BUFF];
extern unsigned volatile char transfereUart1done;
extern unsigned char interval;

extern signed char *pRxData;
extern unsigned char RxDataLen;
extern int8_t volatile flightCtrlDataBeingReceived;
extern int8_t volatile flightCtrlByteReceive;

void flightCtrlParseChar(char incomingChar);
void Decode64(void);

#endif

//~ --------------------------------------------------------------------
//~ if data processing from gumstix is on
//~ --------------------------------------------------------------------

#if GUMSTIX_DATA_RECEIVE == ENABLED

extern volatile unsigned char gumstixParseCharState;
extern volatile unsigned char gumstixParseCharByte;
extern volatile unsigned char gumstixParseCharCrc;
extern volatile int16_t gumstixParseTempInt;
extern volatile int16_t xPosGumstixNew;
extern volatile int16_t yPosGumstixNew;
extern volatile int16_t zPosGumstixNew;
extern volatile int8_t validGumstix;
extern volatile int8_t gumstixDataFlag;

void gumstixParseChar(unsigned char incomingChar);

#endif // GUMSTIX_DATA_RECEIVE == ENABLED

#endif // COMMUNICATION_H
