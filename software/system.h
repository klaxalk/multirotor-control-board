/*
 * This file contains functions for control and configuration of the system
 */

#ifndef _SYSTEM_H
#define _SYSTEM_H

#include <avr/interrupt.h>
#include "communication.h"

// declaration of global variables
extern volatile uint8_t portMask;
extern volatile uint8_t portMask2;

extern volatile float throttleIntegration;
extern volatile unsigned char positionControllerEnabled;

extern volatile uint8_t vehicleArmed;
extern volatile uint8_t armingToggled;
extern volatile uint8_t disarmingToggled;

// buttons variables
extern volatile char button1pressed;
extern volatile char button1pressedMap;
extern volatile char button2pressed;
extern volatile char button2pressedMap;
extern volatile char buttonChangeEnable;

#if PX4FLOW_DATA_RECEIVE == ENABLED

extern volatile float elevatorSpeedIntegration;
extern volatile float aileronSpeedIntegration;

#endif

void disableController();

void enableController();

void disablePositionController();

void enablePositionController();

void disarmVehicle();

void armVehicle();

void initializeMCU();

int8_t button1check();

int8_t button2check();

#if ATOM_DATA_RECEIVE == ENABLED

extern volatile int16_t pitchBuffer[];
extern volatile uint8_t pitchBufferNum;
extern volatile uint8_t pitchBufferLast;
extern volatile uint8_t pitchBufferFirst;

extern volatile int16_t rollBuffer[];
extern volatile uint8_t rollBufferNum;
extern volatile uint8_t rollBufferLast;
extern volatile uint8_t rollBufferFirst;

uint8_t bufferSucc(uint8_t ptr, const uint8_t size);

uint8_t pitchBufferEmpty();
void pitchBufferPut(int16_t c);
int16_t pitchBufferGet();

uint8_t rollBufferEmpty();
void rollBufferPut(int16_t c);
int16_t rollBufferGet();

#endif

#endif // _SYSTEM_H

