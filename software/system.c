/*
 * This file contains functions for control and configuration of the system
 */

#include "system.h"

// disable controllers
void disableController() {

	controllerEnabled = 0;
}

// enable controllers
void enableController() {

	if (controllerEnabled == 0) {
		throttleIntegration = 0;

#if GUMSTIX_DATA_RECEIVE == ENABLED

		gumstixElevatorIntegral = 0;
		gumstixAileronIntegral = 0;

#endif // GUMSTIX_DATA_RECEIVE == ENABLED

#if PX4FLOW_DATA_RECEIVE == ENABLED

		elevatorSpeedIntegration = 0;
		aileronSpeedIntegration = 0;

#endif
	}
	controllerEnabled = 1;
}

void disablePositionController() {

	positionControllerEnabled = 0;
}

void enablePositionController() {

	if (positionControllerEnabled == 0) {
		// set integrated variables to default
	}
	positionControllerEnabled = 1;
}

// disarm the quadcopter
// transmit controll sequence to the controll board
// throttle -> min
// rudder -> left
void disarmVehicle() {

	disarmingToggled = 1;
	outputChannels[0] = PULSE_MIN;
	outputChannels[1] = PULSE_MAX;
	vehicleArmed = 0;
}

// arm the quadcopter
// transmit control sequece to the control board
// throttle -> min
// rudder -> right
void armVehicle() {

	if (vehicleArmed == 0) {

		outputChannels[0] = PULSE_MAX;
		outputChannels[1] = PULSE_MIN;
		armingToggled = 1;
		vehicleArmed = 1;
	}
}

// check if the button1 was pressed
int8_t button1check() {

	if (!(PIND & (1 << PD7))) {

		button1pressedMap = (button1pressedMap << 1)|1;
		button1pressed = 1;
	} else {
		button1pressedMap = (button1pressedMap << 1);
	}

	if ((buttonChangeEnable == 1) && (button1pressed == 1) && (button1pressedMap == 0)) {

		button1pressed = 0;
		buttonChangeEnable = 0;

		return 1;
	} else {
		return 0;
	}
}

// check if the button2 was pressed
int8_t button2check() {
	if (!(PIND & (1 << PD6))) {

		button2pressedMap = (button2pressedMap << 1)|1;
		button2pressed = 1;
	} else {
		button2pressedMap = (button2pressedMap << 1);
	}

	if ((buttonChangeEnable == 1) && (button2pressed == 1) && (button2pressedMap == 0)) {

		button2pressed = 0;
		buttonChangeEnable = 0;

		return 1;
	} else {
		return 0;
	}
}

// set the parameters of the MCU
void initializeMCU() {

	// stop all interrupts
	cli();

	// set port PC6 as green LED and PC7 as red LED
	DDRC |= _BV(PC6)|_BV(PC7);

	// set port PA4 as output1 (PPM) and PA5 as output2
	DDRA |= _BV(PA4)|_BV(PA5);

	// set ports PA0 .. PA7 as inputs (channels)
	DDRA |= 0b11110000;

	// set ports PB0 .. PB4 as aux inputs
	DDRB |= 0b11100000;

	// button1 pullup
	PORTD |= (1<<PD7);

	// button2 pullup
	PORTD |= (1<<PD6);

	// set pull ups for ports PA0 .. PA7
	PORTA |= 0b11110000;

	// set pull ups for ports PB0 .. PB4
	PORTB |= 0b11100000;

	// enables 16timer
	TIMSK1 |= _BV(OCIE1A)|_BV(OCIE1B);

	// using /8 prescaler
	TCCR1B |= _BV(CS11);

	// enables 8timer
	TIMSK0 |= _BV(TOIE0);

	// using /1024 prescaler
	TCCR0B |= _BV(CS02)|_BV(CS00);

	// external interrupts
	// any change of PCINT0 - PCINT7 will fire interrupt
	EICRA |= _BV(ISC00);

	// any change of PCINT8 - PCINT15 will fire interrupt
	EICRA |= _BV(ISC10);

	// enable pin change interrupt on ports PCINT0 - PCINT7 (PA0 .. PA7)
	PCICR |= _BV(PCIE0);

	// enable pin change interrupt on ports PCINT8 - PCINT15 (PB0 - PB7)
	PCICR |= _BV(PCIE1);

	// enables pin change interrupt flag for PCINT0 - PCINT7
	PCIFR |= _BV(PCIF0);

	// enables pin change interrupt flag for PCINT8 - PCINT15
	PCIFR |= _BV(PCIF1);

	// trigger on change of PCINT1 pins PA0 .. PA7
	PCMSK0 |= 0b00001111;

	// mask for PB
	PCMSK1 |= 0b00011111;

	// copy actual PINA states into portMask
	portMask = PINA;

	// copy actual PINA states into portMask
	portMask2 = PINB;

	// serial init 0
	USART0_init(MYUBRR0);

	// serial init 1
	USART1_init(MYUBRR1);

	led_R_off();
	led_Y_off();
	led_control_off();

	// turn all interupts on
	sei();
}

#if ATOM_DATA_RECEIVE == ENABLED

// functions for pitch buffer

uint8_t bufferSucc(uint8_t ptr, const uint8_t size) {
	ptr++;
	if (ptr==size) {
		ptr=0;
	}
	return ptr;
}

uint8_t pitchBufferEmpty() {

	return (pitchBufferFirst == pitchBufferLast);
}

void pitchBufferPut(int16_t c) {
	uint8_t last = bufferSucc(pitchBufferLast, PITCH_BUFFER_SIZE);
	if (last == pitchBufferFirst) {
		return;
	}
	pitchBuffer[last] = c;
	pitchBufferNum++;
	pitchBufferLast = last;
}

int16_t pitchBufferGet() {
	if (pitchBufferEmpty()) {
		return 0;
	}
	uint8_t first = bufferSucc(pitchBufferFirst, PITCH_BUFFER_SIZE);
	int16_t result = pitchBuffer[first];
	pitchBufferFirst = first;
	pitchBufferNum--;
	return result;
}

uint8_t rollBufferEmpty() {

	return (rollBufferFirst == rollBufferLast);
}

void rollBufferPut(int16_t c) {
	uint8_t last = bufferSucc(rollBufferLast, ROLL_BUFFER_SIZE);
	if (last == rollBufferFirst) {
		return;
	}
	rollBuffer[last] = c;
	rollBufferNum++;
	rollBufferLast = last;
}

int16_t rollBufferGet() {
	if (rollBufferEmpty()) {
		return 0;
	}
	uint8_t first = bufferSucc(rollBufferFirst, ROLL_BUFFER_SIZE);
	int16_t result = rollBuffer[first];
	rollBufferFirst = first;
	rollBufferNum--;
	return result;
}

#endif
