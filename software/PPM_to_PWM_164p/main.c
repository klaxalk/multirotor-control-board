/*
 * Multirotor Control Board
 * Revision 1.0 created by Tomáš Báča, bacatoma@fel.cvut.cz, klaxalk@gmail.com
 *
 * This version is made for 8bit ATmega with 16bit and 8bit timers. It's
 * set for 18.432 Mhz. When it is used on a different clock, you need to
 * readjust some constants in config.h.
 *
 * Knowledge need for understanding this piece of software (from the hardware point of view):
 * - to know 50Hz PWM modulation for RC servo control
 * - to know 50Hz PPM modulation (erlier used by RC transmitters), in this
 * 	 case with positive constant-length pulses
 * - be familiar with ATmega164p timers and interrupts
 *
 */

#include <stdio.h> // sprintf
#include <stdlib.h> // abs
#include <avr/interrupt.h>
#include "config.h"

#define NUMBER_OF_CHANNELS	9

// state variable for incoming pulses
volatile uint16_t pulseStart[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
volatile uint16_t pulseEnd[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
volatile uint8_t pulseFlag[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

// values received from the RC
volatile uint16_t RCchannel[9] = {PULSE_MIN, PULSE_MIDDLE, PULSE_MIDDLE, PULSE_MIDDLE, PULSE_MIN, PULSE_MIN, PULSE_MIN, PULSE_MIN, PULSE_MIN};

// values to transmite to Flight-CTRL
volatile uint16_t outputChannels[9] = {PULSE_OUT_MIN, PULSE_OUT_MIN, PULSE_OUT_MIN, PULSE_OUT_MIN, PULSE_OUT_MIN, PULSE_OUT_MIN, PULSE_OUT_MIN, PULSE_OUT_MIN, PULSE_OUT_MIN};

// port mask of PC inputs
volatile uint8_t portMask = 0;

// port mask for PA inputs
volatile uint8_t portMask2 = 0;

// preserves the current out-transmitting channel number
volatile uint8_t currentChannelOut = 0;

// temporary var for current time preservation
volatile uint16_t currentTime = 0;

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

	led_R_off();
	led_Y_off();
	led_control_off();

	// turn all interupts on
	sei();
}

// PWM input capture
void capturePWMInput() {

	int8_t j = 0;
	uint32_t pulseLength = 0;

	// iterate through all channel possible inputs
	for (j=0; j < 9; j++) {

		// if the state of channel has changed
		if (pulseFlag[j]) {

			// count the length of the pulse
			if (pulseStart[j] > pulseEnd[j]) {

				pulseLength = pulseEnd[j];
				pulseLength += 65535;
				pulseLength -= pulseStart[j];
			} else {

				pulseLength = pulseEnd[j] - pulseStart[j];
			}

			// if it's in defined boundaries
			if ((pulseLength >= PULSE_MIN) && (pulseLength <= PULSE_MAX)) {

				// set the length
				RCchannel[j] = pulseLength;
				led_R_toggle();
			}

			// clear the flag
			pulseFlag[j] = 0;
		}
	}
}

// merge RC channels with controller output
// the most important function, do not modify
// unless you know what you are doing!
void mergeSignalsToOutput() {

	int i;
	for (i = 0; i < 4; i++) {
		
		outputChannels[i] = RCchannel[3-i]/2;
	}
	for (i = 4; i < NUMBER_OF_CHANNELS; i++) {
		
		outputChannels[i] = RCchannel[i]/2;
	}
}

int main() {

	// initialize the MCU (timers, uarts and so on)
	initializeMCU();

	// the main while cycle
	while (1) {
		
		capturePWMInput();
		mergeSignalsToOutput();
	}

	return 0;
}

//~ --------------------------------------------------------------------
//~ Generation of PPM output
//~ --------------------------------------------------------------------

// fires interrupt on 16bit timer (starts the new PPM pulse)
ISR(TIMER1_COMPA_vect) {

	currentTime = TCNT1;

	// startes the output PPM pulse
	pulse1_on();

	if (currentChannelOut < NUMBER_OF_CHANNELS) {
		OCR1A = currentTime + outputChannels[currentChannelOut];
		currentChannelOut++;
		led_Y_toggle();

	} else {

		int i = 0;
		int outputSum = 0;
		
		for (i = 0; i < NUMBER_OF_CHANNELS; i++) {
			outputSum += outputChannels[i];
		}

		// if the next space is the sync space, calculates it's length
		OCR1A = currentTime + PPM_FRAME_LENGTH - outputSum;
		currentChannelOut = 0;
	}

	// the next pulse shutdown
	OCR1B = currentTime+PPM_PULSE;
}

// fires interrupt on 16bit timer (the earlier clearing interrupt)
ISR(TIMER1_COMPB_vect) {

	// shut down the output PPM pulse
	pulse1_off();
}

//~ --------------------------------------------------------------------
//~ Detection of PWM inputs
//~ --------------------------------------------------------------------

// fires interrupt on state change of PINA PWM_IN pins
ISR(PCINT0_vect) {

	currentTime = TCNT1;
	int i = 0;

	// walks through all 6 channel inputs
	for (i=0; i<4; i++) {

		// i-th port has changed
		if ((PINA ^ portMask) & _BV(i)) {

			// i-th port is now HIGH
			if (PINA & _BV(i)) {

				pulseStart[i] = currentTime;
			} else { // i-th port is no LOW

				pulseEnd[i] = currentTime;
				pulseFlag[i] = 1;
			}
		}
	}

	// saves the current A port mask
	portMask = PINA;
}

// fires interrupt on state change of PINB PWM_IN pins
ISR(PCINT1_vect) {

	currentTime = TCNT1;
	int i = 0;

	// walks through all 5 channel inputs
	for (i=0; i<5; i++) {

		// i-th port has changed
		if ((PINB ^ portMask2) & _BV(i)) {

			// i-th port is now HIGH
			if (PINB & _BV(i)) {

				pulseStart[i+4] = currentTime;
			} else { // i-th port is no LOW

				pulseEnd[i+4] = currentTime;
				pulseFlag[i+4] = 1;
			}
		}
	}

	// saves the current A port mask
	portMask2 = PINB;
}

//~ --------------------------------------------------------------------
//~ Timer for controller execution etc.
//~ --------------------------------------------------------------------

// fires onterrupt on 8bit timer overflow (aprox 70x in second)
ISR(TIMER0_OVF_vect) {

}

