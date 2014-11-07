/*
 * system.c
 *
 * Created: 10.9.2014 21:52:53
 *  Author: Tomas Baca
 */ 

#include "system.h"
#include "pmic_driver.h"
#include "TC_driver.h"
#include "port_driver.h"
#include "ioport.h"
#include "sysclk.h"
#include "usart_driver_RTOS.h"
#include "communication.h"

#define NUMBER_OF_CHANNELS_OUT	6

#ifndef FLIGHT_CONTROLLER
#define OUTPUT_THROTTLE 0
#define OUTPUT_RUDDER 1
#define OUTPUT_ELEVATOR 2
#define OUTPUT_AILERON 3
#define OUTPUT_AUX1 4
#define OUTPUT_AUX2 5
#define OUTPUT_AUX3 6
#define OUTPUT_CHANNEL_DEFAULTS {PULSE_OUT_MIN, PULSE_OUT_MIDDLE, PULSE_OUT_MIDDLE, PULSE_OUT_MIDDLE, PULSE_OUT_MIN, PULSE_OUT_MIN}
#endif

#if FLIGHT_CONTROLLER == NAZA
#define OUTPUT_THROTTLE 2
#define OUTPUT_RUDDER 3
#define OUTPUT_ELEVATOR 1
#define OUTPUT_AILERON 0
#define OUTPUT_AUX1 4
#define OUTPUT_AUX2 5
#define OUTPUT_AUX3 6
#undef NUMBER_OF_CHANNELS_OUT
#define NUMBER_OF_CHANNELS_OUT 7
#define OUTPUT_CHANNEL_DEFAULTS {PULSE_OUT_MIDDLE, PULSE_OUT_MIDDLE, PULSE_OUT_MIN, PULSE_OUT_MIDDLE, PULSE_OUT_MIN, PULSE_OUT_MIN, PULSE_OUT_MIN}
//#warning NAZA FLIGHT CONTROLLER
#endif

/* -------------------------------------------------------------------- */
/*	Variables for PPM input capture										*/
/* -------------------------------------------------------------------- */
uint16_t PPM_in_start = 0;
uint8_t PPM_in_current_channel = 0;
volatile uint16_t RCchannel[9] = {PPM_IN_MIN_LENGTH, PPM_IN_MIDDLE_LENGTH, PPM_IN_MIDDLE_LENGTH, PPM_IN_MIDDLE_LENGTH, PPM_IN_MIN_LENGTH, PPM_IN_MIN_LENGTH, PPM_IN_MIN_LENGTH, PPM_IN_MIN_LENGTH, PPM_IN_MIN_LENGTH};

/* -------------------------------------------------------------------- */
/*	Variables for PPM output generation									*/
/* -------------------------------------------------------------------- */
volatile uint16_t outputChannels[NUMBER_OF_CHANNELS_OUT] = OUTPUT_CHANNEL_DEFAULTS;
volatile uint8_t currentChannelOut = 0;

/* -------------------------------------------------------------------- */
/*	Variables related to controllers									*/
/* -------------------------------------------------------------------- */
volatile unsigned char controllerEnabled;

// controllers output variables
volatile int16_t controllerElevatorOutput;
volatile int16_t controllerAileronOutput;
volatile int16_t controllerThrottleOutput;
volatile int16_t controllerRudderOutput;

/* -------------------------------------------------------------------- */
/*	Buffers for USARTs													*/
/* -------------------------------------------------------------------- */
UsartBuffer * usart_buffer_stm;
UsartBuffer * usart_buffer_xbee;
UsartBuffer * usart_buffer_log;
UsartBuffer * usart_buffer_1;
UsartBuffer * usart_buffer_2;
UsartBuffer * usart_buffer_3;
UsartBuffer * usart_buffer_4;

/* -------------------------------------------------------------------- */
/*	USART baud rates													*/
/* -------------------------------------------------------------------- */
#define USART_STM_BAUDRATE		BAUD57600
#define USART_XBEE_BAUDRATE		BAUD19200
#define USART_LOG_BAUDRATE		BAUD9600
#define USART_1_BAUDRATE		BAUDPX4FLOW
#define USART_2_BAUDRATE		BAUD19200
#define USART_3_BAUDRATE		BAUD115200
#define USART_4_BAUDRATE		BAUD57600

/* -------------------------------------------------------------------- */
/*	Basic initialization of the MCU, peripherals and i/o				*/
/* -------------------------------------------------------------------- */
void boardInit() {

	/* -------------------------------------------------------------------- */
	/*	Setup GPIO for LEDs and PPM i/o										*/
	/* -------------------------------------------------------------------- */

	ioport_init();

	ioport_set_pin_dir(RED, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(BLUE, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(ORANGE, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(GREEN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(YELLOW, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(PPM_OUT, IOPORT_DIR_OUTPUT);

	ioport_set_pin_level(RED, true);
	ioport_set_pin_level(BLUE, true);
	ioport_set_pin_level(ORANGE, true);
	ioport_set_pin_level(GREEN, true);
	ioport_set_pin_level(YELLOW, true);

	/* -------------------------------------------------------------------- */
	/*	Enable system clock for all peripheral modules						*/
	/* -------------------------------------------------------------------- */

	sysclk_init();

	sysclk_enable_module(SYSCLK_PORT_GEN, 0xff);
	sysclk_enable_module(SYSCLK_PORT_A, 0xff);
	sysclk_enable_module(SYSCLK_PORT_B, 0xff);
	sysclk_enable_module(SYSCLK_PORT_C, 0xff);
	sysclk_enable_module(SYSCLK_PORT_D, 0xff);
	sysclk_enable_module(SYSCLK_PORT_E, 0xff);
	sysclk_enable_module(SYSCLK_PORT_F, 0xff);

	/* -------------------------------------------------------------------- */
	/*	Timer TCD1 for measuring incoming PPM with RC Receiver Data			*/
	/* -------------------------------------------------------------------- */

	// select the clock source and pre-scaler by 8
	TC1_ConfigClockSource(&TCD1, TC_CLKSEL_DIV8_gc);

	/* -------------------------------------------------------------------- */
	/*	Timer TCD0 for output PPM generation								*/
	/* -------------------------------------------------------------------- */

	// select the clock source and pre-scaler by 8
	TC0_ConfigClockSource(&TCD0, TC_CLKSEL_DIV8_gc);

	// enable compare A
	TC0_EnableCCChannels(&TCD0, TC0_CCAEN_bm);

	// set the overflow interrupt
	TC0_SetOverflowIntLevel(&TCD0, TC_OVFINTLVL_LO_gc);

	// set the compare A low level interrupt
	TC0_SetCCAIntLevel(&TCD0, TC_CCAINTLVL_LO_gc);

	// set the length of PPM pulse beginning
	TC_SetCompareA(&TCD0, PPM_PULSE);

	/* -------------------------------------------------------------------- */
	/*	setup PD4 as a PPM input with interrupt								*/
	/* -------------------------------------------------------------------- */

	// self explanatory
	PORT_ConfigurePins(&PORTD, 0x10, false, false, PORT_OPC_TOTEM_gc, PORT_ISC_RISING_gc);
	PORT_SetPinsAsInput(&PORTD, 0x10);

	// configure interrupt on PD4
	PORT_ConfigureInterrupt0(&PORTD, PORT_INT0LVL_LO_gc, 0x10);

	/* -------------------------------------------------------------------- */
	/*	Initialize USARTs													*/
	/* -------------------------------------------------------------------- */
	usart_buffer_1 = usartBufferInitialize(&USART_1, USART_1_BAUDRATE, 128);
	usart_buffer_2 = usartBufferInitialize(&USART_2, USART_2_BAUDRATE, 128);
	usart_buffer_3 = usartBufferInitialize(&USART_3, USART_3_BAUDRATE, 128);
	usart_buffer_4 = usartBufferInitialize(&USART_4, USART_4_BAUDRATE, 128);
	usart_buffer_stm = usartBufferInitialize(&USART_STM, USART_STM_BAUDRATE, 128);
	usart_buffer_xbee = usartBufferInitialize(&USART_XBEE, USART_XBEE_BAUDRATE, 128);
	usart_buffer_log = usartBufferInitialize(&USART_LOG, USART_LOG_BAUDRATE, 128);

	/* -------------------------------------------------------------------- */
	/*	enable low-level interrupts											*/
	/* -------------------------------------------------------------------- */

	PMIC_EnableLowLevel();
}

/* -------------------------------------------------------------------- */
/*	Merge signals from RC Receiver with the controller outputs			*/
/* -------------------------------------------------------------------- */
void mergeSignalsToOutput() {

	int16_t outputThrottle = PULSE_OUT_MIN;
	int16_t outputElevator = PULSE_OUT_MIDDLE;
	int16_t outputAileron = PULSE_OUT_MIDDLE;
	int16_t outputRudder = PULSE_OUT_MIDDLE;

	//portENTER_CRITICAL();

	outputThrottle = RCchannel[THROTTLE];
	outputRudder = RCchannel[RUDDER];
	outputElevator = RCchannel[ELEVATOR];
	outputAileron = RCchannel[AILERON];

	if (controllerEnabled == 1) {

		led_red_on();

		outputThrottle += controllerThrottleOutput;
		outputElevator += controllerElevatorOutput;
		outputAileron += controllerAileronOutput;
		// we do not have a rudder controller, yet
		// TODO outputRudder += controllerRudderOutput; 
	} else {

		led_red_off();
	}

	// Everithing is *2 because the PPM incoming to this board is twice slower then the PPM goeing out
	outputChannels[OUTPUT_THROTTLE] = outputThrottle*2;
	outputChannels[OUTPUT_RUDDER] = outputRudder*2;
	outputChannels[OUTPUT_ELEVATOR] = outputElevator*2;
	outputChannels[OUTPUT_AILERON] = outputAileron*2;

#if FLIGHT_CONTROLLER == NAZA
	outputChannels[OUTPUT_AUX3] = RCchannel[8]*2; // IN7->U TODO: make configurable
#endif

	//portEXIT_CRITICAL();
}

/* -------------------------------------------------------------------- */
/*	Interrupt for receiving PPM with RC Receiver signals, DO NOT MODIFY!*/
/* -------------------------------------------------------------------- */
ISR(PORTD_INT0_vect) {

	// stores the current time as and END of the last PPM pulse
	uint16_t PPM_in_end = TCD1.CNT;
	uint16_t PPM_in_length = 0;

	// if the timer has not overflown
	if (PPM_in_end > PPM_in_start) {

		// computes the PPM pulse length
		PPM_in_length = PPM_in_end - PPM_in_start;

	} else { // if the time has overflown

		// computes the PPM pulse length
		PPM_in_length = 65535;
		PPM_in_length += PPM_in_end;
		PPM_in_length -= PPM_in_start;
	}

	// if the PPM is longer then threshold => synchronizing pulse
	if (PPM_in_length > PPM_IN_TRESHOLD) {

		PPM_in_current_channel = 0;

		// if it is within the boundaries of desired PPM pulse
	} else if ((PPM_in_length >= PPM_IN_MIN_LENGTH) && (PPM_in_length <= PPM_IN_MAX_LENGTH)) {

		// stores the value into the RCchannel array
		RCchannel[PPM_in_current_channel] = PPM_in_length;
		PPM_in_current_channel++;
	} else {

		PPM_in_current_channel++;
	}

	PPM_in_start = PPM_in_end;
}

/* -------------------------------------------------------------------- */
/*	Interrupt for timing the PPM output pulse, DO NOT MODIFY!			*/
/* -------------------------------------------------------------------- */
ISR(TCD0_OVF_vect) {

	// starts the output PPM pulse
	ppm_out_on();

	if (currentChannelOut < NUMBER_OF_CHANNELS_OUT) {

#ifdef MULTI_OUTPUT_PPM
		switch(currentChannelOut) {
			case 0: ppm_out1_on(); break;
			case 1: ppm_out2_on(); break;
			case 2: ppm_out3_on(); break;
			case 3: ppm_out4_on(); break;
		}
#endif

		TC_SetPeriod(&TCD0, outputChannels[currentChannelOut]);

		currentChannelOut++;

	} else {

		int i = 0;
		int outputSum = 0;

		for (i = 0; i < NUMBER_OF_CHANNELS_OUT; i++) {
			outputSum += outputChannels[i];
		}

		currentChannelOut = 0;

		// if the next space is the sync space, calculates it's length
		uint32_t finalOutLen = PPM_FRAME_LENGTH - outputSum;
		TC_SetPeriod(&TCD0, (uint16_t) finalOutLen);
	}
}

// disable controllers
void disableController() {

	controllerEnabled = 0;
}

// enable controllers
void enableController() {

	if (controllerEnabled == 0) {

#if PX4FLOW_DATA_RECEIVE == ENABLED

		elevatorIntegration = 0;
		aileronIntegration = 0;
		throttleIntegration = 0;

#if GUMSTIX_DATA_RECEIVE == ENABLED
		if(validGumstix != 1) {

			estimatedElevatorPos = elevatorSetpoint;
			estimatedAileronPos  = aileronSetpoint;
		}
#else
		estimatedElevatorPos = elevatorSetpoint;
		estimatedAileronPos  = aileronSetpoint;
#endif // GUMSTIX_DATA_RECEIVE == ENABLED

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

/* -------------------------------------------------------------------- */
/*	Interrupt for timing the PPM output pulse, DO NOT MODIFY!			*/
/* -------------------------------------------------------------------- */
ISR(TCD0_CCA_vect)
{

	// shut down the output PPM pulse
	ppm_out_off();
#ifdef MULTI_OUTPUT_PPM
	ppm_out1_off();
	ppm_out2_off();
	ppm_out3_off();
	ppm_out4_off();
#endif

}

uint16_t getOutputChannelValue(uint8_t index)
{
	return outputChannels[index];
}

