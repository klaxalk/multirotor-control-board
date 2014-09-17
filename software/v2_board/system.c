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
#include "communication.h"

// for usart
#include "usart_driver.h"
#include "avr_compiler.h"

/* -------------------------------------------------------------------- */
/*	Variables for PPM input capture										*/
/* -------------------------------------------------------------------- */
uint16_t PPM_in_start = 0;
uint8_t PPM_in_current_channel = 0;
volatile uint16_t RCchannel[9] = {PPM_IN_MIN_LENGTH, PPM_IN_MIN_LENGTH, PPM_IN_MIN_LENGTH, PPM_IN_MIN_LENGTH, PPM_IN_MIN_LENGTH, PPM_IN_MIN_LENGTH, PPM_IN_MIN_LENGTH, PPM_IN_MIN_LENGTH, PPM_IN_MIN_LENGTH};

/* -------------------------------------------------------------------- */
/*	Variables for PPM output generation									*/
/* -------------------------------------------------------------------- */
volatile uint16_t outputChannels[6] = {PULSE_OUT_MIN, PULSE_OUT_MIN, PULSE_OUT_MIN, PULSE_OUT_MIN, PULSE_OUT_MIN, PULSE_OUT_MIN};
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
/*	USART variables														*/
/* -------------------------------------------------------------------- */
USART_data_t usart_xbee;
USART_data_t usart_1;

extern volatile float elevatorIntegration;
extern volatile float aileronIntegration;
extern volatile float throttleIntegration;
extern volatile float elevatorSetpoint;
extern volatile float aileronSetpoint;
extern volatile float throttleSetpoint;

//vars for estimators
extern volatile float estimatedElevatorPos;
extern volatile float estimatedAileronPos;
extern volatile float estimatedThrottlePos;
extern volatile float estimatedElevatorVel;
extern volatile float estimatedAileronVel;
extern volatile float estimatedThrottleVel;

// flag to run the controllers
extern volatile int8_t controllersFlag;

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
	ioport_set_pin_dir(OUT1, IOPORT_DIR_OUTPUT);
	
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
	/*	Timer for 70Hz controller loop										*/
	/* -------------------------------------------------------------------- */
	
	TC0_ConfigClockSource(&TCC0, TC_CLKSEL_DIV1024_gc);
	
	// set the overflow interrupt
	TC0_SetOverflowIntLevel(&TCC0, TC_OVFINTLVL_LO_gc);
	
	// Set period ( TOP value ).
	TC_SetPeriod(&TCC0, (uint16_t) 446);
	
	/* -------------------------------------------------------------------- */
	/*	setup PD4 as a PPM input with interrupt								*/
	/* -------------------------------------------------------------------- */
	
	// self explanatory
	PORT_ConfigurePins(&PORTD, 0x10, false, false, PORT_OPC_TOTEM_gc, PORT_ISC_RISING_gc);
	PORT_SetPinsAsInput(&PORTD, 0x10);
	
	// configure interrupt on PD4
	PORT_ConfigureInterrupt0(&PORTD, PORT_INT0LVL_LO_gc, 0x10);
	
	/* -------------------------------------------------------------------- */
	/*	enable low-level interrupts											*/
	/* -------------------------------------------------------------------- */
	
	PMIC_EnableLowLevel();
}

void USART_XBEE_init() {

	/* Disable global interrupts */
	cli();

	/* PC2 (RXD0) as input. */
	PORTC.DIRCLR   = PIN2_bm;
  	/* PC3 (TXD0) as output. */
  	PORTC.DIRSET   = PIN3_bm;
	  
	/* Use USARTC0 and initialize buffers. */
	USART_InterruptDriver_Initialize(&usart_xbee, &USARTC0, USART_DREINTLVL_LO_gc);

	/* USARTC0, 8 Data bits, No Parity, 1 Stop bit. */
	USART_Format_Set(usart_xbee.usart, USART_CHSIZE_8BIT_gc,
	USART_PMODE_DISABLED_gc, false);

	/* Enable RXC interrupt. */
	USART_RxdInterruptLevel_Set(usart_xbee.usart, USART_RXCINTLVL_LO_gc);

	/* Select baudrate */
	USART_Baudrate_Set(&USARTC0, 3301 , -5);

	/* Enable both RX and TX. */
	USART_Rx_Enable(usart_xbee.usart);
	USART_Tx_Enable(usart_xbee.usart);
	
	/* Enable PMIC interrupt level low. */
	PMIC.CTRL |= PMIC_LOLVLEX_bm;
	
	/* Enable global interrupts. */
	sei();
}

/* -------------------------------------------------------------------- */
/*	USART_XBEE RX interrupt handler										*/
/* -------------------------------------------------------------------- */
ISR(USARTC0_RXC_vect)
{
	USART_RXComplete(&usart_xbee);
}


/* -------------------------------------------------------------------- */
/*	USART_XBEE TX interrupt handler										*/
/* -------------------------------------------------------------------- */
ISR(USARTC0_DRE_vect)
{
	USART_DataRegEmpty(&usart_xbee);
}

void USART_1_init() {

	/* Disable global interrupts */
	cli();

	/* PC2 (RXD0) as input. */
	PORTD.DIRCLR   = PIN6_bm;
	/* PC3 (TXD0) as output. */
	PORTD.DIRSET   = PIN7_bm;
	
	/* Use USARTC0 and initialize buffers. */
	USART_InterruptDriver_Initialize(&usart_1, &USARTD1, USART_DREINTLVL_LO_gc);

	/* USARTC0, 8 Data bits, No Parity, 1 Stop bit. */
	USART_Format_Set(usart_1.usart, USART_CHSIZE_8BIT_gc,
	USART_PMODE_DISABLED_gc, false);

	/* Enable RXC interrupt. */
	USART_RxdInterruptLevel_Set(usart_1.usart, USART_RXCINTLVL_LO_gc);

	/* Select baudrate */
	USART_Baudrate_Set(&USARTD1, 2234 , -7);

	/* Enable both RX and TX. */
	USART_Rx_Enable(usart_1.usart);
	USART_Tx_Enable(usart_1.usart);
	
	/* Enable PMIC interrupt level low. */
	PMIC.CTRL |= PMIC_LOLVLEX_bm;
	
	/* Enable global interrupts. */
	sei();
}

/* -------------------------------------------------------------------- */
/*	USART_1 RX interrupt handler										*/
/* -------------------------------------------------------------------- */
ISR(USARTD1_RXC_vect)
{
	USART_RXComplete(&usart_1);
}


/* -------------------------------------------------------------------- */
/*	USART_1 TX interrupt handler										*/
/* -------------------------------------------------------------------- */
ISR(USARTD1_DRE_vect)
{
	USART_DataRegEmpty(&usart_1);
}

/* -------------------------------------------------------------------- */
/*	Merge signals from RC Receiver with the controller outputs			*/
/* -------------------------------------------------------------------- */
void mergeSignalsToOutput() {

	int16_t outputThrottle = PULSE_OUT_MIN;
	int16_t outputElevator = PULSE_OUT_MIDDLE;
	int16_t outputAileron = PULSE_OUT_MIDDLE;
	int16_t outputRudder = PULSE_OUT_MIDDLE;

	outputThrottle = RCchannel[THROTTLE];
	outputRudder = RCchannel[RUDDER];
	outputElevator = RCchannel[ELEVATOR];
	outputAileron = RCchannel[AILERON];

	if (controllerEnabled == 1) {

		led_red_on();

		outputThrottle += controllerThrottleOutput;
		outputElevator += controllerElevatorOutput;
		outputAileron += controllerAileronOutput;
		//~ outputRudder += controllerRudderOutput;
		} else {

		led_red_off();
	}

	outputChannels[0] = outputThrottle*2;
	outputChannels[1] = outputRudder*2;
	outputChannels[2] = outputElevator*2;
	outputChannels[3] = outputAileron*2;
}

/* -------------------------------------------------------------------- */
/*	Interrupt for receiving PPM with RC Receiver signals				*/
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
/*	Interrupt for timing the PPM output pulse							*/
/* -------------------------------------------------------------------- */
ISR(TCD0_OVF_vect) {

	// starts the output PPM pulse
	ppm_out_on();

	if (currentChannelOut < NUMBER_OF_CHANNELS_OUT) {
		
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

		if(validGumstix != 1) {

			estimatedElevatorPos = elevatorSetpoint;
			estimatedAileronPos  = aileronSetpoint;

		}

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
/*	Interrupt for timing the PPM output pulse							*/
/* -------------------------------------------------------------------- */
ISR(TCD0_CCA_vect) {
	
	// shut down the output PPM pulse
	ppm_out_off();
}

/* -------------------------------------------------------------------- */
/*	Interrupt for 70Hz control loop										*/
/* -------------------------------------------------------------------- */
ISR(TCC0_OVF_vect) {

	controllersFlag = 1;
}