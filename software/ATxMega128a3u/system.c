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
#include "commTask.h"
#include "controllers.h"
#include "config.h"
#include "commTask.h"

/* -------------------------------------------------------------------- */
/*	Variables for PPM input capture										*/
/* -------------------------------------------------------------------- */
uint16_t PPM_in_start = 0;
uint8_t PPM_in_current_channel = 0;
volatile uint16_t RCchannel[9] = {PPM_IN_MIN_LENGTH, PPM_IN_MIDDLE_LENGTH, PPM_IN_MIDDLE_LENGTH, PPM_IN_MIDDLE_LENGTH, PPM_IN_MIN_LENGTH, PPM_IN_MIN_LENGTH, PPM_IN_MIN_LENGTH, PPM_IN_MIN_LENGTH, PPM_IN_MIN_LENGTH};
volatile uint8_t channelUpdated[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
volatile uint8_t inFailsave = 0;

#ifdef PWM_INPUT

volatile uint8_t portAMask = 0;
volatile uint8_t portRMask = 0;
volatile uint8_t portDMask = 0;
volatile uint16_t pulseStart[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
volatile uint16_t pulseEnd[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
volatile uint8_t pulseFlag[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

#endif

/* -------------------------------------------------------------------- */
/*	Variables for PPM output generation									*/
/* -------------------------------------------------------------------- */
volatile uint16_t outputChannels[6] = {PULSE_OUT_MIN, PULSE_OUT_MIDDLE, PULSE_OUT_MIDDLE, PULSE_OUT_MIDDLE, PULSE_OUT_MIN, PULSE_OUT_MIN};
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
/*	Realtime clock														*/
/* -------------------------------------------------------------------- */
volatile int16_t milisecondsTimer = 0;
volatile int16_t secondsTimer;
volatile int16_t hoursTimer;

/* -------------------------------------------------------------------- */
/*	Queue from main to comms											*/
/* -------------------------------------------------------------------- */
xQueueHandle main2commsQueue;

volatile int8_t auxSetpointFlag = 0;

/* -------------------------------------------------------------------- */
/*	Basic initialization of the MCU, peripherals and i/o				*/
/* -------------------------------------------------------------------- */
void boardInit(void) {
	
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
	/*	Timer for RTC														*/
	/* -------------------------------------------------------------------- */
	
	// select the clock source and pre-scaler by 8
	TC1_ConfigClockSource(&TCC1, TC_CLKSEL_DIV64_gc);
	
	TC1_SetOverflowIntLevel(&TCC1, TC_OVFINTLVL_LO_gc);
	
	TC_SetPeriod(&TCC1, 499);
	
	milisecondsTimer = 0;
	secondsTimer = 0;
	hoursTimer = 0;
	
	/* -------------------------------------------------------------------- */
	/*	setup PD4 as a PPM input with interrupt								*/
	/* -------------------------------------------------------------------- */
	
	#ifdef PPM_INPUT
	
	// self explanatory
	PORT_ConfigurePins(&PORTD, 0x10, false, false, PORT_OPC_TOTEM_gc, PORT_ISC_RISING_gc);
	PORT_SetPinsAsInput(&PORTD, 0x10);
	
	// configure interrupt on PD4
	PORT_ConfigureInterrupt0(&PORTD, PORT_INT0LVL_LO_gc, 0x10);
	
	#endif

	// in case of PWM (9-Wire) input from the RC receiver
	#ifdef PWM_INPUT

	// setup PORTA (pins 0 to 5) as inputs 1 to 6s
	PORT_ConfigurePins(&PORTA, 0x3F, false, false, PORT_OPC_TOTEM_gc, PORT_ISC_BOTHEDGES_gc);
	PORT_SetPinsAsInput(&PORTA, 0x3F);
	
	// configure interrupt on PA
	PORT_ConfigureInterrupt0(&PORTA, PORT_INT0LVL_LO_gc, 0x3F);
	
	// setup PORTR (pins 0 to 1) as inputs 7 to 8
	PORT_ConfigurePins(&PORTR, 0x3, false, false, PORT_OPC_TOTEM_gc, PORT_ISC_BOTHEDGES_gc);
	PORT_SetPinsAsInput(&PORTR, 0x3);
	
	// configure interrupt on PR
	PORT_ConfigureInterrupt0(&PORTR, PORT_INT0LVL_LO_gc, 0x3);
	
	// setup PORTD (pin 4) as in 9
	PORT_ConfigurePins(&PORTD, 0x10, false, false, PORT_OPC_TOTEM_gc, PORT_ISC_BOTHEDGES_gc);
	PORT_SetPinsAsInput(&PORTD, 0x10);
	
	// configure interrupt on PD4
	PORT_ConfigureInterrupt0(&PORTD, PORT_INT0LVL_LO_gc, 0x10);

	#endif
	
	#ifdef PRASE
	
	#ifdef GRIPPER
	
	ioport_set_pin_dir(GRIP_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(GRIP_PIN, false);
	
	#endif
	
	#endif
	
	/* -------------------------------------------------------------------- */
	/*	Initialize USARTs													*/
	/* -------------------------------------------------------------------- */
	usart_buffer_1 = usartBufferInitialize(&USART_1, USART_1_BAUDRATE, 64);
	usart_buffer_2 = usartBufferInitialize(&USART_2, USART_2_BAUDRATE, 64);
	usart_buffer_3 = usartBufferInitialize(&USART_3, USART_3_BAUDRATE, 64);
	usart_buffer_4 = usartBufferInitialize(&USART_4, USART_4_BAUDRATE, 64);
	usart_buffer_stm = usartBufferInitialize(&USART_STM, USART_STM_BAUDRATE, 64);
	usart_buffer_xbee = usartBufferInitialize(&USART_XBEE, USART_XBEE_BAUDRATE, 64);
	usart_buffer_log = usartBufferInitialize(&USART_LOG, USART_LOG_BAUDRATE, 64);
	
	/* -------------------------------------------------------------------- */
	/*	enable low-level interrupts											*/
	/* -------------------------------------------------------------------- */
	PMIC_EnableLowLevel();
	
	/* -------------------------------------------------------------------- */
	/*	Initialize queues													*/
	/* -------------------------------------------------------------------- */
	main2commsQueue = xQueueCreate(10, sizeof(main2commMessage_t));
}

/* -------------------------------------------------------------------- */
/*	Merge signals from RC Receiver with the controller outputs			*/
/* -------------------------------------------------------------------- */
void mergeSignalsToOutput(void) {
	
	int16_t outputThrottle;
	int16_t outputElevator;
	int16_t outputAileron;
	int16_t outputRudder;
		
	portENTER_CRITICAL();
	
	if (inFailsave == 0) {

		outputThrottle = RCchannel[THROTTLE];
		outputElevator = RCchannel[ELEVATOR];
		outputAileron = RCchannel[AILERON];
		outputRudder = RCchannel[RUDDER];
		
	} else {
		
		outputThrottle = PULSE_OUT_MIN;
		outputElevator = PULSE_OUT_MIDDLE;
		outputAileron = PULSE_OUT_MIDDLE;
		outputRudder = PULSE_OUT_MIDDLE;
	}

	// add altitude controller to output
	if (altitudeControllerEnabled == true) {

		// saturate controller's throttle output before adding it to the RC channel
		if (controllerThrottleOutput > CONTROLLER_SATURATION)
			controllerThrottleOutput = CONTROLLER_SATURATION;
		else if (controllerThrottleOutput < -CONTROLLER_SATURATION)
			controllerThrottleOutput = -CONTROLLER_SATURATION;
		
		outputThrottle += controllerThrottleOutput;
	}
	
	// add mpc controller controller to output
	if (positionControllerEnabled == true) {
		
		// saturate controller's elevator output before adding it to the RC channel
		if (controllerElevatorOutput > CONTROLLER_SATURATION)
			controllerElevatorOutput = CONTROLLER_SATURATION;
		else if (controllerElevatorOutput < -CONTROLLER_SATURATION)
			controllerElevatorOutput = -CONTROLLER_SATURATION;
			
		// saturate controller's aileron output before adding it to the RC channel
		if (controllerAileronOutput > CONTROLLER_SATURATION)
			controllerAileronOutput = CONTROLLER_SATURATION;
		else if (controllerAileronOutput < -CONTROLLER_SATURATION)
			controllerAileronOutput = -CONTROLLER_SATURATION;

		outputElevator += controllerElevatorOutput;
		outputAileron += controllerAileronOutput;
		led_blue_on();
	} else
		led_blue_off();

	// Everithing is *2 because the PPM incoming to this board is twice slower then the PPM going out
	outputChannels[0] = outputThrottle;
	outputChannels[1] = outputRudder;
	outputChannels[2] = outputElevator;
	outputChannels[3] = outputAileron;
	
	portEXIT_CRITICAL();
}

#ifdef PPM_INPUT

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
		channelUpdated[PPM_in_current_channel] = 1;
		PPM_in_current_channel++;
	} else {
		
		PPM_in_current_channel++;
	}
	
	PPM_in_start = PPM_in_end;
}

#endif

#ifdef PWM_INPUT

ISR(PORTA_INT0_vect) {

	uint16_t currentTime = TCD1.CNT;
	uint8_t i;

	// walks through all 5 channel inputs
	for(i = 0; i < 6; i++) {
		
		// i-th port has changed
		if ((PORTA.IN ^ portAMask) & _BV(5-i)) {

			// i-th pin is now HIGH
			if (PORTA.IN & _BV(5-i)) {
				
				pulseStart[i] = currentTime;
			} else { // i-th pin is no LOW
				
				pulseEnd[i] = currentTime;
				pulseFlag[i] = 1;
			}
		}
	}
	
	// saves the current A port mask
	portAMask = PORTA.IN;
}

// capture the channel AUX
ISR(PORTR_INT0_vect) {

	uint16_t currentTime = TCD1.CNT;
	uint8_t i;

	// walks through all 5 channel inputs
	for(i = 6; i < 8; i++) {
		
		// i-th port has changed
		if ((PORTR.IN ^ portRMask) & _BV(1-(i-6))) {

			// i-th pin is now HIGH
			if (PORTR.IN & _BV(1-(i-6))) {
				
				pulseStart[i] = currentTime;
			} else { // i-th pin is no LOW
				
				pulseEnd[i] = currentTime;
				pulseFlag[i] = 1;
			}
		}
	}
	
	// saves the current A port mask
	portRMask = PORTR.IN;
}

// capture the channel AUX5 from PD4
ISR(PORTD_INT0_vect) {

	uint16_t currentTime = TCD1.CNT;
		
	// i-th port has changed
	if ((PORTD.IN ^ portDMask) & _BV(4)) {

		// i-th pin is now HIGH
		if (PORTD.IN & _BV(4)) {
				
			pulseStart[8] = currentTime;
		} else { // i-th pin is no LOW
				
			pulseEnd[8] = currentTime;
			pulseFlag[8] = 1;
		}
	}
	
	// saves the current A port mask
	portDMask = PORTD.IN;
}

#endif

/* -------------------------------------------------------------------- */
/*	Interrupt for timing the PPM output pulse, DO NOT MODIFY!			*/
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

/* -------------------------------------------------------------------- */
/*	Interrupt for timing the PPM output pulse, DO NOT MODIFY!			*/
/* -------------------------------------------------------------------- */
ISR(TCD0_CCA_vect) {
	
	// shut down the output PPM pulse
	ppm_out_off();
}

#ifdef PWM_INPUT

void capture_pwm_inputs(void) {
	
    // PWM input capture
	uint8_t j;
	uint16_t pulseLength;
	
	// ite/rate through all channel possible inputs
    for(j = 0; j < 9; j++) {
	    
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
		    if ((pulseLength >= PWM_IN_MIN_LENGTH) && (pulseLength <= PWM_IN_MAX_LENGTH)) {
			    
			    // set the length
			    RCchannel[j] = pulseLength;
				channelUpdated[j] = 1;
		    }
		    
		    // clear the flag
		    pulseFlag[j] = 0;
	    }
    }
	 
}

#endif

/* -------------------------------------------------------------------- */
/*	Failsave for NOT receiving RC channels								*/
/* -------------------------------------------------------------------- */
void RCInputFailsave(void) {
	
	uint8_t i;
	
	// iterate the control channels
	for (i = 0; i < 4; i++) {
		
		if (channelUpdated[i] == 0) {
			
			// problem, jump to fail save
			led_red_on();
			inFailsave = 1;
			altitudeControllerEnabled = 0;
			positionControllerEnabled = 0;
			
		} else {
			
			channelUpdated[i] = 0;
		}
	}
}

/* -------------------------------------------------------------------- */
/*	Interrupt for timing the RTC & mergeSignalsToOutput					*/
/* -------------------------------------------------------------------- */
ISR(TCC1_OVF_vect) {
	
	auxSetpointFlag = 1;
	
	if (milisecondsTimer++ == 1000) {

		if ((secondsTimer > 3) || (hoursTimer > 0))
			RCInputFailsave();
		
		mpcRate = mpcCounter;
		mpcCounter = 0;
		kalmanRate = kalmanCounter;
		kalmanCounter = 0;
		xbeeflag = 1;
		commTaskRate = commTaskCounter;
		commTaskCounter = 0;
		
		milisecondsTimer = 0;
		
		if (secondsTimer++ == 3600) {
			
			secondsTimer = 0;
			hoursTimer++;
		}
	}
	
	#ifdef PWM_INPUT
	capture_pwm_inputs();
	#endif
	
	mergeSignalsToOutput();
}