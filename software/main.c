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
#include "controllers.h"
#include "system.h"

// state variable for incoming pulses
volatile uint16_t pulseStart[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
volatile uint16_t pulseEnd[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
volatile uint8_t pulseFlag[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

// values received from the RC
volatile uint16_t RCchannel[9] = {PULSE_MIN, PULSE_MIDDLE, PULSE_MIDDLE, PULSE_MIDDLE, PULSE_MIN, PULSE_MIN, PULSE_MIN, PULSE_MIN, PULSE_MIN};

// values to transmite to Flight-CTRL
volatile uint16_t outputChannels[6] = {PULSE_MIN, PULSE_MIDDLE, PULSE_MIDDLE, PULSE_MIDDLE, PULSE_MIN, PULSE_MIN};

// port mask of PC inputs
volatile uint8_t portMask = 0;

// port mask for PA inputs
volatile uint8_t portMask2 = 0;

// preserves the current out-transmitting channel number
volatile uint8_t currentChannelOut = 0;

// temporary var for current time preservation
volatile uint16_t currentTime = 0;

// the slow timer variable
volatile uint8_t myTimer = 0;

// the quadcopter armed state
volatile uint8_t vehicleArmed = 0;

// buttons variables
volatile char button1pressed = 0;
volatile char button1pressedMap = 0;
volatile char button2pressed = 0;
volatile char button2pressedMap = 0;
volatile char buttonChangeEnable = 0;

volatile uint8_t armingToggled = 0;
volatile uint8_t disarmingToggled = 0;

// flag to run the controllers
volatile int8_t controllersFlag = 0;

// controllers output variables
volatile int16_t controllerElevatorOutput = 0;
volatile int16_t controllerAileronOutput = 0;
volatile int16_t controllerThrottleOutput = 0;
volatile int16_t controllerRudderOutput = 0;

// variables for altitude controller
volatile float throttleIntegration = 0;
volatile float estimator_cycle = 0;
volatile float estimated_position = 0;
volatile float estimated_pos_prev = 0;
volatile float estimated_velocity = 0;
volatile float altitudeSetpoint = ALTITUDE_SETPOINT;

// constants from RC transmitter
volatile float constant1 = 0;
volatile float constant2 = 0;
volatile float constant3 = 0;
volatile float constant4 = 0;
volatile float constant5 = 0;

// timestamp for debug and logging
volatile double timeStamp = 0;
volatile int    debug_cycle = 0;

// controller on/off
volatile unsigned char controllerEnabled = 0;
volatile unsigned char positionControllerEnabled = 0;

// for on-off by AUX3 channel
int8_t previous_AUX3 = 0;

// for auto on-off by throttle stick
volatile int8_t previous_throttle = 0;

//~ --------------------------------------------------------------------
//~ Variables used with the px4flow sensor
//~ --------------------------------------------------------------------

#if (PX4FLOW_DATA_RECEIVE == ENABLED) || (ATOM_DATA_RECEIVE == ENABLED) || (GUMSTIX_DATA_RECEIVE == ENABLED)

volatile float elevatorSpeedSetpoint = 0;
volatile float aileronSpeedSetPoint = 0;
volatile float elevatorSpeedIntegration = 0;
volatile float aileronSpeedIntegration = 0;

#endif

#if PX4FLOW_DATA_RECEIVE == ENABLED

// variables used by the mavlink library
mavlink_message_t mavlinkMessage;
mavlink_status_t mavlinkStatus;
int8_t mavlinkFlag = 0;
mavlink_optical_flow_t opticalFlowData;
int8_t opticalFlowDataFlag = 0;

volatile float groundDistance = 0;

// speed controller variables
volatile float elevatorSpeedPreviousError = 0;
volatile float aileronSpeedPreviousError = 0;

#endif // PX4FLOW_DATA_RECEIVE == ENABLED

//~ --------------------------------------------------------------------
//~ Variables used with the Atom computer
//~ --------------------------------------------------------------------

#if ATOM_DATA_RECEIVE == ENABLED

// variables from surfnav computer
volatile int8_t previousConstant1 = 0;
volatile int8_t previousConstant2 = 0;
volatile unsigned char atomParseCharState = 0;
volatile unsigned char atomParseCharByte = 0;
volatile int16_t atomParseTempInt;
volatile int16_t xPosSurf = 0;
volatile int16_t yPosSurf = 0;
volatile int16_t xPosSurfNew = 0;
volatile int16_t yPosSurfNew = 0;
volatile int16_t headingSurf = 0;
volatile int16_t scaleSurf = 0;
volatile int8_t atomDataFlag = 0;

// buffers for angle synchronization

volatile int16_t pitchBuffer[PITCH_BUFFER_SIZE];
volatile uint8_t pitchBufferNum = 0;
volatile uint8_t pitchBufferLast = 0;
volatile uint8_t pitchBufferFirst = 0;

volatile int16_t rollBuffer[ROLL_BUFFER_SIZE];
volatile uint8_t rollBufferNum = 0;
volatile uint8_t rollBufferLast = 0;
volatile uint8_t rollBufferFirst = 0;

volatile int16_t delayedPitchAngle = 0;
volatile int16_t delayedRollAngle = 0;

#endif

//~ --------------------------------------------------------------------
//~ Variables for communication with Gumstix
//~ --------------------------------------------------------------------

#if GUMSTIX_DATA_RECEIVE == ENABLED

// variables for gumstix
volatile unsigned char gumstixParseCharState = 0;
volatile unsigned char gumstixParseCharByte = 0;
volatile int16_t gumstixParseTempInt;
volatile int16_t xPosGumstixNew = 0;
volatile int16_t yPosGumstixNew = 0;
volatile int16_t zPosGumstixNew = 0;
volatile int16_t xPosGumstix = 0;
volatile int16_t yPosGumstix = 0;
volatile int16_t zPosGumstix = 0;
volatile int8_t validGumstix = 0;
volatile int8_t gumstixDataFlag = 0;
volatile int16_t aileronSetPoint = 0;
volatile int16_t elevatorSetPoint = 2000;
volatile unsigned char gumstixParseCharCrc = 0;
volatile float gumstixElevatorIntegral = 0;
volatile float gumstixAileronIntegral = 0;
volatile int16_t xGumstixPreviousError = 0;
volatile int16_t yGumstixPreviousError = 0;

#endif

//~ --------------------------------------------------------------------
//~ Variables for communication with flightCTRL
//~ --------------------------------------------------------------------

#if (FLIGHTCTRL_DATA_RECEIVE == ENABLED) || (ATOM_DATA_RECEIVE == ENABLED)

// variables used by the Mikrokopters functions itself
unsigned volatile char flightCtrlDataFlag = 0;
unsigned volatile char CntCrcError = 0;
unsigned volatile char BytesReceiving = 0;
signed volatile char TxdBuffer[MAX_SENDE_BUFF];
signed volatile char RxdBuffer[MAX_EMPFANGS_BUFF];
unsigned volatile char transfereUart1done = 1;
unsigned char interval = 1;

signed char *pRxData = 0;
unsigned char RxDataLen = 0;

// aux variables
int8_t volatile flightCtrlDataBeingReceived = 0;
// tracks how much bytes it has been received
// if it is more than 15, shut down the
// receiving (flightCtrlDataBeingReceived %= 0)
int8_t volatile flightCtrlByteReceive = 0;
volatile int8_t flightCtrlDataReceived = 0;

#endif

// STATE VARIABLES
// angles with respect to the board
volatile int16_t pitchBoardAngle = 0;
volatile int16_t rollBoardAngle = 0;
// angles with respect to the "front"
volatile int16_t pitchAngle = 0;
volatile int16_t rollAngle = 0;
// speeds
volatile float elevatorSpeed = 0;
volatile float aileronSpeed = 0;

// write debug message to Uart0 serial output
void debug() {

	char num[20];
	static double  dbg_start;
	double dbg_duration;
	
	static float   dbg_elevatorSpeed_raw;
	static float   dbg_elevatorSpeed;
	static int16_t dbg_xPosGumstix_raw;
	static int16_t dbg_xPosGumstix;
	static int16_t dbg_pitchAngle;
	static int16_t dbg_elevator_auto;
	static int16_t dbg_elevator_man;

	static float   dbg_aileronSpeed_raw;
	static float   dbg_aileronSpeed;
	static int16_t dbg_yPosGumstix_raw;
	static int16_t dbg_yPosGumstix;
	static int16_t dbg_rollAngle;
	static int16_t dbg_aileron_auto;
	static int16_t dbg_aileron_man;


	if(debug_cycle == 0) {

		//start time trace
		dbg_start = timeStamp;
		sprintf(num, "%f", ((double) dbg_start));
		Uart0_write_string(num, strlen(num));
		Uart0_write_char(' ');

	} else if(debug_cycle == 1)  { //1+7 values (cycle 0+1)

		//throttle trace
		sprintf(num, "%f", ((double) groundDistance));
		Uart0_write_string(num, strlen(num));
		Uart0_write_char(' ');

		sprintf(num, "%f", ((double) estimated_position));
		Uart0_write_string(num, strlen(num));
		Uart0_write_char(' ');

		sprintf(num, "%f", ((double) estimated_velocity));
		Uart0_write_string(num, strlen(num));
		Uart0_write_char(' ');

		sprintf(num, "%f", ((double) altitudeSetpoint)); //altitude
		Uart0_write_string(num, strlen(num));
		Uart0_write_char(' ');

		sprintf(num, "%i", ((int16_t) controllerThrottleOutput));
		Uart0_write_string(num, strlen(num));
		Uart0_write_char(' ');

		sprintf(num, "%f", ((double) throttleIntegration));
		Uart0_write_string(num, strlen(num));
		Uart0_write_char(' ');

		sprintf(num, "%i", ((int16_t) RCchannel[THROTTLE])); //man
		Uart0_write_string(num, strlen(num));
		Uart0_write_char(' ');

		//elevator snapshot
		dbg_elevatorSpeed_raw = opticalFlowData.flow_comp_m_x;
		dbg_elevatorSpeed = elevatorSpeed;
		dbg_xPosGumstix_raw = xPosGumstixNew;
		dbg_xPosGumstix = xPosGumstix;
		dbg_pitchAngle = pitchAngle;
		dbg_elevator_auto = controllerElevatorOutput;
		dbg_elevator_man = RCchannel[ELEVATOR];

		//aileron snapshot
		dbg_aileronSpeed_raw = opticalFlowData.flow_comp_m_y;
		dbg_aileronSpeed = aileronSpeed;
		dbg_yPosGumstix_raw = yPosGumstixNew;
		dbg_yPosGumstix = yPosGumstix;
		dbg_rollAngle = rollAngle;
		dbg_aileron_auto = controllerAileronOutput;
		dbg_aileron_man = RCchannel[AILERON];

	} else if(debug_cycle == 2)  { //1+7 values

		//controllers enabled
		sprintf(num, "%i", ((int16_t) controllerEnabled)); //may be ahead
		Uart0_write_string(num, strlen(num));
		Uart0_write_char(' ');

		//elevator trace
		sprintf(num, "%f", ((double) dbg_elevatorSpeed_raw)); //px4flow
		Uart0_write_string(num, strlen(num));
		Uart0_write_char(' ');

		sprintf(num, "%f", ((double) dbg_elevatorSpeed)); //px4flow
		Uart0_write_string(num, strlen(num));
		Uart0_write_char(' ');

		sprintf(num, "%i", ((int16_t) dbg_xPosGumstix_raw)); //px4flow
		Uart0_write_string(num, strlen(num));
		Uart0_write_char(' ');

		sprintf(num, "%i", ((int16_t) dbg_xPosGumstix)); //px4flow
		Uart0_write_string(num, strlen(num));
		Uart0_write_char(' ');

		sprintf(num, "%i", ((int16_t) dbg_pitchAngle)); //stab.board
		Uart0_write_string(num, strlen(num));
		Uart0_write_char(' ');

		sprintf(num, "%i", ((int16_t) dbg_elevator_auto));
		Uart0_write_string(num, strlen(num));
		Uart0_write_char(' ');

		sprintf(num, "%i", ((int16_t) dbg_elevator_man));
		Uart0_write_string(num, strlen(num));
		Uart0_write_char(' ');

	} else if(debug_cycle == 3)  { //1+7 values

		//aileron trace
		sprintf(num, "%f", ((double) dbg_aileronSpeed_raw)); //px4flow
		Uart0_write_string(num, strlen(num));
		Uart0_write_char(' ');

		sprintf(num, "%f", ((double) dbg_aileronSpeed)); //px4flow
		Uart0_write_string(num, strlen(num));
		Uart0_write_char(' ');

		sprintf(num, "%i", ((int16_t) dbg_yPosGumstix_raw)); //px4flow
		Uart0_write_string(num, strlen(num));
		Uart0_write_char(' ');

		sprintf(num, "%i", ((int16_t) dbg_yPosGumstix)); //px4flow
		Uart0_write_string(num, strlen(num));
		Uart0_write_char(' ');

		sprintf(num, "%i", ((int16_t) dbg_rollAngle)); //stab.board
		Uart0_write_string(num, strlen(num));
		Uart0_write_char(' ');

		sprintf(num, "%i", ((int16_t) dbg_aileron_auto));
		Uart0_write_string(num, strlen(num));
		Uart0_write_char(' ');

		sprintf(num, "%i", ((int16_t) dbg_aileron_man));
		Uart0_write_string(num, strlen(num));
		Uart0_write_char(' ');

		//duration trace
		dbg_duration = (timeStamp - dbg_start) / 0.0142222;
		sprintf(num, "%f", ((double) dbg_duration));
		Uart0_write_string(num, strlen(num));
		Uart0_write_char('\n');
	}
}

int main() {

	// initialize the MCU (timers, uarts and so on)
	initializeMCU();

	// the main while cycle
	while (1) {

		// runs controllers
		if (controllersFlag == 1) {
		
			controllersFlag = 0;

			if (positionControllerEnabled == 1) {

#if ATOM_DATA_RECEIVE == ENABLED
				controllerElevator_surfnav();
				controllerAileron_surfnav();
#endif
				led_Y_on();

			} else {
				led_Y_off();

#if PX4FLOW_DATA_RECEIVE == ENABLED
				aileronSpeedSetPoint = 0;
				elevatorSpeedSetpoint = 0;
#endif
			}
			
#if LOGGING_ON == ENABLED

			if(debug_cycle == 0) {
				debug();
				debug_cycle++;
			}

#endif

#if PX4FLOW_DATA_RECEIVE == ENABLED

			altitudeSetpoint = ALTITUDE_SETPOINT + ((constant1-0.2)/1.6)/2; // c1=<0.2; 1.8>

			controllerThrottleEstimator();

			controllerElevatorSpeed();
			controllerAileronSpeed();
			controllerThrottle();

#endif

#if LOGGING_ON == ENABLED

			debug();
			
			debug_cycle = (debug_cycle+1)%4;

#endif

		}

		// PWM input capture
		capturePWMInput();

		// autooff throttle when the RC throttle stick went down
		if (RCchannel[THROTTLE] > 2700) {
			previous_throttle = 1;
		} else {
			if (previous_throttle == 1) {
				//~ disableController();
				disarmVehicle();
			}
			previous_throttle = 0;
		}

		// controller on/off
		if (abs(RCchannel[AUX3] - PULSE_MIDDLE) < 200) {
			if (previous_AUX3 == 0) {
				enableController();
			}
			disablePositionController();
			previous_AUX3 = 1;
		} else if (RCchannel[AUX3] > (PULSE_MIDDLE + 200)) {
			if (previous_AUX3 == 1) {
				enablePositionController();
			}
			previous_AUX3 = 2;
		} else {
			disableController();
			disablePositionController();
			previous_AUX3 = 0;
		}

		// load the constant values from the RC
		// <0.2; 1.8>
		constant1 = (float)((RCchannel[AUX1] - 2304))/1152;
		constant2 = (float)((RCchannel[AUX2] - 2304))/1152;
		constant4 = (float)((RCchannel[AUX4] - 2304))/1152;
		constant5 = (float)((RCchannel[AUX5] - 2304))/1152;

#if ATOM_DATA_RECEIVE == ENABLED

		// sends r char to the surfnav computer
		if (constant2 > 1.3) {

			if (previousConstant2 == 0) {
				Uart0_write_char('r');
			}

			previousConstant2 = 1;
		} else if (constant2 < 0.5) {
			previousConstant2 = 0;
		}

		//~ // sends l char to the surfnav computer
		//~ if (constant1 > 1.3) {
//~ 
			//~ if (previousConstant1 == 0) {
				//~ Uart0_write_char('l');
				//~ timeStamp = 0;
			//~ }
//~ 
			//~ previousConstant1 = 1;
		//~ } else if (constant1 < 0.5) {
			//~ if (previousConstant1 == 1) {
//~ 
				//~ Uart0_write_char('!');
			//~ }
//~ 
			//~ previousConstant1 = 0;
		//~ }

#endif

#if PX4FLOW_DATA_RECEIVE == ENABLED

		// filter the optical flow velocity data
		if (opticalFlowDataFlag == 1) {

			// decode the message (there will be new values in opticalFlowData...
			mavlink_msg_optical_flow_decode(&mavlinkMessage, &opticalFlowData);

			elevatorSpeed = elevatorSpeed*PX4FLOW_FILTER_WEIGHT + opticalFlowData.flow_comp_m_x*(1 - PX4FLOW_FILTER_WEIGHT);
			aileronSpeed = aileronSpeed*PX4FLOW_FILTER_WEIGHT + opticalFlowData.flow_comp_m_y*(1 - PX4FLOW_FILTER_WEIGHT);

			if (opticalFlowData.ground_distance < 2) {

				groundDistance = opticalFlowData.ground_distance;
			}

			// integrate velocities
			elevatorSpeedIntegration = elevatorSpeedIntegration*0.99 + elevatorSpeed*0.01428;
			aileronSpeedIntegration = aileronSpeedIntegration*0.99 + aileronSpeed*0.01428;

			//~ elevatorSpeed = opticalFlowData.flow_comp_m_x;
			//~ aileronSpeed = opticalFlowData.flow_comp_m_y;

			led_Y_toggle();

			opticalFlowDataFlag = 0;
		}

#endif // PX4FLOW_DATA_RECEIVE == ENABLED

#if ATOM_DATA_RECEIVE == ENABLED

		// filter the surfnav position data
		if (atomDataFlag == 1) {

			if (abs(xPosSurfNew) < 2000) {

				xPosSurf = xPosSurf*SURFNAV_FILTER_WEIGHT + xPosSurfNew*(1 - SURFNAV_FILTER_WEIGHT);
			}

			if (abs(yPosSurfNew) < 2000) {

				yPosSurf =  yPosSurf*SURFNAV_FILTER_WEIGHT + yPosSurfNew*(1 - SURFNAV_FILTER_WEIGHT);
			}

			//~ xPosSurf = atomParseTempInt;
			//~ yPosSurf = atomParseTempInt;

			led_control_toggle();

			atomDataFlag = 0;
		}

#endif

#if (ATOM_DATA_RECEIVE == ENABLED) || (FLIGHTCTRL_DATA_RECEIVE == ENABLED)

		if (flightCtrlDataFlag == 1) {

			Decode64();

			char* dummy;
			int16_t tempAngle;

			dummy = (char*)(&tempAngle);

			dummy[0] = pRxData[0];
			dummy[1] = pRxData[1];

			if (RxdBuffer[2] == 'X') {

				pitchBoardAngle = tempAngle;

			} else if (RxdBuffer[2] == 'Y') {

				rollBoardAngle = tempAngle;
			}

#if FRAME_ORIENTATION == X_COPTER

			pitchAngle = (pitchBoardAngle-rollBoardAngle)/2;
			rollAngle = (pitchBoardAngle+rollBoardAngle)/2;

#endif

#if FRAME_ORIENTATION == PLUS_COPTER

// zpožďování dat o úhlu
#if ATOM_DATA_RECEIVE == ENABLED

			pitchBufferPut(pitchBoardAngle);
			rollBufferPut(rollBoardAngle);
			
			delayedPitchAngle = pitchBufferGet();
			delayedRollAngle = rollBufferGet();

#endif

			pitchAngle = pitchBoardAngle;
			rollAngle = rollBoardAngle;

#endif

			flightCtrlDataFlag = 0;
		}

#endif

#if GUMSTIX_DATA_RECEIVE == ENABLED

		if (gumstixDataFlag == 1) {

			// rotate the coordinates
#if GUMSTIX_CAMERA_POINTING == FORWARD

			if (validGumstix == 1) {

				xPosGumstix = xPosGumstix*0.4 + xPosGumstixNew*0.6;
				yPosGumstix = yPosGumstix*0.4 + zPosGumstixNew*0.6;
				zPosGumstix = zPosGumstix*0.4 + yPosGumstixNew*0.6;

			}

#endif

			gumstixDataFlag = 0;
		}

#endif

		// set outputs signals for MK only if it should be
		mergeSignalsToOutput();

		// button1 was pressed
		//~ if (button1check()) {}

		// button2 was pressed
		//~ if (button2check()) {}

		// gets triggered every second
		// handles slow periodic events
		if (myTimer >= 61) {

			buttonChangeEnable = 1;

			myTimer = 0;

			// arming procedure
			if ((armingToggled >= 1) && (armingToggled <= 4)) {

				armingToggled++;
			} else if (armingToggled == 5) {

				outputChannels[0] = PULSE_MIN;
				outputChannels[1] = PULSE_MIDDLE;
				armingToggled++;
			} else if (armingToggled == 6) {

				outputChannels[0] = PULSE_MIN;
				outputChannels[1] = PULSE_MIN;
				armingToggled++;
			} else if (armingToggled == 7) {

				outputChannels[0] = PULSE_MIN;
				outputChannels[1] = PULSE_MIDDLE;
				armingToggled = 0;
				// enableController();
			}

			// disarming procedure
			if ((disarmingToggled >= 1) && (disarmingToggled <= 3)) {

				disarmingToggled++;
			} else if (disarmingToggled == 4) {

				outputChannels[0] = PULSE_MIN;
				outputChannels[1] = PULSE_MIDDLE;
				disarmingToggled = 0;
			}
		}
	}

	return 0;
}

//~ --------------------------------------------------------------------
//~ Serial communication handling
//~ --------------------------------------------------------------------

// interrupt fired by Uart1
ISR(USART0_RX_vect) {

	char incomingChar = UDR0;

#if (PX4FLOW_DATA_RECEIVE == ENABLED) && (PX4FLOW_RECEIVE_PORT == UART0)

	px4flowParseChar(incomingChar);

#endif

#if (GUMSTIX_DATA_RECEIVE == ENABLED) && (GUMSTIX_RECEIVE_PORT == UART0)

	gumstixParseChar(incomingChar);

#endif

#if (FLIGHTCTRL_DATA_RECEIVE == ENABLED) && (FLIGHTCTRL_RECEIVE_PORT == UART0)

	flightCtrlParseChar(incomingChar);

#endif

#if (ATOM_DATA_RECEIVE == ENABLED) && (ATOM_RECEIVE_PORT == UART0)

	atomParseChar(incomingChar);

#endif

}

// interrupt fired by Uart1
ISR(USART1_RX_vect) {

	int incomingChar = UDR1;

#if (PX4FLOW_DATA_RECEIVE == ENABLED) && (PX4FLOW_RECEIVE_PORT == UART1)

	px4flowParseChar(incomingChar);

#endif

#if (GUMSTIX_DATA_RECEIVE == ENABLED) && (GUMSTIX_RECEIVE_PORT == UART1)

	gumstixParseChar(incomingChar);

#endif

#if (FLIGHTCTRL_DATA_RECEIVE == ENABLED) && (FLIGHTCTRL_RECEIVE_PORT == UART1)

	flightCtrlParseChar(incomingChar);

#endif

#if (ATOM_DATA_RECEIVE == ENABLED) && (ATOM_RECEIVE_PORT == UART1)

	atomParseChar(incomingChar);

#endif
}

//~ --------------------------------------------------------------------
//~ Generation of PPM output
//~ --------------------------------------------------------------------

// fires interrupt on 16bit timer (starts the new PPM pulse)
ISR(TIMER1_COMPA_vect) {

	currentTime = TCNT1;

	// startes the output PPM pulse
	pulse1_on();

	if (currentChannelOut < 6) {
		OCR1A = currentTime+outputChannels[currentChannelOut];
		currentChannelOut++;

	} else {

		// if the next space is the sync space, calculates it's length
		OCR1A = currentTime+PPM_FRAME_LENGTH-outputChannels[0]-outputChannels[1]-outputChannels[2]-outputChannels[3]-outputChannels[4]-outputChannels[5];
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

	myTimer++;
	timeStamp += 0.0142222;
	controllersFlag = 1;
}

