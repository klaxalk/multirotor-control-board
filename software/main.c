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

// flag to run the controllers
volatile int8_t controllersFlag = 0;

// controllers output variables
volatile int16_t controllerElevatorOutput = 0;
volatile int16_t controllerAileronOutput = 0;
volatile int16_t controllerThrottleOutput = 0;
volatile int16_t controllerRudderOutput = 0;

// constants from RC transmitter
volatile float constant1 = 0;
volatile float constant2 = 0;
volatile float constant3 = 0;
volatile float constant4 = 0;
volatile float constant5 = 0;

// timestamp for debug and logging
volatile double timeStamp = 0;
volatile uint16_t main_cycle = 0;

// controller on/off
volatile unsigned char controllerEnabled = 0;
volatile unsigned char positionControllerEnabled = 0;
volatile unsigned char landingMode = 0;

// for on-off by AUX3 channel
int8_t previous_AUX3 = 0;

//~ --------------------------------------------------------------------
//~ Variables used with the px4flow sensor
//~ --------------------------------------------------------------------

#if PX4FLOW_DATA_RECEIVE == ENABLED

// variables used by the mavlink library
mavlink_message_t mavlinkMessage;
mavlink_status_t mavlinkStatus;
int8_t mavlinkFlag = 0;
mavlink_optical_flow_t opticalFlowData;
int8_t opticalFlowDataFlag = 0;

//px4flow values
volatile float groundDistance = 0;
volatile float elevatorSpeed = 0;
volatile float aileronSpeed = 0;
volatile uint8_t px4Confidence = 0;

//vars for estimators
volatile float estimatedElevatorPos = 0;
volatile float estimatedAileronPos  = 0;
volatile float estimatedThrottlePos = 0;
volatile float estimatedElevatorVel = 0;
volatile float estimatedAileronVel  = 0;
volatile float estimatedThrottleVel = 0;

//vars for controllers
volatile float elevatorIntegration = 0;
volatile float aileronIntegration  = 0;
volatile float throttleIntegration = 0;
volatile float elevatorSetpoint = (ELEVATOR_SP_LOW + ELEVATOR_SP_HIGH)/2;
volatile float aileronSetpoint  = (AILERON_SP_LOW  + AILERON_SP_HIGH )/2;
volatile float throttleSetpoint = (THROTTLE_SP_LOW + THROTTLE_SP_HIGH)/2;

//auto-trajectory vars
volatile unsigned char trajectoryEnabled = 0;
volatile float trajTimer = 0;
volatile int trajIndex = -1;
volatile trajectoryPoint_t trajectory[TRAJECTORY_LENGTH];

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
volatile float elevatorGumstix = 0;
volatile float aileronGumstix = 0;
volatile float throttleGumstix = 0;
volatile int8_t validGumstix = 0;
volatile int8_t gumstixDataFlag = 0;
volatile unsigned char gumstixParseCharCrc = 0;

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

//~ --------------------------------------------------------------------
//~ write debug message to Uart0 serial output
//~ --------------------------------------------------------------------

#if LOGGING_ON == ENABLED

void debug() {

	char num[20];
	static uint8_t debug_cycle = 0;

	static float   dbg_elevatorPos_raw;
	static float   dbg_elevatorPos;
	static float   dbg_elevatorVel_raw;
	static float   dbg_elevatorVel;
	static float   dbg_elevator_sp;
	static int16_t dbg_elevator_man;
	static int16_t dbg_elevator_auto;
	static float   dbg_elevator_int;
	
	static float   dbg_aileronPos_raw;
	static float   dbg_aileronPos;
	static float   dbg_aileronVel_raw;
	static float   dbg_aileronVel;
	static float   dbg_aileron_sp;
	static int16_t dbg_aileron_man;
	static int16_t dbg_aileron_auto;
	static float   dbg_aileron_int;

	static int8_t  dbg_gumstix_valid;
	static uint8_t dbg_px4flow_confidence;
	static uint8_t dbg_position_enabled;

	//perform 3 debug cycles per 10 values
	if(++debug_cycle >= 3) debug_cycle = 0;

	if(debug_cycle == 0) {

		//Time and main cycles per controller cycle
		sprintf(num, "%.5f", ((double) timeStamp)); //1 TimeStamp
		Uart0_write_string(num, strlen(num));
		Uart0_write_char(' ');

		Uart0_write_num(main_cycle/3); //2 Main cycles
		Uart0_write_char(' ');
		main_cycle = 0;

		//Throttle values
		Uart0_write_num(groundDistance*1000); //3 PX4 raw altitude (from ground)
		Uart0_write_char(' ');

		Uart0_write_num(estimatedThrottlePos*1000); //4 Estimated TH position
		Uart0_write_char(' ');

		Uart0_write_num(throttleGumstix*1000); //5 Gumstix raw altitude (from blob)
		Uart0_write_char(' ');

		Uart0_write_num(estimatedThrottleVel*1000); //6 Estimated TH velocity
		Uart0_write_char(' ');

		Uart0_write_num(throttleSetpoint*1000); //7 TH setpoint
		Uart0_write_char(' ');

		Uart0_write_num(RCchannel[THROTTLE]); //8 TH manual
		Uart0_write_char(' ');

		Uart0_write_num(controllerThrottleOutput * controllerEnabled); //9 TH controller
		Uart0_write_char(' ');

		Uart0_write_num(throttleIntegration * controllerEnabled); //10 TH integration
		Uart0_write_char(' ');

		//elevator snapshot
		dbg_elevatorPos_raw = elevatorGumstix;
		dbg_elevatorPos     = estimatedElevatorPos;
		dbg_elevatorVel_raw = elevatorSpeed;
		dbg_elevatorVel     = estimatedElevatorVel;
		dbg_elevator_sp     = elevatorSetpoint;
		dbg_elevator_man    = RCchannel[ELEVATOR];
		dbg_elevator_auto   = controllerElevatorOutput * controllerEnabled;
		dbg_elevator_int    = elevatorIntegration * controllerEnabled;
		
		//aileron snapshot
		dbg_aileronPos_raw = aileronGumstix;
		dbg_aileronPos     = estimatedAileronPos;
		dbg_aileronVel_raw = aileronSpeed;
		dbg_aileronVel     = estimatedAileronVel;
		dbg_aileron_sp     = aileronSetpoint;
		dbg_aileron_man    = RCchannel[AILERON];
		dbg_aileron_auto   = controllerAileronOutput * controllerEnabled;
		dbg_aileron_int    = aileronIntegration * controllerEnabled;

		//other values snapshot
		dbg_gumstix_valid      = validGumstix;
		dbg_px4flow_confidence = px4Confidence;
		dbg_position_enabled   = positionControllerEnabled;

	} else if(debug_cycle == 1)  {

		//gumstix and px4flow confidences
		Uart0_write_num(dbg_gumstix_valid); //11 Gumstix vaid
		Uart0_write_char(' ');

		Uart0_write_num(dbg_px4flow_confidence); //12 XP4 confidence
		Uart0_write_char(' ');

		//elevator values
		Uart0_write_num(dbg_elevatorPos_raw*1000); //13 EL raw gumstix position
		Uart0_write_char(' ');

		Uart0_write_num(dbg_elevatorPos*1000); //14 EL estimated position
		Uart0_write_char(' ');

		Uart0_write_num(dbg_elevatorVel_raw*1000); //15 EL PX4 raw velocity
		Uart0_write_char(' ');

		Uart0_write_num(dbg_elevatorVel*1000); //16 EL estimated velocity
		Uart0_write_char(' ');

		Uart0_write_num(dbg_elevator_sp*1000); //17 EL setpoint
		Uart0_write_char(' ');

		Uart0_write_num(dbg_elevator_man); //18 EL manual
		Uart0_write_char(' ');

		Uart0_write_num(dbg_elevator_auto); //19 EL controller
		Uart0_write_char(' ');

		Uart0_write_num(dbg_elevator_int); //20 EL integration
		Uart0_write_char(' ');

	} else if(debug_cycle == 2)  {

		//position enabled
		Uart0_write_num(dbg_position_enabled); //21 Position enabled
		Uart0_write_char(' ');

		//sprintf(num, "%i", ((int16_t) 0)); //22 (nothing)
		//Uart0_write_string(num, strlen(num));
		Uart0_write_char('0');
		Uart0_write_char(' ');

		//aileron values
		Uart0_write_num(dbg_aileronPos_raw*1000); //23 AIL raw gumstix position
		Uart0_write_char(' ');

		Uart0_write_num(dbg_aileronPos*1000); //24 AIL estimated position
		Uart0_write_char(' ');

		Uart0_write_num(dbg_aileronVel_raw*1000); //25 AIL PX4 raw velocity
		Uart0_write_char(' ');

		Uart0_write_num(dbg_aileronVel*1000); //26 AIL estimated velocity
		Uart0_write_char(' ');

		Uart0_write_num( dbg_aileron_sp*1000); //27 AIL setpoint
		Uart0_write_char(' ');

		Uart0_write_num(dbg_aileron_man); //28 AIL manual
		Uart0_write_char(' ');

		Uart0_write_num(dbg_aileron_auto); //29 AIL controller
		Uart0_write_char(' ');

		Uart0_write_num(dbg_aileron_int); //30 AIL integration
		Uart0_write_char(' ');

		//end of line
		Uart0_write_char('\n');

	}

}

#endif // LOGGING_ON == ENABLED

void writeTrajectory1(){

	TRAJ_POINT(0,  2, -1000,    0);
	TRAJ_POINT(1,  4, -1500,    0);

	TRAJ_POINT(2,  6, -1500, -500);
	TRAJ_POINT(3,  8, -1500,    0);

	TRAJ_POINT(4, 10, -2000,    0);
	TRAJ_POINT(5, 12, -1500,    0);

	TRAJ_POINT(6, 14, -1500, +500);
	TRAJ_POINT(7, 16, -1500,    0);

	TRAJ_POINT(8,999, -1500,    0);
}

int main() {

	// initialize the MCU (timers, uarts and so on)
	initializeMCU();

	writeTrajectory1();

	// the main while cycle
	while (1) {

		main_cycle++;

		// runs controllers
		if (controllersFlag == 1) {

#if PX4FLOW_DATA_RECEIVE == ENABLED

			//If controllerEnabled == 0 the controller output signals
			//are "unplugged" (in mergeSignalsToOutput) but the
			//controllers keep running.
			//When the controllers are turned on, it's integral actions
			//are reset (in enableController).

			positionEstimator();
			altitudeEstimator();

			setpoints();

			positionController();
			altitudeController();

#endif // PX4FLOW_DATA_RECEIVE == ENABLED

#if LOGGING_ON == ENABLED

			debug();

#endif
			controllersFlag = 0;
		}

		// PWM input capture
		capturePWMInput();

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

		// landing on/off
		if (RCchannel[AUX4] < (PULSE_MIDDLE - 200)) {
			landingMode = 1;
		} else if(RCchannel[AUX4] > (PULSE_MIDDLE + 200)) {
			if(!trajectoryEnabled){
				trajectoryEnabled = 1;
				trajIndex = -1;
				trajTimer = 0;
			}
		}else{
			landingMode = 0;
			trajectoryEnabled = 0;
		}

		// load the constant values from the RC
		// <0; 1>
		constant1 = ((float)(RCchannel[AUX1] - PULSE_MIN))/(PULSE_MAX - PULSE_MIN);
		if(constant1 > 1) constant1 = 1;
		if(constant1 < 0) constant1 = 0;

		constant2 = ((float)(RCchannel[AUX2] - PULSE_MIN))/(PULSE_MAX - PULSE_MIN);
		if(constant2 > 1) constant2 = 1;
		if(constant2 < 0) constant2 = 0;

		constant5 = ((float)(RCchannel[AUX5] - PULSE_MIN))/(PULSE_MAX - PULSE_MIN);
		if(constant5 > 1) constant5 = 1;
		if(constant5 < 0) constant5 = 0;

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

		// receive px4flow data
		if (opticalFlowDataFlag == 1) {

			// decode the message (there will be new values in opticalFlowData...
			mavlink_msg_optical_flow_decode(&mavlinkMessage, &opticalFlowData);

			//px4flow returns velocities in m/s and gnd. distance in m
			// +elevator = front
			// +aileron  = left
			// +throttle = up

#if PX4_CAMERA_ORIENTATION == FORWARD

			elevatorSpeed = + opticalFlowData.flow_comp_m_x;
			aileronSpeed  = - opticalFlowData.flow_comp_m_y;

#elif PX4_CAMERA_ORIENTATION == BACKWARD

			elevatorSpeed = - opticalFlowData.flow_comp_m_x;
			aileronSpeed  = + opticalFlowData.flow_comp_m_y;

#endif

			if (opticalFlowData.ground_distance < ALTITUDE_MAXIMUM &&
			    opticalFlowData.ground_distance > 0.3) {

				groundDistance = opticalFlowData.ground_distance;
			}

			px4Confidence = opticalFlowData.quality;

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

		//receive gumstix data
		if (gumstixDataFlag == 1) {

			if (validGumstix == 1) {

			//Gumstix returns position of the blob relative to camera
			//in milimeters, we want position of the drone relative
			//to the blob in meters.
			// +elevator = front
			// +aileron  = left
			// +throttle = up

#if GUMSTIX_CAMERA_POINTING == FORWARD //camera led on up side

				elevatorGumstix = - (float)xPosGumstixNew / 1000;
				aileronGumstix  = - (float)zPosGumstixNew / 1000;
				throttleGumstix = + (float)yPosGumstixNew / 1000;

#elif GUMSTIX_CAMERA_POINTING == DOWNWARD //camera led on front side

				elevatorGumstix = + (float)yPosGumstixNew / 1000;
				aileronGumstix  = - (float)zPosGumstixNew / 1000;
				throttleGumstix = + (float)xPosGumstixNew / 1000;

#endif

			}

			gumstixDataFlag = 0;
		}

#endif

		// set outputs signals for MK only if it should be
		mergeSignalsToOutput();

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

	timeStamp += DT;
	controllersFlag = 1;
}

