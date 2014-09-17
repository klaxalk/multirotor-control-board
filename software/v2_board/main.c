/*
* main.c
*
* Created: 24.8.2014 15:10:04
*  Author: Tomas Baca
*/

#include "ioport.h"
#include "twi_master_driver.h"
#include "twi_slave_driver.h"

// basic system functions
#include "system.h"
#include "communication.h"
#include "usart_driver.h"

// for sprintf
#include <stdio.h>

/*! Defining an example slave address. */
#define SLAVE_ADDRESS    0x55

/*! Defining number of bytes in buffer. */
#define NUM_BYTES        8

/*! CPU speed 2MHz, BAUDRATE 100kHz and Baudrate Register Settings */
#define CPU_SPEED	32000000UL
#define BAUDRATE	100000
#define TWI_BAUDSETTING TWI_BAUD(CPU_SPEED, BAUDRATE)

/* -------------------------------------------------------------------- */
/*	Contains signals from RC receiver									*/
/* -------------------------------------------------------------------- */
extern volatile uint16_t RCchannel[9];

extern USART_data_t usart_xbee;
extern USART_data_t usart_1;

// constants from RC transmitter
volatile float constant1 = 0;
volatile float constant2 = 0;
volatile float constant5 = 0;

/* Global variables */
TWI_Master_t twiMaster;    /*!< TWI master module. */
TWI_Slave_t twiSlave;      /*!< TWI slave module. */

extern volatile uint16_t RCchannel[9];

extern volatile float groundDistance;
extern volatile float elevatorSpeed;
extern volatile float aileronSpeed;
extern volatile uint8_t px4Confidence;

extern volatile float estimatedElevatorPos;
extern volatile float estimatedAileronPos;

// variables used by the mavlink library
extern mavlink_message_t mavlinkMessage;
extern mavlink_status_t mavlinkStatus;
extern int8_t mavlinkFlag;
extern mavlink_optical_flow_t opticalFlowData;
extern int8_t opticalFlowDataFlag;

// flag to run the controllers
volatile int8_t controllersFlag = 0;

// timestamp for debug and logging
volatile double timeStamp = 0;
volatile uint16_t main_cycle = 0;

// for on-off by AUX3 channel
volatile int8_t previous_AUX3 = 0;

//auto-trajectory variables
volatile unsigned char trajectoryEnabled = 0;
volatile float trajTimer = 0;
volatile int trajIndex = -1;
volatile trajectoryPoint_t trajectory[TRAJECTORY_LENGTH];

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
//~ volatile float elevatorSetpoint = (ELEVATOR_SP_LOW + ELEVATOR_SP_HIGH)/2;
volatile float elevatorSetpoint = -1.5;
volatile float aileronSetpoint  = (AILERON_SP_LOW  + AILERON_SP_HIGH )/2;
//~ volatile float throttleSetpoint = (THROTTLE_SP_LOW + THROTTLE_SP_HIGH)/2;
volatile float throttleSetpoint = 0.75;

// controller on/off
volatile unsigned char controllerEnabled = 0;
volatile unsigned char positionControllerEnabled = 0;

//auto-landing variables
volatile unsigned char landingRequest = 0;
volatile unsigned char landingState = LS_ON_GROUND;
volatile uint8_t landingCounter = 0;

#if GUMSTIX_DATA_RECEIVE == ENABLED

/* -------------------------------------------------------------------- */
/*	Variables for gumstix												*/
/* -------------------------------------------------------------------- */
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

#if TRAJECTORY_FOLLOWING == ENABLED

void writeTrajectory1(){

	//TRAJ_POINT(0,  0, -1500,     0);

	// (i, time (s), x (+ forward), y (+ leftward), z (altitude))

	//~ Square
	//~ TRAJ_POINT(0,  8,  -1500,  -300, 750);
	//~ TRAJ_POINT(1,  16,  -2100,  -300, 750);
	//~ TRAJ_POINT(2,  24,  -2100,  +300, 750);
	//~ TRAJ_POINT(3,  32,  -1500,  +300, 750);
	//~ TRAJ_POINT(4,  40,  -1500,  0, 750);
	
	//~ Zuzeni Follower Experiment Telocvicna
	//~  Puvodni*3 - 0.5
	//~ TRAJ_POINT(0,  3,  -1900,  0, 1000);
	//~ TRAJ_POINT(1,  6,  -1902,  0, 1000);
	//~ TRAJ_POINT(2,  9,  -1660,  0, 1000);
	//~ TRAJ_POINT(3,  12,  -1136,  0, 1000);
	//~ TRAJ_POINT(4,  15,  -1380,  0, 1000);
	//~ TRAJ_POINT(5,  18,  -1993,  0, 1000);
	//~ TRAJ_POINT(6,  21,  -2063,  0, 1000);
	//~ TRAJ_POINT(7,  24,  -1897,  0, 1000);
	//~ TRAJ_POINT(8,  27,  -1826,  0, 1000);
	//~ TRAJ_POINT(9,  30,  -1846,  0, 1000);

	//~ Zuzeni LEADER Experiment Telocvicna
	//~  Aileron: Puvodni*3 - 0.5
	//~ TRAJ_POINT(0,  3,  -900,  0, 1000);
	//~ TRAJ_POINT(1,  6,  -300,  0, 1000);
	//~ TRAJ_POINT(2,  9,  +300,  0, 1000);
	//~ TRAJ_POINT(3,  12,  +900,  0, 1000);
	//~ TRAJ_POINT(4,  15,  +1500,  0, 1000);
	//~ TRAJ_POINT(5,  18,  +2100,  0, 1000);
	//~ TRAJ_POINT(6,  21,  +2700,  0, 1000);
	//~ TRAJ_POINT(7,  24,  +3300,  0, 1000);
	//~ TRAJ_POINT(8,  27,  +3900,  0, 1000);
	//~ TRAJ_POINT(9,  30,  -1500,  0, 1000);

	//~ Preplanovani LEADER Experiment Telocvicna
	//~  Aileron: Pùvodní*2
	TRAJ_POINT(0,  3,  -900,  0, 1000);
	TRAJ_POINT(1,  6,  -300,  0, 1000);
	TRAJ_POINT(2,  9,  +300,  0, 1000);
	TRAJ_POINT(3,  12,  +900,  0, 1000);
	TRAJ_POINT(4,  15,  +1500,  0, 1000);
	TRAJ_POINT(5,  18,  +2100,  0, 1000);
	TRAJ_POINT(6,  21,  +2700,  0, 1000);
	TRAJ_POINT(7,  24,  +3300,  0, 1000);
	TRAJ_POINT(8,  27,  +3900,  0, 1000);
	TRAJ_POINT(9,  30,  -1500,  0, 1000);
	
	//~ Experimetn TV
	//~ TRAJ_POINT(0,  5,  -500,  -50, 1000);
	//~ TRAJ_POINT(1, 10,  +500, -360, 1120);
	//~ TRAJ_POINT(2, 15, +1500, -740, 1250);
	//~ TRAJ_POINT(3, 20, +2500, -880,  980);
	//~ TRAJ_POINT(4, 25, +3500, -820,  690);
	//~
	//~ TRAJ_POINT(5, 30, +4500, -550,  630);
	//~ TRAJ_POINT(6, 35, +5500, -150,  690);
	//~ TRAJ_POINT(7, 40, +6500, +280,  790);
	//~ TRAJ_POINT(8, 45, +7500, +220,  930);
	//~ TRAJ_POINT(9, 50, +8500,    0, 1000);

	//TRAJ_POINT(9,999, 0, -1500,     0);

}

#endif //TRAJECTORY_FOLLOWING == ENABLED

// this function processes received data on the I2C Slave line
// It is call by the I2C driver
void TWIE_SlaveProcessData(void) {
	
	// takes the received character, increments it's value and sends it back
	twiSlave.sendData[0] = twiSlave.receivedData[0]+1;
}

/*! TWIC Master Interrupt vector. */
ISR(TWIC_TWIM_vect)
{
	TWI_MasterInterruptHandler(&twiMaster);
}

/*! TWIE Slave Interrupt vector. */
ISR(TWIE_TWIS_vect)
{
	TWI_SlaveInterruptHandler(&twiSlave);
}

/*
This task shows the example of TWI (I2C) master and slave. There are two TWI interfaces set up, one as the
master, one as the slave. The task receives characters from the USART, sends it to the TWI slave. The slave takes
the characters, increments their value and returns them to the master. Then they are returned through the USART.
*/
void twi_loopback(void *p) {
	
	// Initialize TWI master on PORTC
	TWI_MasterInit(&twiMaster, &TWIC, TWI_MASTER_INTLVL_LO_gc, TWI_BAUDSETTING);

	// Initialize TWI slave on PORTE
	TWI_SlaveInitializeDriver(&twiSlave, &TWIE, TWIE_SlaveProcessData);
	TWI_SlaveInitializeModule(&twiSlave, SLAVE_ADDRESS, TWI_SLAVE_INTLVL_LO_gc);

	// Enable LO interrupt level (needed without the FreeRTOS)
	// PMIC.CTRL |= PMIC_LOLVLEN_bm;

	// turn on USART on port
	unsigned char inChar;

	while (1) {
		
		// wait for some characters in USART input buffer
		//!!! while (!usartBufferGetByte(usart_buffer_4, &inChar, 0)) {}
		
		// send the character to I2C Slave
		TWI_MasterWriteRead(&twiMaster, SLAVE_ADDRESS, &inChar, 1, 1);
		
		// Wait until transaction is complete
		while (twiMaster.status != TWIM_STATUS_READY) {}
		
		// send the received character back to the USART
		//!!! USART_TXBuffer_PutByte(usart_buffer_4, twiMaster.readData[0]);
	}
}

int main(void)
{
		
	boardInit();
	USART_XBEE_init();
	USART_1_init();
	char buffer[20];
	
	unsigned char inChar;
	char* ukazatel;
	
	#if TRAJECTORY_FOLLOWING == ENABLED
	writeTrajectory1();
	#endif
	
	while (1) {
		
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

			landingStateAutomat();

			positionController();
			altitudeController();

			#endif // PX4FLOW_DATA_RECEIVE ==
			
			led_orange_toggle();
			
			controllersFlag = 0;
		}
		
		main_cycle++;
		
		// controller on/off
		if (abs(RCchannel[AUX3] - PPM_IN_MIDDLE_LENGTH) < 200) {
			if (previous_AUX3 == 0) {
				enableController();
			}
			disablePositionController();
			previous_AUX3 = 1;
			} else if (RCchannel[AUX3] > (PPM_IN_MIDDLE_LENGTH + 200)) {
			if (previous_AUX3 == 1) {
				enablePositionController();
			}
			previous_AUX3 = 2;
			} else {
			disableController();
			disablePositionController();
			previous_AUX3 = 0;
		}
		
		#if PX4FLOW_DATA_RECEIVE == ENABLED

		// landing on/off, trajectory on/off
		if (RCchannel[AUX4] < (PPM_IN_MIDDLE_LENGTH - 200)) {
			landingRequest = 1;
			trajectoryEnabled = 0;
			} else if(RCchannel[AUX4] > (PPM_IN_MIDDLE_LENGTH + 200)) {
			landingRequest = 0;
			trajectoryEnabled = 1;
			}else{
			landingRequest = 0;
			trajectoryEnabled = 0;
		}

		#endif // PX4FLOW_DATA_RECEIVE == ENABLED

		// load the constant values from the RC
		// <0; 1>
		constant1 = ((float)(RCchannel[AUX1] - PPM_IN_MIN_LENGTH))/(PPM_IN_MAX_LENGTH - PPM_IN_MIN_LENGTH);
		if(constant1 > 1) constant1 = 1;
		if(constant1 < 0) constant1 = 0;

		constant2 = ((float)(RCchannel[AUX2] - PPM_IN_MIN_LENGTH))/(PPM_IN_MAX_LENGTH - PPM_IN_MIN_LENGTH);
		if(constant2 > 1) constant2 = 1;
		if(constant2 < 0) constant2 = 0;

		constant5 = ((float)(RCchannel[AUX5] - PPM_IN_MIN_LENGTH))/(PPM_IN_MAX_LENGTH - PPM_IN_MIN_LENGTH);
		if(constant5 > 1) constant5 = 1;
		if(constant5 < 0) constant5 = 0;
		
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

				//saturation
				if(xPosGumstixNew > 2*POSITION_MAXIMUM) xPosGumstixNew = 2*POSITION_MAXIMUM;
				if(xPosGumstixNew < 0) xPosGumstixNew = 0; //distance from the blob (positive)

				if(yPosGumstixNew > +POSITION_MAXIMUM) yPosGumstixNew = +POSITION_MAXIMUM;
				if(yPosGumstixNew < -POSITION_MAXIMUM) yPosGumstixNew = -POSITION_MAXIMUM;

				if(zPosGumstixNew > +POSITION_MAXIMUM) zPosGumstixNew = +POSITION_MAXIMUM;
				if(zPosGumstixNew < -POSITION_MAXIMUM) zPosGumstixNew = -POSITION_MAXIMUM;

				#if GUMSTIX_CAMERA_POINTING == FORWARD //camera led on up side

				//~ Camera pointing forward and being PORTRAIT oriented
				//~ elevatorGumstix = - (float)xPosGumstixNew / 1000;
				//~ aileronGumstix  = - (float)zPosGumstixNew / 1000;
				//~ throttleGumstix = + (float)yPosGumstixNew / 1000;

				//~ Camera pointing forward and being LANDSCAPE oriented
				elevatorGumstix = - (float) xPosGumstixNew / 1000;
				aileronGumstix  = - (float) yPosGumstixNew / 1000;
				throttleGumstix = - (float) zPosGumstixNew / 1000;

				#elif GUMSTIX_CAMERA_POINTING == DOWNWARD //camera led on front side

				elevatorGumstix = + (float) yPosGumstixNew / 1000;
				aileronGumstix  = - (float) zPosGumstixNew / 1000;
				throttleGumstix = + (float) xPosGumstixNew / 1000;

				#endif

			}

			gumstixDataFlag = 0;
		}

		#endif
		
		// check the xbee rc buffer
		if (USART_RXBufferData_Available(&usart_xbee)) {
			
			inChar = USART_RXBuffer_GetByte(&usart_xbee);
			
			if (inChar == 'b') {

				ukazatel = (char*) &estimatedElevatorPos;
				USART_TXBuffer_PutByte(&usart_xbee, *(ukazatel));
				USART_TXBuffer_PutByte(&usart_xbee, *(ukazatel+1));
				USART_TXBuffer_PutByte(&usart_xbee, *(ukazatel+2));
				USART_TXBuffer_PutByte(&usart_xbee, *(ukazatel+3));

				ukazatel = (char*) &estimatedAileronPos;
				USART_TXBuffer_PutByte(&usart_xbee, *(ukazatel));
				USART_TXBuffer_PutByte(&usart_xbee, *(ukazatel+1));
				USART_TXBuffer_PutByte(&usart_xbee, *(ukazatel+2));
				USART_TXBuffer_PutByte(&usart_xbee, *(ukazatel+3));

				ukazatel = (char*) &estimatedThrottlePos;
				USART_TXBuffer_PutByte(&usart_xbee, *(ukazatel));
				USART_TXBuffer_PutByte(&usart_xbee, *(ukazatel+1));
				USART_TXBuffer_PutByte(&usart_xbee, *(ukazatel+2));
				USART_TXBuffer_PutByte(&usart_xbee, *(ukazatel+3));

				ukazatel = (char*) &elevatorSetpoint;
				USART_TXBuffer_PutByte(&usart_xbee, *(ukazatel));
				USART_TXBuffer_PutByte(&usart_xbee, *(ukazatel+1));
				USART_TXBuffer_PutByte(&usart_xbee, *(ukazatel+2));
				USART_TXBuffer_PutByte(&usart_xbee, *(ukazatel+3));

				ukazatel = (char*) &aileronSetpoint;
				USART_TXBuffer_PutByte(&usart_xbee, *(ukazatel));
				USART_TXBuffer_PutByte(&usart_xbee, *(ukazatel+1));
				USART_TXBuffer_PutByte(&usart_xbee, *(ukazatel+2));
				USART_TXBuffer_PutByte(&usart_xbee, *(ukazatel+3));
	
				USART_TXBuffer_PutByte(&usart_xbee, controllerEnabled);
				USART_TXBuffer_PutByte(&usart_xbee, positionControllerEnabled);
				USART_TXBuffer_PutByte(&usart_xbee, landingRequest);
				USART_TXBuffer_PutByte(&usart_xbee, trajectoryEnabled);
	
				ukazatel = (char*) &elevatorSpeed;
				USART_TXBuffer_PutByte(&usart_xbee, *(ukazatel));
				USART_TXBuffer_PutByte(&usart_xbee, *(ukazatel+1));
				USART_TXBuffer_PutByte(&usart_xbee, *(ukazatel+2));
				USART_TXBuffer_PutByte(&usart_xbee, *(ukazatel+3));

				ukazatel = (char*) &aileronSpeed;
				USART_TXBuffer_PutByte(&usart_xbee, *(ukazatel));
				USART_TXBuffer_PutByte(&usart_xbee, *(ukazatel+1));
				USART_TXBuffer_PutByte(&usart_xbee, *(ukazatel+2));
				USART_TXBuffer_PutByte(&usart_xbee, *(ukazatel+3));

				USART_TXBuffer_PutByte(&usart_xbee, '\r');
				USART_TXBuffer_PutByte(&usart_xbee, '\n');
			}
		}
		
		// check the px4flow rx buffer
		if (USART_RXBufferData_Available(&usart_1)) {
		
			px4flowParseChar(USART_RXBuffer_GetByte(&usart_1));
		}
		
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

			opticalFlowDataFlag = 0;
		}
		
		mergeSignalsToOutput();
	}
	
	return 0;
}

