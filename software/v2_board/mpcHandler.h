/*
 * mpcHandler.h
 *
 * Created: 10.2.2015 10:46:34
 *  Author: klaxalk
 */ 

#ifndef MPCHANDLER_H_
#define MPCHANDLER_H_

#include "system.h"

#define STM_BUFFER_SIZE	256

typedef struct {

	char messageId;
	char * messageBuffer;
} stmMessageHandler_t;

/* -------------------------------------------------------------------- */
/*	variables that supports MPC											*/
/* -------------------------------------------------------------------- */

#define MPC_SATURATION	800

volatile int16_t mpcElevatorOutput;
volatile int16_t mpcAileronOutput;

// this structure hold setpoints for elevator and aileron position
typedef struct {
	
	float elevatorSetpoint;
	float aileronSetpoint;
} mpcSetpoints_t;

volatile mpcSetpoints_t mpcSetpoints;

/* -------------------------------------------------------------------- */
/*	variables that support kalman filter								*/
/* -------------------------------------------------------------------- */

// this struct contains variables for elevator&aileron axis
typedef struct {
	
	volatile float position;
	volatile float velocity;
	volatile float acceleration;
	volatile float acceleration_input;
	volatile float acceleration_error;
	
} oneAxisStates_t;

// this struct contains states computed by kalman filter in STM MCU
typedef struct {

	oneAxisStates_t elevator;
	oneAxisStates_t aileron;
	
} kalmanStates_t;

volatile kalmanStates_t kalmanStates;

/**
 * @brief send a px4flow measurement to STM together with currently used controller actions
 * 
 * @param elevSpeed Elevator speed
 * @param ailerSpeed Aileron speed
 * @param elevInput Elevator controller input
 * @param aileInput	Aileron controller input
 */
void stmSendMeasurement(float elevSpeed, float aileSpeed, int16_t elevInput, int16_t aileInput);

/**
 * @brief send basic point setpoints to MPC
 */
void stmSendSetpointsSimple();

/**
 * @brief initialize the local kalman states to ZERO
 */
void initializeKalmanStates();

/**
 * @brief parse an incoming char from the STM u-controller
 * 
 * @return 1 if the whole message is completed, 0 otherwise
 *
 * @param inChar incoming character from uart
 * @param messageHandler a returning structure with the messageId and *message
 */
int8_t stmParseChar(char inChar, stmMessageHandler_t * messageHandler);

/**
 * @brief Reset the kalman states (in STM) and set initial position
 * 
 * @param initElevator the value of ElevatorPosition after reset
 * @param initAileron the value of AileronPosition after reset
 */
void stmResetKalman(float initElevator, float initAileron);

#endif /* MPCHANDLER_H_ */