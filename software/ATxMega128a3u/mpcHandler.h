/*
 * mpcHandler.h
 *
 * Created: 10.2.2015 10:46:34
 *  Author: Tomas Baca
 */ 

#ifndef MPCHANDLER_H_
#define MPCHANDLER_H_

#include "system.h"

#define STM_BUFFER_SIZE	256
#define MPC_SATURATION	1000

/* -------------------------------------------------------------------- */
/*	variables that supports MPC											*/
/* -------------------------------------------------------------------- */

// this structure holds a message received from STM
typedef struct {

	char messageId;
	char * messageBuffer;
} stmMessageHandler_t;

// this structure hold setpoints for elevator and aileron position, mainly for debugging
typedef struct {
	
	float elevator;
	float aileron;
} mpcSetpoints_t;

volatile int16_t mpcElevatorOutput;
volatile int16_t mpcAileronOutput;

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
 * @brief read a float value from a message starting at indexFrom
 *
 * @param message pointer to a buffer with the message
 * @param index in the buffer where the value starts
 */
float readFloat(char * message, int * indexFrom);

/**
 * @brief read a int16_t value from a message starting at indexFrom
 *
 * @param message pointer to a buffer with the message
 * @param index in the buffer where the value starts
 */
int16_t readInt16(char * message, int * indexFrom);

/**
 * @brief read a uint32_t value from a message starting at indexFrom
 *
 * @param message pointer to a buffer with the message
 * @param index in the buffer where the value starts
 */
uint32_t readUint32(char * message, int * indexFrom);

/**
 * @brief read a char value from a message starting at indexFrom
 *
 * @param message pointer to a buffer with the message
 * @param index in the buffer where the value starts
 */
char readChar(char * message, int * indexFrom);

/**
 * @brief send a float value to a usartBuffer, incrementing crc
 * 
 * @param usartBuffer where to send the data
 * @param var variable to send
 * @param crc variable to accumulate the control summ
 */
void sendFloat(UsartBuffer * usartBuffer, const float var, char * crc);

/**
 * @brief send a int16_t value to a usartBuffer, incrementing crc
 *
 * @param usartBuffer where to send the data
 * @param var variable to send
 * @param crc variable to accumulate the control summ
 */
void sendInt16(UsartBuffer * usartBuffer, const int16_t var, char * crc);

/**
 * @brief send a char value to a usartBuffer, incrementing crc
 *
 * @param usartBuffer where to send the data
 * @param var variable to send
 * @param crc variable to accumulate the control summ
 */
void sendChar(UsartBuffer * usartBuffer, const char var, char * crc);

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
 * @brief send setpoint for the whole optimized horizon (constant setpoint)
 * @param elevatorSetpoint desired elevator position [m], + means forwards
 * @param aileronSetpoint desired aileron position [m], + means leftwards
 */
void stmSendSetpoint(float elevatorSetpoint, float aileronSetpoint);

/**
 * @brief send trajectory (2.2s) by means of 5 points equally distributed from the start to the end of the optimization horizon
 * @param elevatorTrajectory 5 points from the elevator trajectory
 * @param aileronTrajectory 5 points from the aileron trajectory
 */
void stmSendTrajectory(float elevatorTrajectory[5], float aileronTrajectory[5]);

/**
 * @brief initialize the LOCAL copy of kalman states to ZERO
 */
void initializeKalmanStates(void);

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

/**
 * @brief Set Kalman's integrated position to particular values
 * 
 * @param elevatorPos the value of ElevatorPosition
 * @param aileronPos the value of AileronPosition
 */
void stmSetKalmanPosition(float elevatorPos, float aileronPos);

#endif /* MPCHANDLER_H_ */