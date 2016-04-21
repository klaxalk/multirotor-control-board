/*
 * communicationTask.h
 *
 * Created: 11.9.2014 11:17:48
 *  Author: Tomas Baca
 */ 

#ifndef COMMTASK_H_
#define COMMTASK_H_

volatile uint16_t commTaskRate;
volatile uint16_t commTaskCounter;

/* -------------------------------------------------------------------- */
/*	message from mainTask to commTask									*/
/* -------------------------------------------------------------------- */

enum main2commMessageType_t {CLEAR_STATES, SET_SETPOINT, SET_TRAJECTORY};

typedef struct {
	
	float elevator;
	float aileron;
} simpleSetpoint_t;

typedef struct {
	
	float elevatorTrajectory[5];
	float aileronTrajectory[5];
} trajectorySetpoint_t;

typedef union {
	
	simpleSetpoint_t simpleSetpoint;
	trajectorySetpoint_t trajectory;
} main2commMessagePayload;

typedef struct {
	
	enum main2commMessageType_t messageType;
	main2commMessagePayload data;
} main2commMessage_t;

int xbeeflag;

/* -------------------------------------------------------------------- */
/*	Time stamp from Matlab												*/
/* -------------------------------------------------------------------- */

volatile uint32_t timeStamp;

/* -------------------------------------------------------------------- */
/*	Variables for measuring MPC and kalman rates						*/
/* -------------------------------------------------------------------- */
volatile int16_t mpcCounter;
volatile int16_t mpcRate;
volatile int16_t kalmanCounter;
volatile int16_t kalmanRate;

/* -------------------------------------------------------------------- */
/*	For px4flow															*/
/* -------------------------------------------------------------------- */

#define PX4FLOW_SPEED_SATURATION	3.0

/* -------------------------------------------------------------------- */
/*	methods in commTask													*/
/* -------------------------------------------------------------------- */

// the communication task
void commTask(void *p);

#endif /* COMMTASK_H_ */