/*
 * communicationTask.h
 *
 * Created: 11.9.2014 11:17:48
 *  Author: Tomas Baca
 */ 

#ifndef COMMTASK_H_
#define COMMTASK_H_

/* -------------------------------------------------------------------- */
/*	receiver from STM													*/
/* -------------------------------------------------------------------- */

#define STM_BUFFER_SIZE	256

/* -------------------------------------------------------------------- */
/*	message from mainTask to commTask									*/
/* -------------------------------------------------------------------- */

enum main2commMessageType_t {CLEAR_STATES};

typedef struct {
	
	enum main2commMessageType_t messageType;
} main2commMessage_t;

/* -------------------------------------------------------------------- */
/*	MPC support variables	 											*/
/* -------------------------------------------------------------------- */
volatile int16_t mpcCounter;
volatile int16_t mpcRate;

/* -------------------------------------------------------------------- */
/*	kalman support variables	 										*/
/* -------------------------------------------------------------------- */
volatile int16_t kalmanCounter;
volatile int16_t kalmanRate;

/* -------------------------------------------------------------------- */
/*	methods in commTask													*/
/* -------------------------------------------------------------------- */

// the communication task
void commTask(void *p);

#endif /* COMMTASK_H_ */