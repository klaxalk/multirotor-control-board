/*
 * communicationTask.h
 *
 * Created: 11.9.2014 11:17:48
 *  Author: Tomas Baca
 */ 

#ifndef COMMTASK_H_
#define COMMTASK_H_

// the communication task
void commTask(void *p);

enum main2commMessageType {CLEAR_STATES};

typedef struct {
	
	enum main2commMessageType messageType;
} main2commMessage_t;

#endif /* COMMTASK_H_ */