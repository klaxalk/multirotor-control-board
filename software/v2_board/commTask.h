/*
 * communicationTask.h
 *
 * Created: 11.9.2014 11:17:48
 *  Author: Tomas Baca
 */ 

#ifndef COMMTASK_H_
#define COMMTASK_H_

volatile int16_t pitchAngle;
volatile int16_t rollAngle;

// the communication task
void commTask(void *p);

#endif /* COMMTASK_H_ */