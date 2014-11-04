/*
 * mainTask.h
 *
 * Created: 11.9.2014 11:05:51
 *  Author: Tomas Baca
 */ 


#ifndef MAINTASK_H_
#define MAINTASK_H_

// constants = AUX channels from the RC transmitter
extern volatile float constant1; // TODO: better names for constants
extern volatile float constant2;
extern volatile float constant5;

// the main task routine
void mainTask(void *p);

#endif /* MAINTASK_H_ */
