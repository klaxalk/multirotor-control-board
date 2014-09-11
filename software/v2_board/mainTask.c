/*
 * mainTask.c
 *
 * Created: 11.9.2014 11:05:32
 *  Author: Tomas Baca
 */ 

#include "mainTask.h"
#include "system.h"

void mainTask(void *p) {
	
	while (1) {

		mergeSignalsToOutput();
	}
}