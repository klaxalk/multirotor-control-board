/*
 * mainTask.c
 *
 * Created: 11.9.2014 11:05:32
 *  Author: Tomas Baca
 */ 

#include "mainTask.h"
#include "system.h"
#include "controllers.h"
#include "communication.h"
#include "commTask.h"
#include <stdio.h> // sprintf
#include <stdlib.h> // abs

// timestamp for debug and logging
volatile double timeStamp = 0;
volatile uint16_t main_cycle = 0;

// for on-off by AUX3 channel
int8_t previous_AUX3 = 0;

void mainTask(void *p) {

	main2commMessage_t main2commMessage;
	main2commMessage.messageType = CLEAR_STATES;
	xQueueSend(main2commsQueue, &main2commMessage, 0);

	vTaskDelay(50);
		
	while (1) {
		
		main_cycle++;
		
		// controller on/off
		if (abs(RCchannel[AUX1] - PPM_IN_MIDDLE_LENGTH) < 500) {
		
			if (previous_AUX3 == 0) {
				enableAltitudeController();
			}
		
			disableMpcController();
			previous_AUX3 = 1;
			
		} else if (RCchannel[AUX1] > (PPM_IN_MIDDLE_LENGTH + 500)) {
			
			if (previous_AUX3 == 1) {
				
				main2commMessage.messageType = CLEAR_STATES;
				xQueueSend(main2commsQueue, &main2commMessage, 0);
				
				vTaskDelay(50);
				
				enableMpcController();
			}
		
			previous_AUX3 = 2;
		
		} else {
		
			disableAltitudeController();
			disableMpcController();
			previous_AUX3 = 0;
		}
	}
}