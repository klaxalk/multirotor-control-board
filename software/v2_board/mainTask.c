/*
 * mainTask.c
 *
 * Created: 11.9.2014 11:05:32
 *  Author: Tomas Baca
 */ 

#include "mainTask.h"
#include "system.h"
#include "controllers.h"
#include "commTask.h"
#include "mpcHandler.h"
#include "trajectories.h"

// for on-off by AUX3 channel
volatile int8_t AUX1_previous = 0;
volatile int8_t AUX2_previous = 0;
volatile int8_t setpointChangeTrigger = 0;
volatile int16_t currentSetpointIdx = 0;

void mainTask(void *p) {

	main2commMessage_t main2commMessage;
	main2commMessage.messageType = CLEAR_STATES;
	xQueueSend(main2commsQueue, &main2commMessage, 0);

	vTaskDelay(50);
	
	int16_t aux2filtered = PPM_IN_MIDDLE_LENGTH; 
		
	while (1) {
		
		// controller on/off
		if (abs(RCchannel[AUX1] - PPM_IN_MIDDLE_LENGTH) < 500) {
		
			if (AUX1_previous == 0) {
				enableAltitudeController();
			}
		
			disableMpcController();
			AUX1_previous = 1;
			
		} else if (RCchannel[AUX1] > (PPM_IN_MIDDLE_LENGTH + 500)) {
			
			if (AUX1_previous == 1) {
				
				main2commMessage.messageType = CLEAR_STATES;
				main2commMessage.data.simpleSetpoint.elevator = 0;
				main2commMessage.data.simpleSetpoint.aileron = 0;
				xQueueSend(main2commsQueue, &main2commMessage, 0);
				
				// wait between reseting the kalman and starting the controller
				vTaskDelay(50);
				
				enableMpcController();
			}
		
			AUX1_previous = 2;
		
		} else {
		
			disableAltitudeController();
			disableMpcController();
			AUX1_previous = 0;
		}
		
		// aux stick for setting the setpoint
		
		aux2filtered = aux2filtered*0.997 + RCchannel[AUX2]*0.003;
		
		if (auxSetpointFlag == 1) {
			
			if (setpointChangeTrigger++ == 11) {
				
				setpointChangeTrigger = 1;
		
				if ((aux2filtered > (PPM_IN_MIDDLE_LENGTH + 300)) && AUX2_previous != 1) {
			
					main2commMessage.messageType = SET_SETPOINT;
					main2commMessage.data.simpleSetpoint.elevator = 2;
					main2commMessage.data.simpleSetpoint.aileron = 0;
					xQueueSend(main2commsQueue, &main2commMessage, 0);
			
					AUX2_previous = 1;
					led_orange_off();
			
				} else if ((aux2filtered < (PPM_IN_MIDDLE_LENGTH - 300))) {
					
					main2commMessage.messageType = SET_TRAJECTORY;

					// when the trajectory is over
					if (++currentSetpointIdx >= TRAJECTORY_CIRCLE_LENGTH)
						currentSetpointIdx = 0; // reset it and go again
						
					int16_t futureSetpointIdx = currentSetpointIdx;
					
					int i;
					for (i = 0; i < 5; i++) {

						main2commMessage.data.trajectory.elevatorTrajectory[i] = pgm_read_float(&(elevatorCircle[futureSetpointIdx]));
						main2commMessage.data.trajectory.aileronTrajectory[i] = 0; // pgm_read_float(&(aileronCircle[futureSetpointIdx]));

						futureSetpointIdx = futureSetpointIdx + 50;
						
						if (futureSetpointIdx >= TRAJECTORY_CIRCLE_LENGTH)
							futureSetpointIdx = futureSetpointIdx - TRAJECTORY_CIRCLE_LENGTH;
					}

					xQueueSend(main2commsQueue, &main2commMessage, 0);
					
					AUX2_previous = 2;
					led_orange_on();

				} else if ((abs(aux2filtered - PPM_IN_MIDDLE_LENGTH) <= 300)) {
			
					main2commMessage.messageType = SET_SETPOINT;
					main2commMessage.data.simpleSetpoint.elevator = 0;
					main2commMessage.data.simpleSetpoint.aileron = 0;
					xQueueSend(main2commsQueue, &main2commMessage, 0);
					
					AUX2_previous = 3;
					led_orange_off();
				}
			}
		
			auxSetpointFlag = 0;
		}
	}
}


					/*						

					*/
							
		/* 
		 * Manual setpoint changes 
		 *
		if (auxSetpointFlag == 1) {
			
			aux2filtered = aux2filtered*0.99 + RCchannel[AUX2]*0.01;
			
			float filteredSetpoint = 5*((aux2filtered - PPM_IN_MIDDLE_LENGTH)/((float) (PPM_IN_MAX_LENGTH - PPM_IN_MIN_LENGTH)));
			
			float diference = filteredSetpoint - mpcSetpoints.elevatorSetpoint;
			
			if (diference > 0.00030)
				diference = 0.00030;
			else if (diference < -0.00030)
				diference = -0.00030;
				
			mpcSetpoints.elevatorSetpoint = mpcSetpoints.elevatorSetpoint + diference;
			
			auxSetpointFlag = 0;
		}
		*/