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
int8_t AUX1_previous = 0;
int8_t setpointChangeTrigger = 0;
int16_t currentSetpointIdx = 0;

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
		
		aux2filtered = aux2filtered*0.99 + RCchannel[AUX2]*0.01;
		
		led_orange_off();
		
		if (aux2filtered > (PPM_IN_MIDDLE_LENGTH + 500)) {
			
			
		} else if (aux2filtered < (PPM_IN_MIDDLE_LENGTH - 500)) {
			
			led_orange_on();
			
			if (auxSetpointFlag == 1) {
						
				if (setpointChangeTrigger++ == 11) {
							
					if (currentSetpointIdx++ >= TRAJECTORY_CIRCLE_LENGTH)
						currentSetpointIdx = 0;
						
					int futureSetpointIdx = currentSetpointIdx + 200;
					if (futureSetpointIdx >= TRAJECTORY_CIRCLE_LENGTH)
						futureSetpointIdx = futureSetpointIdx - TRAJECTORY_CIRCLE_LENGTH;
						
					mpcSetpoints.elevatorSetpoint = pgm_read_float(&(elevatorCircle[currentSetpointIdx]));
					mpcSetpoints.aileronSetpoint = pgm_read_float(&(aileronCircle[currentSetpointIdx]));
					
					mpcSetpoints.elevatorEndSetpoint = pgm_read_float(&(elevatorCircle[futureSetpointIdx]));
					mpcSetpoints.aileronEndSetpoint = pgm_read_float(&(aileronCircle[futureSetpointIdx]));
							
					setpointChangeTrigger = 1;
				}
				
				auxSetpointFlag = 0;
			}
		} else {
			
			mpcSetpoints.elevatorSetpoint = 0;
			mpcSetpoints.aileronSetpoint = 0;
			mpcSetpoints.elevatorEndSetpoint = 0;
			mpcSetpoints.aileronEndSetpoint = 0;
		}
		
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
	}
}