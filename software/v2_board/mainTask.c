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

#ifdef MULTICON

#include "multiCon.h"
float correctionX = 0;
float correctionY = 0;

#endif

#ifdef RASPBERRY_PI

#include "raspberryPi.h"
float correctionX = -0.5;
float correctionY = 0;

#endif

void mainTask(void *p) {

	main2commMessage_t main2commMessage;
	main2commMessage.messageType = CLEAR_STATES;
	xQueueSend(main2commsQueue, &main2commMessage, 0);

	vTaskDelay(50);
	
	int16_t aux2filtered = PPM_IN_MIDDLE_LENGTH;
	int16_t aux3filtered = PPM_IN_MIDDLE_LENGTH;
	
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
				main2commMessage.data.simpleSetpoint.throttle = 1;
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
			
			if (setpointChangeTrigger++ == 30) {
				
				setpointChangeTrigger = 1;
				
				// up
				if ((aux2filtered > (PPM_IN_MIDDLE_LENGTH + 300))) {
					
					main2commMessage.messageType = SET_SETPOINT;
					main2commMessage.data.simpleSetpoint.elevator = 0;
					main2commMessage.data.simpleSetpoint.aileron = 0;
					main2commMessage.data.simpleSetpoint.throttle = 1;
					xQueueSend(main2commsQueue, &main2commMessage, 0);
					
					AUX2_previous = 1;
					led_orange_off();
					
					// down
					} else if ((aux2filtered < (PPM_IN_MIDDLE_LENGTH - 300))) {
					
					main2commMessage.messageType = SET_SETPOINT;
					main2commMessage.data.simpleSetpoint.elevator = 0;
					main2commMessage.data.simpleSetpoint.aileron = 0;
					main2commMessage.data.simpleSetpoint.throttle = 1;
					xQueueSend(main2commsQueue, &main2commMessage, 0);
					
					AUX2_previous = 2;
					led_orange_on();

					// mid
					} else if ((abs(aux2filtered - PPM_IN_MIDDLE_LENGTH) <= 300)) {
					
					main2commMessage.messageType = SET_SETPOINT;
					
					#if defined (MULTICON)
					
					if (numberOfDetectedBlobs >= 1) {
						
						correctionX = kalmanStates.elevator.position + blobs[0].x;
						correctionY = kalmanStates.aileron.position + blobs[0].y;
						
					}
					
					main2commMessage.data.simpleSetpoint.elevator = -pgm_read_float(&(elevatorDiff[0])) + correctionX;
					main2commMessage.data.simpleSetpoint.aileron = -pgm_read_float(&(aileronDiff[0])) + correctionY;
					#elif defined (RASPBERRY_PI)
					
					if (rpiOk >= 1) {
						
						correctionX = kalmanStates.elevator.position + rpix;
						correctionY = kalmanStates.aileron.position + rpiy;
					}
					
					main2commMessage.data.simpleSetpoint.elevator = 0 + correctionX;
					main2commMessage.data.simpleSetpoint.aileron = 0 + correctionY;
					
					#else
					
					main2commMessage.data.simpleSetpoint.elevator = 0;
					main2commMessage.data.simpleSetpoint.aileron = 0;
					
					#endif
					
					main2commMessage.data.simpleSetpoint.throttle = 1;
					xQueueSend(main2commsQueue, &main2commMessage, 0);
					
					AUX2_previous = 3;
					led_orange_off();
				}
			}
			
			auxSetpointFlag = 0;
		}
		
		// aux stick for setting the altitude setpoint
		aux3filtered = aux3filtered*0.997 + RCchannel[AUX3]*0.003;
		
		if (setpointChangeTrigger == 29) {			
			
			// up (2m)
			if ((aux3filtered > (PPM_IN_MIDDLE_LENGTH + 300))) {
				throttleSetpoint = 2;
				throttleIntegration = 0;
				
			// down (1m)
			} else if ((aux3filtered < (PPM_IN_MIDDLE_LENGTH - 300))) {
				throttleSetpoint = 1;
				throttleIntegration = 0;

			// mid (1.5m)
			} else if ((abs(aux3filtered - PPM_IN_MIDDLE_LENGTH) <= 300)) {
				throttleSetpoint = 1.5;
				throttleIntegration = 0;	//Smoke mid everyday. watch?v=dSeDMjfjVGo
			}
		}		
	}
}