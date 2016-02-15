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
				main2commMessage.data.simpleSetpoint.throttle = 0;
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
		
				if ((aux2filtered > (PPM_IN_MIDDLE_LENGTH + 300))) {
			
					main2commMessage.messageType = SET_SETPOINT;
					main2commMessage.data.simpleSetpoint.elevator = 0;
					main2commMessage.data.simpleSetpoint.aileron = 0;
					xQueueSend(main2commsQueue, &main2commMessage, 0);
			
					AUX2_previous = 1;
					led_orange_off();
			
				} else if ((aux2filtered < (PPM_IN_MIDDLE_LENGTH - 300))) {
					
					main2commMessage.messageType = SET_TRAJECTORY;

					// when the trajectory is over
					if (++currentSetpointIdx >= TRAJECTORY_LENGTH)
						currentSetpointIdx = TRAJECTORY_LENGTH-1; // reset it and go again
						
					int16_t futureSetpointIdx = currentSetpointIdx;
					
					// calculate the setpoint correction based on the multicon blob detection
					#ifdef MULTICON
		
					if (numberOfDetectedBlobs >= 1) {
			
						correctionX = kalmanStates.elevator.position + blobs[0].x - pgm_read_float(&(elevatorDiff[currentSetpointIdx])) - pgm_read_float(&(elevator[currentSetpointIdx]));
						correctionY = kalmanStates.aileron.position + blobs[0].y - pgm_read_float(&(aileronDiff[currentSetpointIdx])) - pgm_read_float(&(aileron[currentSetpointIdx]));
				
					}
		
					#endif
					
					// calculate the setpoint correction based on the multicon blob detection
					#ifdef RASPBERRY_PI
					
					if (rpiOk == 1) {
						
						correctionX = kalmanStates.elevator.position + rpix - 0.5 - pgm_read_float(&(elevator[currentSetpointIdx]));
						correctionY = kalmanStates.aileron.position + rpiy - pgm_read_float(&(aileron[currentSetpointIdx]));
					}
					
					#endif
					
					int i;
					for (i = 0; i < 5; i++) {
						
						#if defined (MULTICON) || defined (RASPBERRY_PI)

							main2commMessage.data.trajectory.elevatorTrajectory[i] = correctionX + pgm_read_float(&(elevator[futureSetpointIdx]));
							main2commMessage.data.trajectory.aileronTrajectory[i] = correctionY + pgm_read_float(&(aileron[futureSetpointIdx]));
						
						#else	

							main2commMessage.data.trajectory.elevatorTrajectory[i] = pgm_read_float(&(elevator[futureSetpointIdx]));
							main2commMessage.data.trajectory.aileronTrajectory[i] = pgm_read_float(&(aileron[futureSetpointIdx]));
						
						#endif

						futureSetpointIdx = futureSetpointIdx + 15;
					}

					xQueueSend(main2commsQueue, &main2commMessage, 0);
					
					AUX2_previous = 2;
					led_orange_on();

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
					
					xQueueSend(main2commsQueue, &main2commMessage, 0);
					
					AUX2_previous = 3;
					led_orange_off();
				}
			}
		
			auxSetpointFlag = 0;
		}
	}
}