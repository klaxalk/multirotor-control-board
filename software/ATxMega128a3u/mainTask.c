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
volatile int16_t setpointChangeTrigger = 0;
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
				xQueueSend(main2commsQueue, &main2commMessage, 0);
				
				// wait between reseting the kalman and starting the controller
				vTaskDelay(50);
				
				enablePositionController();
			}
		
			AUX1_previous = 2;
		
		} else {
		
			disableAltitudeController();
			disableMpcController();
			AUX1_previous = 0;
		}
		
		#ifdef GRIPPER
		
		// aux stick for setting the setpoint
		aux3filtered = aux3filtered*0.997 + RCchannel[AUX3]*0.003;
		
		// up
		if ((aux3filtered > (PPM_IN_MIDDLE_LENGTH + 300))) {
			
			ioport_set_pin_level(GRIP_PIN, true);
		
		// down
		} else {
			
			ioport_set_pin_level(GRIP_PIN, false);
		}
		
		#endif
		
		// AUX switch on RC transmitter
		aux2filtered = aux2filtered*0.997 + RCchannel[AUX2]*0.003;
		
		if (auxSetpointFlag == 1) {
			
			if (setpointChangeTrigger++ == 10) {
				
				setpointChangeTrigger = 1;
				
				// switch is DOWN
				if ((aux2filtered < (PPM_IN_MIDDLE_LENGTH - 300))) {
					
					main2commMessage.messageType = SET_SETPOINT;
					main2commMessage.data.simpleSetpoint.elevator = 0;
					main2commMessage.data.simpleSetpoint.aileron = 0;
					xQueueSend(main2commsQueue, &main2commMessage, 0);

					led_orange_off();
				
				// switch is in the MIDDLE
				} else if ((abs(aux2filtered - PPM_IN_MIDDLE_LENGTH) <= 300)) {
					
					main2commMessage.messageType = SET_SETPOINT;
					main2commMessage.data.simpleSetpoint.elevator = 2;
					main2commMessage.data.simpleSetpoint.aileron = 0;
					xQueueSend(main2commsQueue, &main2commMessage, 0);
					
					led_orange_on();
				
				// switch is UP
				} else if ((aux2filtered > (PPM_IN_MIDDLE_LENGTH + 300))) {
				
					main2commMessage.messageType = SET_TRAJECTORY;

					// when the trajectory is over
					if (++currentSetpointIdx >= TRAJECTORY_LENGTH-1)
					currentSetpointIdx = 0; // reset it and go again
				
					int16_t futureSetpointIdx = currentSetpointIdx;
				
					int i;
					for (i = 0; i < 5; i++) {

						main2commMessage.data.trajectory.elevatorTrajectory[i] = pgm_read_float(&(trajectoryElevator[futureSetpointIdx]));
						main2commMessage.data.trajectory.aileronTrajectory[i] = pgm_read_float(&(trajectoryAileron[futureSetpointIdx]));
					
						futureSetpointIdx = futureSetpointIdx + 50;
					
						if (futureSetpointIdx > (TRAJECTORY_LENGTH-1))
						futureSetpointIdx -= TRAJECTORY_LENGTH;
					}

					xQueueSend(main2commsQueue, &main2commMessage, 0);
					
					led_orange_toggle();
				}
				
			}
		
			auxSetpointFlag = 0;
		}
	}
}