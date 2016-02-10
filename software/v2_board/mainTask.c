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
#include "pom_fce.h"

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
		
		//horni poloha
				if ((aux2filtered > (PPM_IN_MIDDLE_LENGTH + 300))) {
			
					main2commMessage.messageType = SET_SETPOINT;
					main2commMessage.data.simpleSetpoint.elevator = -1;
					main2commMessage.data.simpleSetpoint.aileron = 0;
					xQueueSend(main2commsQueue, &main2commMessage, 0);
			
					AUX2_previous = 1;
					led_orange_toggle();
			
			//prostredni poloha
				} else if ((abs(aux2filtered - PPM_IN_MIDDLE_LENGTH) <= 300)) {
					comm2mainMessage_t message;
					
				//	main2commMessage.messageType = SET_SETPOINT;
				//	main2commMessage.data.simpleSetpoint.elevator = 0;
				//	main2commMessage.data.simpleSetpoint.aileron = 0;
				//	xQueueSend(main2commsQueue, &main2commMessage, 0);
					
					if(xQueueReceive(comm2mainQueue,&message,0)){
						if(message.data.pocet_blobu == 1){
							//sledovani targetu
							float target_pos[2];
							float velocity[2];
							
							target_pos[0] = message.data.bloby[0][0];
							target_pos[1] = message.data.bloby[0][1];
							target_intercept_pid(target_pos,velocity,20*0.05,0.56*0.05,0);
							
							main2commMessage.messageType = SET_SETPOINT;
							
							main2commMessage.data.simpleSetpoint.elevator = velocity[0]*0.5;
							main2commMessage.data.simpleSetpoint.aileron = velocity[1]*0.5;
								xQueueSend(main2commsQueue, &main2commMessage, 0);
							
							}else if (message.data.pocet_blobu == 2){
							//pruseciky
							float rel_neigh_pos[2][2];
							float prusecik_vysledny[2];
							rel_neigh_pos[0][0] = message.data.bloby[0][0];
							rel_neigh_pos[0][1] = message.data.bloby[0][1];
							rel_neigh_pos[1][0] = message.data.bloby[1][0];
							rel_neigh_pos[1][1] = message.data.bloby[1][1];
							
							
								nearest_intersection(rel_neigh_pos,1.8,prusecik_vysledny);
								main2commMessage.messageType = SET_SETPOINT;
								main2commMessage.data.simpleSetpoint.elevator = prusecik_vysledny[0];
								main2commMessage.data.simpleSetpoint.aileron = prusecik_vysledny[1];
								
							xQueueSend(main2commsQueue, &main2commMessage, 0);
							
							}else{
							//nic nedelat
						}
						
						
					}
					
					
					AUX2_previous = 2;
					led_orange_on();
					
//dolni


				} else if ((aux2filtered < (PPM_IN_MIDDLE_LENGTH - 300)))  {
			
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