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
#include "src/pom_fce.h"

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

volatile uint8_t pes = 0;
volatile float kocka = 0;
volatile int init = 1;

int target_index=0;
float target_pos[2] = {5,5};
float target_pos_last_known[2] = {5,5};


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
					
					if(xQueueReceive(comm2mainQueue,&message,0)){
						
						if(init == 1){			
							//pes = message.data.pocet_blobu;	
									
							//inicializace
								if(message.data.pocet_blobu == 3){
									//int target_index;						
									target_index = initialize_target(&message,&kocka);
									pes = target_index;
									target_pos[0] = message.data.bloby[target_index][0];
									target_pos[1] =  message.data.bloby[target_index][1];
									target_pos_last_known[0] = target_pos[0];
									target_pos_last_known[1] = target_pos[1];
									init = 0;
									//pes = target_index;
									
								}
						}else {
								if(message.data.pocet_blobu >= 1){
									//sledovani targetu
									float nic;
									
									target_index = find_target(&message,target_pos_last_known,&kocka);
									//pes = target_index;
									//kocka = target_pos[1];
									//pes = target_index;
									if(target_index != 100){
										//vidim target
										target_pos[0] = message.data.bloby[target_index][0];
										target_pos[1] = message.data.bloby[target_index][1];
										target_pos_last_known[0] = target_pos[0];
										target_pos_last_known[1] = target_pos[1];
										//pes = 2;
									}else{
										//nevidim target
										//pes = 3;
										target_pos[0] = 0;
										target_pos[1] = 0;								
									}
									//target_intercept_pid(target_pos,velocity,20*0.05,0.56*0.05,0);
									//pruseciky
									//vidim 2 bloby a ani jeden neni target || nebo vidim 3 bloby a index targetu -> {0,1,2}
									if((message.data.pocet_blobu == 2 && target_index == 100)||(message.data.pocet_blobu == 3 && target_index <= 2)){
										float rel_neigh_pos[2][2];
										float prusecik_vysledny[2];
										int i,j;
										//vezmu si polohy ostatnich helikopter  ( vynecham target)
										for(i=0,j=0;i<message.data.pocet_blobu;i++){
											if(target_index != i){
												rel_neigh_pos[j][0] = message.data.bloby[i][0];
												rel_neigh_pos[j][1] = message.data.bloby[i][1];
												j++;
											}
										}
										//najdu nejblizsi prusecik 
										nearest_intersection(rel_neigh_pos,1.8,prusecik_vysledny);
										main2commMessage.messageType = SET_SETPOINT;
										main2commMessage.data.simpleSetpoint.elevator = prusecik_vysledny[0]/3 + target_pos[0];
										main2commMessage.data.simpleSetpoint.aileron = prusecik_vysledny[1]/3 + target_pos[1];									
										xQueueSend(main2commsQueue, &main2commMessage, 0);
									}else{
										main2commMessage.messageType = SET_SETPOINT;
										main2commMessage.data.simpleSetpoint.elevator = target_pos[0];
										main2commMessage.data.simpleSetpoint.aileron = target_pos[1];
										xQueueSend(main2commsQueue, &main2commMessage, 0);
									}
								}else{
								//nic nedelat
							}
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