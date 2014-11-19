#include "packets.h"
#include "commTask.h"
#include "system.h"
#include "ioport.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usart_driver_RTOS.h"
#include "communication.h"
#include <stdio.h>

//Comm ports/USARTs
extern volatile uint16_t RCchannel[9];
extern UsartBuffer * usart_buffer_xbee;
extern UsartBuffer * usart_buffer_1;
extern UsartBuffer * usart_buffer_4;

//PX4Flow
extern volatile float groundDistance;
extern volatile float elevatorSpeed;
extern volatile float aileronSpeed;
extern volatile uint8_t px4Confidence;

// variables used by the mavlink library
extern mavlink_message_t mavlinkMessage;
extern mavlink_status_t mavlinkStatus;
extern int8_t mavlinkFlag;
extern mavlink_optical_flow_t opticalFlowData;
extern int8_t opticalFlowDataFlag;





//send XBee packet
void sendXBeePacket(unsigned char *packet){
    int i;

    for (i=0;i<*(packet+2)+4;i++){
        usartBufferPutByte(usart_buffer_xbee, *(packet+i), 10);
    }
}

//send XBee packet USART4(DEBUG)
void sendPacketUART(unsigned char *packet){
	int i;

	for (i=0;i<*(packet+2)+4;i++){
		usartBufferPutByte(usart_buffer_4, *(packet+i), 10);
	}
}

void commTask(void *p) {
	unsigned char inChar;
	unsigned char packet[60];
	int i;
	while (1) {			
		// XBEE received
		if (usartBufferGetByte(usart_buffer_xbee, &inChar, 0)) {																			
            //packet received
            if (inChar == 0x7E){								
                *packet=inChar;
                while(!usartBufferGetByte(usart_buffer_xbee, packet+1, 0));
                while(!usartBufferGetByte(usart_buffer_xbee, packet+2, 0));
                for (i=0;i<*(packet+2)+1;i++){
                    while(!usartBufferGetByte(usart_buffer_xbee, packet+3+i, 0));
                }
                packetHandler(packet);
            }		
		}
		
		//PX4Flow
		if (usartBufferGetByte(usart_buffer_1, &inChar, 0)) {
			px4flowParseChar((uint8_t) inChar);
		}

		if (opticalFlowDataFlag == 1) {
			// decode the message (there will be new values in opticalFlowData...
			mavlink_msg_optical_flow_decode(&mavlinkMessage, &opticalFlowData);
			
			//px4flow returns velocities in m/s and gnd. distance in m
			// +elevator = front
			// +aileron  = left
			// +throttle = up

			#if PX4_CAMERA_ORIENTATION == FORWARD
				elevatorSpeed = + opticalFlowData.flow_comp_m_x;
				aileronSpeed  = - opticalFlowData.flow_comp_m_y;

			#elif PX4_CAMERA_ORIENTATION == BACKWARD
				elevatorSpeed = - opticalFlowData.flow_comp_m_x;
				aileronSpeed  = + opticalFlowData.flow_comp_m_y;
			#endif
		
			
			
			

			//Ground distance saturation
			if (opticalFlowData.ground_distance < ALTITUDE_MAXIMUM &&
			opticalFlowData.ground_distance > 0.3) {
				groundDistance = opticalFlowData.ground_distance;
				
				
			}

			px4Confidence = opticalFlowData.quality;
			opticalFlowDataFlag = 0;
		}
	}
}
