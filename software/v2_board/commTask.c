/*
 * communicationTask.c
 *
 * Created: 11.9.2014 11:17:03
 *  Author: Tomas Baca
 */

#include "packets.h"
#include "commands.h"

#include "commTask.h"
#include "system.h"
#include "ioport.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usart_driver_RTOS.h"
#include "communication.h"
#include <stdio.h>

extern volatile uint16_t RCchannel[9];
extern UsartBuffer * usart_buffer_xbee;
extern UsartBuffer * usart_buffer_1;

extern volatile float groundDistance;
extern volatile float elevatorSpeed;
extern volatile float aileronSpeed;
extern volatile uint8_t px4Confidence;

extern volatile float estimatedElevatorPos;
extern volatile float estimatedAileronPos;

// variables used by the mavlink library
extern mavlink_message_t mavlinkMessage;
extern mavlink_status_t mavlinkStatus;
extern int8_t mavlinkFlag;
extern mavlink_optical_flow_t opticalFlowData;
extern int8_t opticalFlowDataFlag;


//recognize packet
void recPacket(){
    char *address16[2];
    char *address64[8];
    char recieveOptions;
    char *data[20];

    switch (*(inPacket+3)) {
    //Modem Status
    case 0x8A:

        break;
    //Transmit Status
    case 0x8B:

        break;
    //Receive Packet
    case 0x90:
            parReceivePacket(address64,address16,&recieveOptions,data);
            switch (*(data+1)){
                //command
                case 'c':
                        switch(*(data+2)){
                            //GroundDistance telemetry request
                            case 0x01:
                                    groundDistanceTelemetry(address64,address16,0x00);
                                break;
                        }
                    break;
                //telemetry
                case 't':
                    break;
                //report
                case 'r':
                    break;
                //warning
                case 'w':
                    break;
                //error
                case 'e':
                    break;
                default:
                    /*TODO neimplementovany parametr*/
                    break;
            }
        break;
    default:
           /*TODO zablikat ledkou ze je neni implementovano nebo poslat PC upozorneni ze se neco nezpracovalo */
        break;
    }

}

void commTask(void *p) {

	unsigned char inChar;

	char* ukazatel;

	int i;

	while (1) {

        //xbee send packet
        if (sendPacket){
            /*TODO
            nemam paru jak to funguje a jak poslat jen cast stringu
            usartBufferPutString(usart_buffer_xbee, outPacket, 10);
            */
            for (i=0;i<*(outPacket+2)+4){
                usartBufferPutByte(usart_buffer_xbee, *(outPacket+i), 10);
            }
            sendPacket=0;
        }


		// xbee received
		if (usartBufferGetByte(usart_buffer_xbee, &inChar, 0)) {

            //packet received
            if (inChar == 0x7E){
                *inPacket=inChar;
                while(!usartBufferGetByte(usart_buffer_xbee, inPacket+1, 0));
                while(!usartBufferGetByte(usart_buffer_xbee, inPacket+2, 0));
                for (i=0;i<*(inPacket+2)+1;i++){
                    while(!usartBufferGetByte(usart_buffer_xbee, inPacket+3+i, 0));
                }
                recPacket();
            }

			if (inChar == 'x') {

				int i;
				for (i = 0; i < 4; i++) {

					usartBufferPutInt(usart_buffer_xbee, RCchannel[i], 10, 10);
					usartBufferPutString(usart_buffer_xbee, ", ", 10);
				}
				usartBufferPutString(usart_buffer_xbee, "\r\n", 10);
			}

			if (inChar == 'v') {

				char buffer[20];
				sprintf(buffer, "%2.2f %2.2f %2.2f\r\n", elevatorSpeed, aileronSpeed, groundDistance);
				usartBufferPutString(usart_buffer_xbee, buffer, 10);
			}

			if (inChar == 'b') {

				ukazatel = (char*) &estimatedElevatorPos;
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel), 10);
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel+1), 10);
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel+2), 10);
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel+3), 10);

				ukazatel = (char*) &estimatedAileronPos;
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel), 10);
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel+1), 10);
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel+2), 10);
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel+3), 10);

				ukazatel = (char*) &estimatedThrottlePos;
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel), 10);
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel+1), 10);
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel+2), 10);
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel+3), 10);

				ukazatel = (char*) &elevatorSetpoint;
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel), 10);
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel+1), 10);
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel+2), 10);
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel+3), 10);

				ukazatel = (char*) &aileronSetpoint;
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel), 10);
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel+1), 10);
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel+2), 10);
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel+3), 10);

				usartBufferPutByte(usart_buffer_xbee, controllerEnabled, 10);
				usartBufferPutByte(usart_buffer_xbee, positionControllerEnabled, 10);
				usartBufferPutByte(usart_buffer_xbee, landingRequest, 10);
				usartBufferPutByte(usart_buffer_xbee, trajectoryEnabled, 10);

				ukazatel = (char*) &elevatorSpeed;
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel), 10);
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel+1), 10);
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel+2), 10);
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel+3), 10);

				ukazatel = (char*) &aileronSpeed;
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel), 10);
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel+1), 10);
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel+2), 10);
				usartBufferPutByte(usart_buffer_xbee, *(ukazatel+3), 10);

				usartBufferPutString(usart_buffer_xbee, "\r\n", 10);
			}
		}

		if (usartBufferGetByte(usart_buffer_1, &inChar, 0)) {

			px4flowParseChar((uint8_t) inChar);
		}

		if (opticalFlowDataFlag == 1) {

			led_blue_toggle();

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

			if (opticalFlowData.ground_distance < ALTITUDE_MAXIMUM &&
			opticalFlowData.ground_distance > 0.3) {

				groundDistance = opticalFlowData.ground_distance;
			}

			px4Confidence = opticalFlowData.quality;

			opticalFlowDataFlag = 0;
		}
	}
}
