/*
 * communicationTask.c
 *
 * Created: 11.9.2014 11:17:03
 *  Author: Tomas Baca
 */ 

#include "commTask.h"
#include "system.h"
#include "ioport.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usart_driver_RTOS.h"
#include "communication.h"
#include <stdio.h>
#include "controllers.h"
#if PC_COMMUNICATION == ENABLED
#include "pc_communication.h"
#endif
#include "debugcomm.h"

void commTask(void *p) {
	
	unsigned char inChar;
	
	while (1) {

		// xBee received
		if (usartBufferGetByte(usart_buffer_xbee, &inChar, 0)) {

			#if (GUMSTIX_DATA_RECEIVE == ENABLED) && (PX4FLOW_DATA_RECEIVE == ENABLED)
			// prototype of answering with log to xBee
			if (inChar == 'b') {
				char* ukazatel;

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

				usartBufferPutByte(usart_buffer_xbee, validGumstix, 10);
				
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
			#endif // GUMSTIX_DATA_RECEIVE == ENABLED
		}
		
		#if GUMSTIX_DATA_RECEIVE == ENABLED
		// gumstix communication
		if (usartBufferGetByte(usart_buffer_4, &inChar, 0)) {
			gumstixParseChar(inChar);
		}
		
		if (gumstixDataFlag == 1) {
			// gumstix message received
			
			if (validGumstix == 1) { // whether the blob detector outputs valid data

				//Gumstix returns position of the blob relative to camera
				//in milimeters, we want position of the drone relative
				//to the blob in meters.
				// +elevator = front
				// +aileron  = left
				// +throttle = up

				//saturation
				if(xPosGumstixNew > 2*POSITION_MAXIMUM) xPosGumstixNew = 2*POSITION_MAXIMUM;
				if(xPosGumstixNew < 0) xPosGumstixNew = 0; //distance from the blob (positive)

				if(yPosGumstixNew > +POSITION_MAXIMUM) yPosGumstixNew = +POSITION_MAXIMUM;
				if(yPosGumstixNew < -POSITION_MAXIMUM) yPosGumstixNew = -POSITION_MAXIMUM;

				if(zPosGumstixNew > +POSITION_MAXIMUM) zPosGumstixNew = +POSITION_MAXIMUM;
				if(zPosGumstixNew < -POSITION_MAXIMUM) zPosGumstixNew = -POSITION_MAXIMUM;

				#if GUMSTIX_CAMERA_POINTING == FORWARD //camera led on up side

					//~ Camera pointing forward and being PORTRAIT oriented
					//~ elevatorGumstix = - (float)xPosGumstixNew / 1000;
					//~ aileronGumstix  = - (float)zPosGumstixNew / 1000;
					//~ throttleGumstix = + (float)yPosGumstixNew / 1000;

					//~ Camera pointing forward and being LANDSCAPE oriented
					elevatorGumstix = - (float) xPosGumstixNew / 1000;
					aileronGumstix  = - (float) yPosGumstixNew / 1000;
					throttleGumstix = - (float) zPosGumstixNew / 1000;

				#elif GUMSTIX_CAMERA_POINTING == DOWNWARD //camera led on front side

					elevatorGumstix = + (float) yPosGumstixNew / 1000;
					aileronGumstix  = - (float) zPosGumstixNew / 1000;
					throttleGumstix = + (float) xPosGumstixNew / 1000;

				#endif
			}

			gumstixDataFlag = 0;
		}
		#endif // GUMSTIX_DATA_RECEIVE == ENABLED

		#if PX4FLOW_DATA_RECEIVE == ENABLED
		if (usartBufferGetByte(usart_buffer_1, &inChar, 0)) {
			// receive data from PX4Flow
			px4flowParseChar((uint8_t) inChar);
		}

		if (opticalFlowDataFlag == 1) {
			// PX4Flow message received

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
		#endif // PX4FLOW_DATA_RECEIVE == ENABLED

		#if PC_COMMUNICATION == ENABLED
		if (usartBufferGetByte(PC_USART_BUFFER, &inChar, 0)) {
			// receive data from PX4Flow
			//debugMessageF("PC in %c (%d)\r\n", inChar, inChar);
			TPcMessageType pcmsg = pcParseChar((uint8_t) inChar);
			switch (pcmsg) {
			case PC_MSG_VELOCITY:
				elevatorVelocitySetpoint = pcVelocityMessageValue[PC_VELOCITY_ELEVATOR];
				aileronVelocitySetpoint = pcVelocityMessageValue[PC_VELOCITY_AILERON];
				// TODO: yaw velocity setpoint = pcVelocityMessageValue[PC_VELOCITY_YAW];
				throttleVelocitySetpoint = pcVelocityMessageValue[PC_VELOCITY_CLIMB]; // TODO: setpoints() may overwrite my value
				// TODO: implement watchdog resetting setpoints when data are not updated
				debugMessageF("PC %.3f, %.3f, %.3f\r\n", pcVelocityMessageValue[PC_VELOCITY_ELEVATOR], pcVelocityMessageValue[PC_VELOCITY_AILERON], pcVelocityMessageValue[PC_VELOCITY_CLIMB]);
				break;
			case PC_MSG_NONE:
			case PC_MSG_NUMBER:
				;
			}
		}
		#endif
	}
}
