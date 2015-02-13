/*
 * communicationTask.c
 *
 * Created: 11.9.2014 11:17:03
 *  Author: Tomas Baca
 */ 

#include "system.h"
#include "commTask.h"
#include "communication.h"
#include "controllers.h"

#include "mpcHandler.h"

/* -------------------------------------------------------------------- */
/*	For calculating rate of MPC and Kalman								*/
/* -------------------------------------------------------------------- */
volatile int16_t mpcCounter = 0;
volatile int16_t mpcRate = 0;
volatile int16_t kalmanCounter = 0;
volatile int16_t kalmanRate = 0;

/* -------------------------------------------------------------------- */
/*	Queues between tasks		 										*/
/* -------------------------------------------------------------------- */
main2commMessage_t main2commMessage;

/* -------------------------------------------------------------------- */
/*	The message handler for STM	 										*/
/* -------------------------------------------------------------------- */
stmMessageHandler_t stmMessage;

void commTask(void *p) {
	
	unsigned char inChar;
	
	initializeKalmanStates();
	
	mpcSetpoints.elevatorSetpoint = 0;
	mpcSetpoints.aileronSetpoint = 0;
		
	while (1) {
				
		/* -------------------------------------------------------------------- */
		/*	A character received from STM										*/
		/* -------------------------------------------------------------------- */
		if (usartBufferGetByte(usart_buffer_stm, &inChar, 0)) {

			// parse it and handle the message if it is complete
			if (stmParseChar(inChar, &stmMessage)) {
				
				// index for iterating the rxBuffer
				int idx = 0;
				
				// messageId == '1'
				// receive control actions computed by MPC
				if (stmMessage.messageId == '1') {
					
					int16_t tempInt;
				
					portENTER_CRITICAL();
				
					// saturate the incoming elevator output
					tempInt = readInt16(stmMessage.messageBuffer, &idx);
					if (tempInt > MPC_SATURATION)
						mpcElevatorOutput = MPC_SATURATION;
					else if (tempInt < -MPC_SATURATION)
						mpcElevatorOutput = -MPC_SATURATION;
					else
						mpcElevatorOutput = tempInt;
				
					// saturate the incoming aileron output
					tempInt = readInt16(stmMessage.messageBuffer, &idx);
					if (tempInt > MPC_SATURATION) {
						mpcAileronOutput = MPC_SATURATION;
					} else if (tempInt < -MPC_SATURATION) {
						mpcAileronOutput = -MPC_SATURATION;
					} else {
						mpcAileronOutput = tempInt;
					}
				
					// copy the saturated values
					controllerElevatorOutput = mpcElevatorOutput;
					controllerAileronOutput = mpcAileronOutput;
				
					portEXIT_CRITICAL();
					
				// messageId == '2'
				// receive all states estimated by kalman filter
				} else if (stmMessage.messageId == '2') {
					
					kalmanStates.elevator.position = readFloat(stmMessage.messageBuffer, &idx);
					kalmanStates.elevator.velocity = readFloat(stmMessage.messageBuffer, &idx);
					kalmanStates.elevator.acceleration = readFloat(stmMessage.messageBuffer, &idx);
					kalmanStates.elevator.acceleration_input = readFloat(stmMessage.messageBuffer, &idx);
					kalmanStates.elevator.acceleration_error = readFloat(stmMessage.messageBuffer, &idx);
					
					kalmanStates.aileron.position = readFloat(stmMessage.messageBuffer, &idx);
					kalmanStates.aileron.velocity = readFloat(stmMessage.messageBuffer, &idx);
					kalmanStates.aileron.acceleration = readFloat(stmMessage.messageBuffer, &idx);
					kalmanStates.aileron.acceleration_input = readFloat(stmMessage.messageBuffer, &idx);
					kalmanStates.aileron.acceleration_error = readFloat(stmMessage.messageBuffer, &idx);
				}
			}
		}
		
		/* -------------------------------------------------------------------- */
		/*	A character received from XBee										*/
		/* -------------------------------------------------------------------- */
		if (usartBufferGetByte(usart_buffer_xbee, &inChar, 0)) {

			// send a simple telemetry
			if (inChar == 'b') {

				char crc = 0;

				sendFloat(usart_buffer_xbee, kalmanStates.elevator.position, &crc);
				sendFloat(usart_buffer_xbee, kalmanStates.aileron.position, &crc);
				
				sendFloat(usart_buffer_xbee, mpcSetpoints.elevatorSetpoint, &crc);
				sendFloat(usart_buffer_xbee, mpcSetpoints.aileronSetpoint, &crc);
					
				sendInt16(usart_buffer_xbee, mpcElevatorOutput, &crc);
				sendInt16(usart_buffer_xbee, mpcAileronOutput, &crc);

				usartBufferPutString(usart_buffer_xbee, "\r\n", 10);
			}
		}
	
		/* -------------------------------------------------------------------- */
		/*	A character received from px4flow									*/
		/* -------------------------------------------------------------------- */
		if (usartBufferGetByte(usart_buffer_1, &inChar, 0)) {

			px4flowParseChar((uint8_t) inChar);
		
			if (opticalFlowDataFlag == 1) {

				// decode the message (there will be new values in opticalFlowData...
				mavlink_msg_optical_flow_decode(&mavlinkMessage, &opticalFlowData);
					
				// rotate px4flow data
				elevatorSpeed = -opticalFlowData.flow_comp_m_x;
				aileronSpeed  = +opticalFlowData.flow_comp_m_y;

				// saturate the ground distance
				if (opticalFlowData.ground_distance < ALTITUDE_MAXIMUM && opticalFlowData.ground_distance > 0.3) {

					groundDistance = opticalFlowData.ground_distance;
				}

				px4Confidence = opticalFlowData.quality;

				opticalFlowDataFlag = 0;
					
				/* -------------------------------------------------------------------- */
				/*	Send data to ARM													*/
				/* -------------------------------------------------------------------- */
					
				led_yellow_toggle();
					
				stmSendMeasurement(elevatorSpeed, aileronSpeed, mpcElevatorOutput, mpcAileronOutput);
				stmSendSetpointsSimple();
			}
		}
		
		/* -------------------------------------------------------------------- */
		/*	A message received from the main Task								*/
		/* -------------------------------------------------------------------- */
		if (xQueueReceive(main2commsQueue, &main2commMessage, 0)) {
						
			// send message to STM to reset its Kalman filter			
			if (main2commMessage.messageType == CLEAR_STATES) {
	
				stmResetKalman(0, 0);
			}
		}
	}
}