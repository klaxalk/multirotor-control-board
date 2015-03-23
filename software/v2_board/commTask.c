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

/* -------------------------------------------------------------------- */
/*	The message handler for raspberry									*/
/* -------------------------------------------------------------------- */
rpiMessageHandler_t rpiMessage;

volatile float rpix = 1.5;
volatile float rpiy = 0;
volatile float rpiz = 0;
volatile char rpiOk = 0;
volatile float curSetX = 0;
volatile float curSetY = 0;

void commTask(void *p) {
	
	unsigned char inChar;
	
	initializeKalmanStates();
	
	mpcSetpoints.elevator = 0;
	mpcSetpoints.aileron = 0;
		
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
					
					mpcSetpoints.elevator = readFloat(stmMessage.messageBuffer, &idx);
					mpcSetpoints.aileron = readFloat(stmMessage.messageBuffer, &idx);
				
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
		/*	A character received from raspberry									*/
		/* -------------------------------------------------------------------- */
		if (usartBufferGetByte(usart_buffer_2, &inChar, 0)) {

			// parse it and handle the message if it is complete
			if (rpiParseChar(inChar, &rpiMessage)) {
				
				// index for iterating the rxBuffer
				int idx = 0;
				
				// messageId == '1'
				// receive control actions computed by MPC
				if (rpiMessage.messageId == 'p') {
					
					int16_t tempInt;
					
					// saturate the incoming elevator output
					rpiy = -readFloat(rpiMessage.messageBuffer, &idx);
					rpiz = readFloat(rpiMessage.messageBuffer, &idx);
					rpix = readFloat(rpiMessage.messageBuffer, &idx);
					rpiOk = 1;
					led_green_on();
					led_blob_on();
					
				} else if (rpiMessage.messageId == '0') {
					
					led_green_off();
					led_blob_off();
					rpiOk = 0;
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
				
				sendFloat(usart_buffer_xbee, mpcSetpoints.elevator, &crc);
				sendFloat(usart_buffer_xbee, mpcSetpoints.aileron, &crc);
					
				sendInt16(usart_buffer_xbee, mpcElevatorOutput, &crc);
				sendInt16(usart_buffer_xbee, mpcAileronOutput, &crc);
				
				sendFloat(usart_buffer_xbee, rpix, &crc);
				sendFloat(usart_buffer_xbee, rpiy, &crc);
				sendFloat(usart_buffer_xbee, rpiz, &crc);
				
				sendFloat(usart_buffer_xbee, opticalFlowData.ground_distance, &crc);

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
				
				portENTER_CRITICAL();
				
				if (fabs(opticalFlowData.flow_comp_m_x) <= 0.6)
					elevatorSpeed = -opticalFlowData.flow_comp_m_x;
				
				if (fabs(opticalFlowData.flow_comp_m_y) <= 0.6)
					aileronSpeed  = +opticalFlowData.flow_comp_m_y;

				// saturate the ground distance
				if (opticalFlowData.ground_distance < ALTITUDE_MAXIMUM && opticalFlowData.ground_distance > 0.3) {

					groundDistance = opticalFlowData.ground_distance;
				}
	
				portEXIT_CRITICAL();

				px4Confidence = opticalFlowData.quality;

				opticalFlowDataFlag = 0;
					
				/* -------------------------------------------------------------------- */
				/*	Send data to ARM													*/
				/* -------------------------------------------------------------------- */
					
				led_yellow_toggle();
					
				stmSendMeasurement(elevatorSpeed, aileronSpeed, mpcElevatorOutput, mpcAileronOutput);
			}
		}
		
		/* -------------------------------------------------------------------- */
		/*	A message received from the main Task								*/
		/* -------------------------------------------------------------------- */
		if (xQueueReceive(main2commsQueue, &main2commMessage, 0)) {
						
			// send message to STM to reset its Kalman filter			
			if (main2commMessage.messageType == CLEAR_STATES) {
	
				stmResetKalman(main2commMessage.data.simpleSetpoint.elevator, main2commMessage.data.simpleSetpoint.aileron);
				
				curSetX = 0;
				curSetY = 0;
				rpix = 1.5;
				rpiy = 0;
				rpiOk = 0;
			
			} else if (main2commMessage.messageType == SET_SETPOINT) {
				
				if (rpiOk) {
					
					curSetX = kalmanStates.elevator.position + rpix - 1.5;
					curSetY = kalmanStates.aileron.position + rpiy;
				}
				stmSendSetpoint(curSetX, curSetY);

			} else if (main2commMessage.messageType == SET_TRAJECTORY) {
				
				stmSendTrajectory(main2commMessage.data.trajectory.elevatorTrajectory, main2commMessage.data.trajectory.aileronTrajectory);
			}
		}
	}
}