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
#include "config.h"
#include "mpcHandler.h"
#include "trajectories.h"
#include "xbee.h"

/* -------------------------------------------------------------------- */
/*	Execution rate of this task	is vital								*/
/* -------------------------------------------------------------------- */
volatile uint16_t commTaskRate = 0;
volatile uint16_t commTaskCounter = 0;

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

#ifdef RASPBERRY_PI

#include "raspberryPi.h"

/* -------------------------------------------------------------------- */
/*	The message handler for raspberry									*/
/* -------------------------------------------------------------------- */
rpiMessageHandler_t rpiMessage;

#endif

#ifdef GUMSTIX

#include "gumstix.h"

/* -------------------------------------------------------------------- */
/*	The message handler for gumstix									*/
/* -------------------------------------------------------------------- */
gumstixMessageHandler_t gumstixMessage;

#endif

#ifdef ARGOS

#include "argos3D.h"

/* -------------------------------------------------------------------- */
/*	The message handler for ARGOS										*/
/* -------------------------------------------------------------------- */
argosMessageHandler_t argosMessage;

#endif

#ifdef MULTICON

#include "multiCon.h"
multiconMessageHandler_t multiconMessage;

volatile float curSetX = 0;
volatile float curSetY = 0;
volatile float lastPositionX = 0;
volatile float lastPositionY = 0;

#endif

/* -------------------------------------------------------------------- */
/*	Xbee																*/
/* -------------------------------------------------------------------- */

xbeeMessageHandler_t xbeeMessage;
volatile uint8_t RSSI;
volatile uint64_t respondTo = 0;
volatile uint32_t timeStamp = 0;

void commTask(void *p) {
	
	uint8_t inChar;
	
	initializeKalmanStates();
	
	mpcSetpoints.elevator = 0;
	mpcSetpoints.aileron = 0;
		
	while (1) {
		
		commTaskCounter++;
				
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
				
					#ifdef MPC_POSITION_CONTROLLER
				
					// copy the saturated values
					controllerElevatorOutput = mpcElevatorOutput;
					controllerAileronOutput = mpcAileronOutput;
					
					mpcSetpoints.elevator = readFloat(stmMessage.messageBuffer, &idx);
					mpcSetpoints.aileron = readFloat(stmMessage.messageBuffer, &idx);
					
					#endif
				
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
		
#ifdef RASPBERRY_PI

/* -------------------------------------------------------------------- */
/*	A character received from raspberry									*/
/* -------------------------------------------------------------------- */
if (usartBufferGetByte(usart_buffer_2, &inChar, 0)) {
	
	#ifdef RASPBERRY_DOWNWARD
	float tempx, tempy, tempz;
	#endif

	// parse it and handle the message if it is complete
	if (rpiParseChar(inChar, &rpiMessage)) {
		
		// index for iterating the rxBuffer
		int idx = 0;
		
		switch (rpiMessage.messageId) {
			
			case 'p':
			
			#ifdef RASPBERRY_FORWARD
			rpiy = -readFloat(rpiMessage.messageBuffer, &idx);
			rpiz = readFloat(rpiMessage.messageBuffer, &idx);
			rpix = readFloat(rpiMessage.messageBuffer, &idx);
			#endif
			
			#ifdef RASPBERRY_DOWNWARD
			tempy = readFloat(rpiMessage.messageBuffer, &idx);
			tempz = readFloat(rpiMessage.messageBuffer, &idx);
			tempx = readFloat(rpiMessage.messageBuffer, &idx);
			
			rpiy = tempy;
			rpix = 0.9*tempz + (1 - 0.9)*tempx;
			rpiz = (1 - 0.9)*tempz + 0.9*tempx;
			#endif
			
			rpiOk = 1;
			
			led_green_on();
			
			break;
			
			case '0':
			
			rpiOk = 0;
			
			led_green_off();
			
			break;
		}
	}
}

#endif

#ifdef GUMSTIX

/* -------------------------------------------------------------------- */
/*	A character received from gumstix									*/
/* -------------------------------------------------------------------- */
if (usartBufferGetByte(usart_buffer_2, &inChar, 0))  {

	// parse it and handle the message if it is complete
	if (gumstixParseChar(inChar, &gumstixMessage)) {
		
		// index for iterating the rxBuffer
		int idx = 0;
		
		switch (gumstixMessage.messageId) {
			
			case 'x':
				
			
			break;
			case 'y':
			
			
			break;
			case 'z':

			
			break;
			case 'v':
			
				if (readInt16(gumstixMessage.messageBuffer, &idx) == 1) {
					
					gumstix_ok = 1;
					
				} else {
				
					gumstix_ok = 0;
					
					led_green_toggle();
				}
			
			break;
		}
	}
}

#endif

#ifdef ARGOS

		/* -------------------------------------------------------------------- */
		/*	A character received from Argos computer							*/
		/* -------------------------------------------------------------------- */
		if (usartBufferGetByte(usart_buffer_3, &inChar, 0)) {

			// parse it and handle the message if it is complete
			if (argosParseChar(inChar, &argosMessage)) {
				
				// index for iterating the rxBuffer
				int idx = 0;
				
				switch (argosMessage.messageId) {
					
					case 'A':
					
					break;
					
					case 'B':
					
					break;
				}
			}
		}

#endif

#ifdef MULTICON

		/* -------------------------------------------------------------------- */
		/*	A character received from Multicon system							*/
		/* -------------------------------------------------------------------- */
		if (usartBufferGetByte(usart_buffer_4, &inChar, 0)) {
			
			// parse it and handle the message if it is complete
			if (multiconParseChar(inChar, &multiconMessage)) {
		
				// index for iterating the rxBuffer
				int idx = 0;
				uint8_t blobId;
				float tempFloat;
				
				switch (multiconMessage.messageId) {
					
					case 'A':
					
						blobId = readUint8(multiconMessage.messageBuffer, &idx);
						
						if (blobId < NUMBER_OF_BLOBS) {
							
							blobs[blobId].y = readFloat(multiconMessage.messageBuffer, &idx);
							blobs[blobId].z = readFloat(multiconMessage.messageBuffer, &idx);
							blobs[blobId].x = readFloat(multiconMessage.messageBuffer, &idx);
							
							led_green_on();
						}
						
					break;
			
					case 'B':
					
						multiconErrorState = readUint8(multiconMessage.messageBuffer, &idx);
						numberOfDetectedBlobs = readUint8(multiconMessage.messageBuffer, &idx);
						
						led_green_toggle();
						
						if (numberOfDetectedBlobs == 0) {
							
							led_green_off();
						}
					
					break;
					
					case 'M':
					
						// led_green_toggle();
						tempFloat = readFloat(multiconMessage.messageBuffer, &idx);
						blobId = readUint8(multiconMessage.messageBuffer, &idx);
						
						led_orange_toggle();
					
					break;
				}
			}
		}

#endif
		
		/* -------------------------------------------------------------------- */
		/*	A character received from XBee										*/
		/* -------------------------------------------------------------------- */
		if (usartBufferGetByte(usart_buffer_xbee, &inChar, 0)) {

			// push the byte into the parsing function
			if (xbeeParseChar(inChar, &xbeeMessage, &xbeeReceiver)) {
				
				// the packet is a valid API packet
				if (xbeeMessage.apiId == XBEE_API_PACKET_RECEIVE) {
					
					int idx = 0;
					
					// check the message id 
					switch (xbeeMessage.content.receiveResponse.messageId) {
						
						case 'M':
						
							timeStamp = readUint32(xbeeMessage.content.receiveResponse.payload, &idx);
						
							#ifdef RASPBERRY_PI
							sendPiBlob(xbeeMessage.content.receiveResponse.address64);
							#endif
							
							#ifdef MULTICON
							sendBlobs(xbeeMessage.content.receiveResponse.address64);
							#endif
						
						break;
					}
					
				// the packet is a valid AT packet (probably response)
				// }
				// else if (xbeeMessage.apiId == XBEE_API_PACKET_AT_RESPONSE) {
				//
				//	// process the potential response to AT command to XBEE
				//	// xbeeMessage.content.atReponse.cmdData contains the response)
				//}
							
				xbeeReceiver.receiverState = XBEE_NOT_RECEIVING;
			}
		}
	
		/* -------------------------------------------------------------------- */
		/*	A character received from px4flow									*/
		/* -------------------------------------------------------------------- */
		if (usartBufferGetByte(usart_buffer_1, &inChar, 0)) {
			
			float elevatorSpeedSaturated, aileronSpeedSaturated;

			px4flowParseChar((uint8_t) inChar);
		
			if (opticalFlowDataFlag == 1) {

				// decode the message (there will be new values in opticalFlowData...
				mavlink_msg_optical_flow_decode(&mavlinkMessage, &opticalFlowData);
					
				// rotate px4flow data
				
				portENTER_CRITICAL();
				
				// saturate the elevator speed
				if (opticalFlowData.flow_comp_m_x > PX4FLOW_SPEED_SATURATION)
					elevatorSpeedSaturated = PX4FLOW_SPEED_SATURATION;
				else if (opticalFlowData.flow_comp_m_x < -PX4FLOW_SPEED_SATURATION)
					elevatorSpeedSaturated = -PX4FLOW_SPEED_SATURATION;
				else
					elevatorSpeedSaturated = opticalFlowData.flow_comp_m_x;
				
				// saturate the aileron speed
				if (opticalFlowData.flow_comp_m_y > PX4FLOW_SPEED_SATURATION)
					aileronSpeedSaturated = PX4FLOW_SPEED_SATURATION;
				else if (opticalFlowData.flow_comp_m_y < -PX4FLOW_SPEED_SATURATION)
					aileronSpeedSaturated = -PX4FLOW_SPEED_SATURATION;
				else
					aileronSpeedSaturated  = opticalFlowData.flow_comp_m_y;

				// rotate the coordinates
				elevatorSpeed = -elevatorSpeedSaturated;
				aileronSpeed = aileronSpeedSaturated;

				// saturate the ground distance
				if (opticalFlowData.ground_distance < ALTITUDE_MAXIMUM && opticalFlowData.ground_distance > 0.3) {

					groundDistance = opticalFlowData.ground_distance;
				}

				px4Confidence = opticalFlowData.quality;

				opticalFlowDataFlag = 0;			
				
				portEXIT_CRITICAL();
					
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
			
			if (main2commMessage.messageType == CLEAR_STATES) {
	
				// send message to STM to reset Kalman filter
				stmResetKalman(main2commMessage.data.simpleSetpoint.elevator, main2commMessage.data.simpleSetpoint.aileron);
			
			} else if (main2commMessage.messageType == SET_SETPOINT) {
				
				// send message to STM to set a single-point setpoint
				stmSendSetpoint(main2commMessage.data.simpleSetpoint.elevator, main2commMessage.data.simpleSetpoint.aileron);

			} else if (main2commMessage.messageType == SET_TRAJECTORY) {
				
				// send message to STM to set a 5-point trajectory
				stmSendTrajectory(main2commMessage.data.trajectory.elevatorTrajectory, main2commMessage.data.trajectory.aileronTrajectory);
			}
		}
	}
}

}