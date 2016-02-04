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

#ifdef IDENTIFICATION

volatile uint16_t dt_identification = 0;

#endif

#ifdef MAGNETOMETER

#include "magnetometer.h"

#endif

/* -------------------------------------------------------------------- */
/*	Xbee																*/
/* -------------------------------------------------------------------- */

xbeeMessageHandler_t xbeeMessage;
volatile uint8_t RSSI;
volatile uint64_t respondTo = 0;

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
					
					kalmanStates.elevatorPositionCovariance = readFloat(stmMessage.messageBuffer, &idx);
					kalmanStates.aileronPositionCovariance = readFloat(stmMessage.messageBuffer, &idx);
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
			if (rpiParseChar(inChar, &rpiMessage, &rpiReceiver)) {
				
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
			
				// free the buffer of the receiver
				rpiReceiver.receiverState = NOT_RECEIVING;
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
				
				switch (multiconMessage.messageId) {
					
					case 'A':
					
						blobId = readUint8(multiconMessage.messageBuffer, &idx);
						
						if (blobId < NUMBER_OF_BLOBS) {
							
							blobs[blobId].y = readFloat(multiconMessage.messageBuffer, &idx);
							blobs[blobId].z = readFloat(multiconMessage.messageBuffer, &idx);
							blobs[blobId].x = readFloat(multiconMessage.messageBuffer, &idx);
							
							/*
							uint8_t temp[32];
							sprintf(temp, "Blob %d [%2.3f %2.3f %2.3f]\n\r", blobId, blobs[blobId].x, blobs[blobId].y, blobs[blobId].z);
							usartBufferPutString(usart_buffer_3, temp, 10);
							*/
							led_green_on();
						}
						
					break;
			
					case 'B':
					
						multiconErrorState = readUint8(multiconMessage.messageBuffer, &idx);
						numberOfDetectedBlobs = readUint8(multiconMessage.messageBuffer, &idx);
						
						/*
						uint8_t temp[32];
						sprintf(temp, "Num. blobs = %d, Error = %d\n\r", numberOfDetectedBlobs, multiconErrorState);
						usartBufferPutString(usart_buffer_3, temp, 10);
						*/
						
						led_green_toggle();
						
						if (numberOfDetectedBlobs == 0) {
							
							led_green_off();
						}
					
					break;
					
					case 'M':
					
						// led_green_toggle();
						bluetooth_RSSI = readFloat(multiconMessage.messageBuffer, &idx);
					
					break;
				}
			}
		}

#endif
		
		/* -------------------------------------------------------------------- */
		/*	A character received from XBee										*/
		/* -------------------------------------------------------------------- */
		if (usartBufferGetByte(usart_buffer_xbee, &inChar, 0)) {

			if (xbeeParseChar(inChar, &xbeeMessage, &xbeeReceiver)) {
				
				if (xbeeMessage.apiId == XBEE_API_PACKET_RECEIVE) {
					
					int idx = 0;
					
					// pro ukazku
					char buffer[20];
					float tempFloat, jinyFloat, dalsiFloat;

					switch (xbeeMessage.content.receiveResponse.messageId) {
						
						case 'R':
						
							// pøíklad toho jak vyndám 3 promìnné z bufferu
							tempFloat = readFloat(xbeeMessage.content.receiveResponse.payload, &idx);
							tempFloat = readFloat(xbeeMessage.content.receiveResponse.payload, &idx);
							tempFloat = readFloat(xbeeMessage.content.receiveResponse.payload, &idx);

							// do something as a response to received message
							buffer[0] = 'A'; // prvni znak zprávy je pozdìji v xbeeMessage.content.receiveResponse.messageId
							writeFloatToBuffer(buffer, blobs[0].x, 1); // poèínaje bytem 1 zapíše float do buffer, tudíž v nìm bude 5 bytù
							xbeeSendMessageTo(buffer, 5, XBEE_UAV1);
						
						break;
						
						case 'M':
						
							sendBlobs(xbeeMessage.content.receiveResponse.address64);
						
						break;
					}
					
				}
							
				xbeeReceiver.receiverState = XBEE_NOT_RECEIVING;
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
				
				if (fabs(opticalFlowData.flow_comp_m_x) <= 1)
					elevatorSpeed = -opticalFlowData.flow_comp_m_x;
				
				if (fabs(opticalFlowData.flow_comp_m_y) <= 1)
					aileronSpeed  = +opticalFlowData.flow_comp_m_y;

				// saturate the ground distance
				if (opticalFlowData.ground_distance < ALTITUDE_MAXIMUM && opticalFlowData.ground_distance > 0.3) {

					groundDistance = opticalFlowData.ground_distance;
				}

				px4Confidence = opticalFlowData.quality;

				opticalFlowDataFlag = 0;			
				
				portEXIT_CRITICAL();
				
				#ifdef IDENTIFICATION
				
				char temp[30];
				
				sprintf(temp, "%1.2f, ", ((float) dt_identification)/1000);
				usartBufferPutString(usart_buffer_log, temp, 10);
				
				sprintf(temp, "%2.5f, ", elevatorSpeed);
				usartBufferPutString(usart_buffer_log, temp, 10);
				
				sprintf(temp, "%2.5f, ", aileronSpeed);
				usartBufferPutString(usart_buffer_log, temp, 10);
				
				sprintf(temp, "%2.5f, ", opticalFlowData.ground_distance);
				usartBufferPutString(usart_buffer_log, temp, 10);
				
				sprintf(temp, "%d, ", RCchannel[ELEVATOR]);
				usartBufferPutString(usart_buffer_log, temp, 10);
				
				sprintf(temp, "%d, ", RCchannel[AILERON]);
				usartBufferPutString(usart_buffer_log, temp, 10);
				
				sprintf(temp, "%d, ", RCchannel[THROTTLE]);
				usartBufferPutString(usart_buffer_log, temp, 10);
				
				usartBufferPutByte(usart_buffer_log, '\n', 10);
				
				dt_identification = 0;
				
				#endif
					
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
			
			} else if (main2commMessage.messageType == SET_SETPOINT) {
				
				stmSendSetpoint(main2commMessage.data.simpleSetpoint.elevator, main2commMessage.data.simpleSetpoint.aileron);

			} else if (main2commMessage.messageType == SET_TRAJECTORY) {
				
				stmSendTrajectory(main2commMessage.data.trajectory.elevatorTrajectory, main2commMessage.data.trajectory.aileronTrajectory);
			}
		}
	}
}