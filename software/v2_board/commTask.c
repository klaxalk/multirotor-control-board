#include "system.h"
#include "commTask.h"
#include "communication.h"
#include "controllers.h"
#include "packets.h"
#include "commands.h"
#include "mpcHandler.h"

//in mm, must be positive! crops Gumstix values
#define POSITION_MAXIMUM   2000 


//XBee
#define XBEE_BUFFER_SIZE 60
unsigned char XBeeBuffer[XBEE_BUFFER_SIZE];	
uint8_t packetIn=0;
uint8_t packetLength=XBEE_BUFFER_SIZE-2;
uint8_t checksum=0xFF;


//	The message handler for STM
stmMessageHandler_t stmMessage;


void commTask(void *p) {	
	unsigned char inChar;
	
	//initialize Kalman filter and MPC
	stmResetKalman(0,0);
	stmSendSetpoint(0,0);	
		

	//wait for XBee	
	vTaskDelay(1000);
			
	while (1) {			
		//Auto state reports			
		stateChecker();				
		//send position to slave
		positionSlaveSend();
				
		
		if(timer20hz>=50){
			timer20hz-=50;
			telemetryToCoordinatorSend();					
		}
		
		//send trajectory to MPC
		if(trajSend==1 && landingState==LANDING_FLIGHT){
			stmSendSetpoint(setpoints.elevator-positionShift.elevator,setpoints.aileron-positionShift.aileron);				
			trajSend=0;			
		}	
		
										
		// XBee
		if (usartBufferGetByte(usart_buffer_xbee, &inChar, 0)) {						
			if(packetIn>0){
				*(XBeeBuffer+packetIn)=inChar;										
				
				if(packetIn==2){
					packetLength=inChar+3;
				}			
				
				if(packetIn>=3){
					checksum-=inChar;
				}				
				packetIn++;
			}
			
			if (inChar == 0x7E && packetIn==0){
				*XBeeBuffer=inChar;
				packetIn=1;
				checksum=0xFF;
			}
			
			if(packetIn>packetLength){
				packetIn=0;
				packetLength=XBEE_BUFFER_SIZE-2;				
				if(checksum==0){
					packetHandler(XBeeBuffer);		
				}		
			}				
		}			

		//Gumstix
		if (usartBufferGetByte(usart_buffer_4, &inChar, 0)) {
			gumstixParseChar(inChar);
		}
		
		if (gumstixDataFlag == 1) {
			if (validGumstix == 1) {
				//Gumstix returns position of the blob relative to camera
				//in mm, we want position of the drone relative to the blob in m.
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
			}
			gumstixDataFlag = 0;
		}
		
		if(gumstixStable){
			//Camera pointing forward and being LANDSCAPE oriented
			elevatorGumstix = xPosGumstixNew / 1000.0;
			aileronGumstix  = yPosGumstixNew / 1000.0;
			throttleGumstix = zPosGumstixNew / 1000.0;		
		}

		//PX4Flow
		if (usartBufferGetByte(usart_buffer_1, &inChar, 0)) {
			px4flowParseChar((uint8_t) inChar);
		}

		if (opticalFlowDataFlag == 1) {
			// decode the message (there will be new values in opticalFlowData...)
			mavlink_msg_optical_flow_decode(&mavlinkMessage, &opticalFlowData);
			
			//px4flow returns velocities in m/s and ground distance in m
			// +elevator = front
			// +aileron  = left
			// +throttle = up
			elevatorSpeed = - opticalFlowData.flow_comp_m_x;
			aileronSpeed  = + opticalFlowData.flow_comp_m_y;
			
			if (opticalFlowData.ground_distance < ALTITUDE_MAXIMUM 
			&& opticalFlowData.ground_distance >= 0.3 ) {
				groundDistance = opticalFlowData.ground_distance;
			}
			px4Confidence = opticalFlowData.quality;
			opticalFlowDataFlag = 0;
			
			if(altitude.position>ALTITUDE_MINIMUM){
				stmSendMeasurement(elevatorSpeed, aileronSpeed, controllerElevatorOutput, controllerAileronOutput);			
			}
		}


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
				
				portENTER_CRITICAL();
				
				// incoming outputs
				mpcElevatorOutput = readInt16(stmMessage.messageBuffer, &idx);
				mpcAileronOutput = readInt16(stmMessage.messageBuffer, &idx);				
				
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
	}	
}