#include "system.h"
#include "commTask.h"
#include "communication.h"
#include "controllers.h"
#include "packets.h"
#include "commands.h"
#include "mpcHandler.h"

//in mm, must be positive! crops Gumstix values
#define POSITION_MAXIMUM   2000 

// serial1 RX and TX
#define MAX_SENDE_BUFF     170
#define MAX_EMPFANGS_BUFF  170

signed volatile char SioTmp = 0;
unsigned volatile char DataReceived = 0;
unsigned volatile char CntCrcError = 0;
unsigned volatile char BytesReceiving = 0;
signed volatile char TxdBuffer[MAX_SENDE_BUFF];
signed volatile char RxdBuffer[MAX_EMPFANGS_BUFF];
unsigned volatile char transfereUart1done = 1;
unsigned char interval = 1;
volatile int flightCtrlDataReceived = 0;

volatile int16_t rollBoardAngle = 0;
volatile int16_t pitchBoardAngle = 0;
volatile int16_t pitchAngle = 0;
volatile int16_t rollAngle = 0;

signed char *pRxData = 0;
unsigned char RxDataLen = 0;

//XBee
#define XBEE_BUFFER_SIZE 60
unsigned char XBeeBuffer[XBEE_BUFFER_SIZE];	
uint8_t packetIn=0;
uint8_t packetLength=XBEE_BUFFER_SIZE-2;
uint8_t checksum=0xFF;


//	The message handler for STM
stmMessageHandler_t stmMessage;

// decode the base64 encoded data
void Decode64(void) {	
	unsigned char a,b,c,d;
	unsigned char x,y,z;
	unsigned char ptrIn = 3; // start at begin of data block
	unsigned char ptrOut = 3;
	unsigned char len = BytesReceiving - 6;

	while(len) {
		
		a = RxdBuffer[ptrIn++] - '=';
		b = RxdBuffer[ptrIn++] - '=';
		c = RxdBuffer[ptrIn++] - '=';
		d = RxdBuffer[ptrIn++] - '=';
		
		x = (a << 2) | (b >> 4);
		y = ((b & 0x0f) << 4) | (c >> 2);
		z = ((c & 0x03) << 6) | d;
		
		if(len--) RxdBuffer[ptrOut++] = x; else break;
		if(len--) RxdBuffer[ptrOut++] = y; else break;
		if(len--) RxdBuffer[ptrOut++] = z;	else break;
	}
	
	pRxData = (signed char*) &RxdBuffer[3]; // dekodování zaèíná 4. bytem
	RxDataLen = ptrOut - 3;  // kolik bylo dekodováno bytù?
}

void commTask(void *p) {	
	unsigned char inChar;
	int16_t counter20Hz = 0;	
	uint8_t i=0;
	
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
				
		//20Hz loop
		if (counter20Hz++>100){
			counter20Hz=0;
			telemetryToCoordinatorSend();						
		}	
		
		if(timer40hz>=25){
			timer40hz-=25;
			stmSendSetpoint(0-positionShift.elevator,0-positionShift.aileron);	
			for(i=0;i<5;i++){
				MPCElevatorTrajectory[i]=0-positionShift.elevator;
				MPCAileronTrajectory[i]=0-positionShift.aileron;
			}
					
		}
		
			//send trajectory to MPC
		/*if(trajSend==1){
			stmSendTrajectory(zeroTraj,zeroTraj);						
			trajSend=0;			
		}*/	
		
		
		
										
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
			&& opticalFlowData.ground_distance >= 0.2 ) {
				groundDistance = opticalFlowData.ground_distance;
			}
			px4Confidence = opticalFlowData.quality;
			opticalFlowDataFlag = 0;
			
			stmSendMeasurement(elevatorSpeed, aileronSpeed, controllerElevatorOutput, controllerAileronOutput);			
		}

		// receive data from MC control board		
		if (usartBufferGetByte(usart_buffer_2, &inChar, 0)) {

			static unsigned int crc;
			static unsigned char crc1,crc2,buf_ptr;
			static unsigned char UartState = 0;
			unsigned char CrcOkay = 0;

			SioTmp = inChar;

			if(buf_ptr >= MAX_SENDE_BUFF)
			UartState = 0;

			if(SioTmp == '\r' && UartState == 2) {
				UartState = 0;
				crc -= RxdBuffer[buf_ptr-2];
				crc -= RxdBuffer[buf_ptr-1];
				crc %= 4096;
				crc1 = '=' + crc / 64;
				crc2 = '=' + crc % 64;
				CrcOkay = 0;
				if((crc1 == RxdBuffer[buf_ptr-2]) && (crc2 == RxdBuffer[buf_ptr-1])) CrcOkay = 1; else { CrcOkay = 0; CntCrcError++;};
				if(!DataReceived && CrcOkay) // Celá správa pøijata
				{
					DataReceived = 1;
					flightCtrlDataReceived = 0;
					BytesReceiving = buf_ptr + 1;
					RxdBuffer[buf_ptr] = '\r';
				}
			}
			else
			switch(UartState)
			{
				case 0:
				if(SioTmp == '#' && !DataReceived) UartState = 1;  // Start char
				buf_ptr = 0;
				RxdBuffer[buf_ptr++] = SioTmp;
				crc = SioTmp;
				break;
				case 1: // Vyhonocení adresy
				UartState++;
				RxdBuffer[buf_ptr++] = SioTmp;
				crc += SioTmp;
				break;
				case 2: //  Bere data
				RxdBuffer[buf_ptr] = SioTmp;
				if(buf_ptr < MAX_EMPFANGS_BUFF) buf_ptr++;
				else UartState = 0;
				crc += SioTmp;
				break;
				default:
				UartState = 0;
				break;
			}
		}
		

		if (DataReceived == 1) {	    
			Decode64();	    
			char* dummy;
			int16_t tempAngle;
				    
			dummy = (char*) (&tempAngle);	    
			dummy[0] = pRxData[0];
			dummy[1] = pRxData[1];
	    
			if (RxdBuffer[2] == 'X') {		   
				pitchBoardAngle = tempAngle;
			} else if (RxdBuffer[2] == 'Y') {		    
				rollBoardAngle = tempAngle;
			}
	    
			pitchAngle = (pitchBoardAngle-rollBoardAngle)/2;
			rollAngle = (pitchBoardAngle+rollBoardAngle)/2;    	    
			DataReceived = 0;
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