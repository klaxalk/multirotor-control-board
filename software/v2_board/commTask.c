#include <stdio.h>
#include "commTask.h"
#include "usart_driver_RTOS.h"
#include "communication.h"
#include "controllers.h"
#include "system.h"
#include "packets.h"
#include "commands.h"

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
	//cca 40kHz	
	unsigned char inChar;
	unsigned char packet[60];	
	int8_t i;
	int16_t counter40Hz=500;
	int16_t counter20Hz=0;
	
	//wait for XBee	
	vTaskDelay(1000);
	

		
	while (1) {				
		stateChecker();
		
		if (counter40Hz++>10000){
			kopterTimeReport(ADDRESS.COORDINATOR,ADDRESS.UNKNOWN16,0x00);
			counter40Hz=0;
		}
		
		if (counter20Hz++>2000){
			counter20Hz=0;
			telemetryToCoordinatorSend();
			if(positionControllerEnabled && leadKopter[7]!=0x00){
				kopterLeadDataSend(leadKopter,ADDRESS.UNKNOWN16,estimatedThrottlePos,elevatorDesiredSpeedPosController,aileronDesiredSpeedPosController,elevatorPosContError,aileronPosContError,0x00);			
				led_blue_toggle();
			}
		}		
		
		
		// XBee
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

				#if GUMSTIX_CAMERA_POINTING == FORWARD //camera led on up side
					//Camera pointing forward and being LANDSCAPE oriented
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
		}
		
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
		
		// receive data from MC control board
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
	}	
}