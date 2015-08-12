#include <avr/io.h>
#include "ioport.h"
#include "controllers.h"
#include "communication.h"
#include "system.h"
#include "packets.h"
#include "commands.h"
#include "mpcHandler.h"

//XBee values
volatile unsigned char posSlave[4][8]={{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0}};

//px4flow values
volatile float groundDistance = 0;
volatile float elevatorSpeed = 0;
volatile float aileronSpeed = 0;
volatile uint8_t px4Confidence = 0;

// variables used by the mavlink library
mavlink_message_t mavlinkMessage;
mavlink_status_t mavlinkStatus;
mavlink_optical_flow_t opticalFlowData;
int8_t opticalFlowDataFlag = 0;

//Gumstix values
volatile unsigned char gumstixParseCharState = 0;
volatile unsigned char gumstixParseCharByte = 0;
volatile int16_t gumstixParseTempInt = 0;
volatile int16_t xPosGumstixNew = 0;
volatile int16_t yPosGumstixNew = 0;
volatile int16_t zPosGumstixNew = 0;
volatile float elevatorGumstix = 0;
volatile float aileronGumstix = 0;
volatile float throttleGumstix = 0;
volatile int8_t validGumstix = 0;
volatile int8_t gumstixDataFlag = 0;
volatile unsigned char gumstixParseCharCrc = 0;
volatile int8_t gumstixStable = 0;

int addressEqlCoord(unsigned char * addr){
	int8_t i;
	for(i=0;i<8;i++){
		if(*(addr+i)!=0x00){
			return 0;
		}
	}
	return 1;
}

//send position to position slave
void positionSlaveSend(){
	//TODO - 4 blobs
	uint8_t i;
	static uint32_t lastSendTime=0;
	for (i=0;i<4;i++)
	{
		if(gumstixStable==1 && (secondsTimer-lastSendTime)>1 && addressEqlCoord(*(posSlave+i))==0){
			kopterPositionSetRequest(*(posSlave+i),ADDRESS.UNKNOWN16,kalmanStates.elevator.position+elevatorGumstix+positionShift.elevator,kalmanStates.aileron.position+aileronGumstix+positionShift.aileron,0x00);
			lastSendTime=secondsTimer;
		}
	}
		
}

//send XBee packet
void sendXBeePacket(unsigned char *packet){
	int i;
	for (i=0;i<*(packet+2)+4;i++){
		usartBufferPutByte(usart_buffer_xbee, *(packet+i), 10);
	}
}

int8_t px4flowParseChar(uint8_t incomingChar) {
	if (mavlink_parse_char(MAVLINK_COMM_0, incomingChar, &mavlinkMessage, &mavlinkStatus)) {
		switch (mavlinkMessage.msgid) {
		case MAVLINK_MSG_ID_OPTICAL_FLOW: 
			opticalFlowDataFlag = 1;
			return 1;		
			break;
		default:
			//Do nothing
			break;
		}
	}
	return 0;
}

void gumstixParseChar(unsigned char incomingChar) {
	static uint8_t blobCounter = 0;
	if (gumstixParseCharByte == 2) {
		gumstixParseCharByte++;
	} else {
		gumstixParseCharCrc += incomingChar;
	}
	
	if (gumstixParseCharState == 0) {
		switch (incomingChar) {
		case 'x':
			gumstixParseCharState = 1;
			break;
		case 'y':
			gumstixParseCharState = 2;
			break;
		case 'z':
			gumstixParseCharState = 3;
			break;
		case 'v':
			gumstixParseCharState = 4;
			break;
		}

		gumstixParseCharByte = 0;
		gumstixParseTempInt = 0;
		gumstixParseCharCrc = incomingChar;

	} else if (gumstixParseCharByte < 2) {
		char* gumstixParseTempIntPointer = (char*) &gumstixParseTempInt;
		*(gumstixParseTempIntPointer+gumstixParseCharByte) = incomingChar;
		gumstixParseCharByte++;
	}

	if ((gumstixParseCharByte == 3) && (gumstixParseCharCrc == incomingChar)) { // we have the whole int red
		switch (gumstixParseCharState) {
		case 1:
			xPosGumstixNew = gumstixParseTempInt;
			break;
		case 2:
			yPosGumstixNew = gumstixParseTempInt;
			break;
		case 3:
			zPosGumstixNew = gumstixParseTempInt;
			break;
		case 4:
			validGumstix = gumstixParseTempInt;
			if (validGumstix == 1) {
				gumstixDataFlag = 1;				
				if(blobCounter<5){
					blobCounter++;
					}else{
					gumstixStable=1;
				}	
				led_blue_on();			
			}else{
				blobCounter=0;
				gumstixStable = 0;
				led_blue_off();
			}
			break;
		}
		gumstixParseCharState = 0;
	}
}

void stateChecker(){
	static unsigned char landingStateCH=0xFF;
	static unsigned char aktControllerCH=0xFF;
	
	if(landingStateCH!=landingState){
		landingStateCH=landingState;
		kopterLandReport(ADDRESS.COORDINATOR,ADDRESS.UNKNOWN16,0x00);
	}
	if(aktControllerCH!=controllerActive){
		aktControllerCH=controllerActive;
		kopterControllersReport(ADDRESS.COORDINATOR,ADDRESS.UNKNOWN16,0x00);
	}	
	
	landingStateCH=landingState;
	aktControllerCH=controllerActive;
}