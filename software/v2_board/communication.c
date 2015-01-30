#include <avr/io.h>
#include "ioport.h"
#include "config.h"
#include "controllers.h"
#include "communication.h"
#include "system.h"
#include "packets.h"
#include "commands.h"

//XBee values
volatile unsigned char leadKopter[8]={0x00};

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
				led_blue_on();
			}else{
				led_blue_off();
			}
			break;
		}
		gumstixParseCharState = 0;
	}
}

void stateChecker(){
	static unsigned char landingStateCH=0xFF;
	static unsigned char trajectoryFollowCH=0xFF;
	static unsigned char aktControllerCH=0xFF;
	
	if(landingStateCH!=landingState){
		landingStateCH=landingState;
		kopterLandReport(ADDRESS.COORDINATOR,ADDRESS.UNKNOWN16,0x00);
	}
	if(trajectoryFollowCH!=trajectoryEnabled){
		trajectoryFollowCH=trajectoryEnabled;
		kopterTrajectoryFollowReport(ADDRESS.COORDINATOR,ADDRESS.UNKNOWN16,0x00);
	}
	if(aktControllerCH!=controllerActive){
		aktControllerCH=controllerActive;
		kopterControllersReport(ADDRESS.COORDINATOR,ADDRESS.UNKNOWN16,0x00);
	}	
	
	landingStateCH=landingState;
	trajectoryFollowCH=trajectoryEnabled;
	aktControllerCH=controllerActive;
}