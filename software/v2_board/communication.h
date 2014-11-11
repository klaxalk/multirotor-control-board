#ifndef _COMMUNICATION_H
#define _COMMUNICATION_H

#include "mavlink/common/mavlink.h"

// define mavlink buffers lengths
#define MAX_SENDE_BUFF 20
#define MAX_EMPFANGS_BUFF 20

	//XBee
	extern volatile unsigned char leadK1enabled;

	// data from px4flow
	extern volatile float groundDistance;
	extern volatile float elevatorSpeed;
	extern volatile float aileronSpeed;
	extern volatile uint8_t px4Confidence;
	
	extern mavlink_message_t mavlinkMessage;
	extern mavlink_status_t mavlinkStatus;
	extern mavlink_optical_flow_t opticalFlowData;
	extern int8_t opticalFlowDataFlag;


	// data from Gumstix
	extern volatile float elevatorGumstix;
	extern volatile float aileronGumstix;
	extern volatile float throttleGumstix;	
	extern volatile int16_t xPosGumstixNew;
	extern volatile int16_t yPosGumstixNew;
	extern volatile int16_t zPosGumstixNew;	
	extern volatile int8_t validGumstix;
	extern volatile int8_t gumstixDataFlag;	
	
	//Telemtery sending
	extern unsigned char telemetryToCoordinatorArr[13];	

void leadK1Altitude();
void sendXBeePacket(unsigned char *packet);
int8_t px4flowParseChar(uint8_t incomingChar);
void gumstixParseChar(unsigned char incomingChar);

#endif // COMMUNICATION_H
