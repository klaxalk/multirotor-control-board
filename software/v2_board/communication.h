#ifndef _COMMUNICATION_H
#define _COMMUNICATION_H

#include "mavlink/common/mavlink.h"

// define mavlink buffers lengths
#define MAX_SENDE_BUFF 20
#define MAX_EMPFANGS_BUFF 20

#if PX4FLOW_DATA_RECEIVE == ENABLED
	// data from px4flow
	extern volatile float groundDistance;
	extern volatile float elevatorSpeed;
	extern volatile float aileronSpeed;
	extern volatile uint8_t px4Confidence;
	
	extern mavlink_message_t mavlinkMessage;
	extern mavlink_status_t mavlinkStatus;
	extern mavlink_optical_flow_t opticalFlowData;
	extern int8_t opticalFlowDataFlag;
#endif // PX4FLOW_DATA_RECEIVE

#if GUMSTIX_DATA_RECEIVE == ENABLED
	// data from Gumstix
	extern volatile float elevatorGumstix;
	extern volatile float aileronGumstix;
	extern volatile float throttleGumstix;	
	extern volatile int16_t xPosGumstixNew;
	extern volatile int16_t yPosGumstixNew;
	extern volatile int16_t zPosGumstixNew;	
	extern volatile int8_t validGumstix;
	extern volatile int8_t gumstixDataFlag;	
#endif // GUMSTIX_DATA_RECEIVE == ENABLED

//send XBee Packet
void sendXBeePacket(unsigned char *packet);

// merge RC channels with controller output
// the most important function, do not modify unless you know what you are doing!
void mergeSignalsToOutput();


#endif // COMMUNICATION_H
