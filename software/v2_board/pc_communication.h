/*
 * File name: pc_communication.h
 * Date:      2014/11/04 15:21
 * Author:    Jan Chudoba
 */

#ifndef __PC_COMMUNICATION_H__
#define __PC_COMMUNICATION_H__

#include <stdint.h>

extern volatile float pcVelocityMessageValue[4];
typedef enum {
	PC_VELOCITY_ELEVATOR,
	PC_VELOCITY_AILERON,
	PC_VELOCITY_YAW,
	PC_VELOCITY_CLIMB
} TPcMessageVelocityValue;

typedef enum {
	PC_MSG_NONE,
	PC_MSG_VELOCITY,
	PC_MSG_NUMBER // last
}TPcMessageType;

TPcMessageType pcParseChar(uint8_t incomingChar);

void pcResetAll();
void pcNewTrajectoryPoint();
void pcGotoHome();
void pcFollowTrajectory();
void pcStay();
void pcSetLogging(uint8_t enable);

#endif

/* end of pc_communication.h */
