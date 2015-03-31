#ifndef CONSTANTS_H_
#define CONSTANTS_H_

#include "system.h"

typedef struct
{
	unsigned char COORDINATOR[8];
	unsigned char KC1[8];
	unsigned char K1[8];
	unsigned char K2[8];
	unsigned char K3[8];
	unsigned char BROADCAST[8];
	unsigned char UNKNOWN16[2];
} ADDRESST;

typedef struct
{
	unsigned char ALTITUDE_ESTIMATED;
	unsigned char ALTITUDE;
	unsigned char ELEVATOR_SPEED;
	unsigned char AILERON_SPEED;
	unsigned char ELEVATOR_SPEED_ESTIMATED;
	unsigned char AILERON_SPEED_ESTIMATED;
	unsigned char ELEVATOR_POSITION;
	unsigned char AILERON_POSITION;
	unsigned char ALTITUDE_CONTROLLER_OUTPUT;
	unsigned char ALTITUDE_SPEED;
	unsigned char AILERON_CONTROLLER_OUTPUT;
	unsigned char ELEVATOR_CONTROLLER_OUTPUT;
	unsigned char ALTITUDE_SETPOINT;
	unsigned char ELEVATOR_POS_SETPOINT;
	unsigned char AILERON_POS_SETPOINT;
	unsigned char ELEVATOR_SPEED_SETPOINT;
	unsigned char AILERON_SPEED_SETPOINT;
	unsigned char ELEVATOR_ACC;
	unsigned char AILERON_ACC;
	unsigned char VALID_GUMSTIX;
	unsigned char OUTPUT_THROTTLE;
	unsigned char OUTPUT_ELEVATOR;
	unsigned char OUTPUT_AILERON;
	unsigned char OUTPUT_RUDDER;
	unsigned char BLOB_ELEVATOR;
	unsigned char BLOB_AILERON;
	unsigned char BLOB_ALTITUDE;
	unsigned char PITCH_ANGLE;
	unsigned char ROLL_ANGLE;
} TELEMETRIEST;

#define TELEMETRY_VARIABLES 29


typedef struct
{
	unsigned char ON;
	unsigned char OFF;
} ONOFFT;

typedef struct
{
	unsigned char OFF;
	unsigned char VELOCITY;
	unsigned char POSITION;
	unsigned char MPC;
} CONTROLLERST;

typedef struct
{
	unsigned char STABILIZATION;
	unsigned char LANDING;
	unsigned char TAKE_OFF;
	unsigned char FLIGHT;
	unsigned char ON_GROUND;
} LANDINGT;

typedef struct
{
	unsigned char TELEMETRY_COORDINATOR;
	unsigned char LANDING;
	unsigned char CONTROLLERS;
	unsigned char TRAJECTORY_POINTS;
	unsigned char FOLLOWER_SET;
	unsigned char TIME;
	unsigned char POSITION_SET;
}COMMANDST;

extern unsigned char GET_STATUS;

extern ADDRESST ADDRESS;
extern TELEMETRIEST TELEMETRIES;
extern ONOFFT ONOFF;
extern CONTROLLERST CONTROLLERS;
extern LANDINGT LANDING;
extern COMMANDST COMMANDS;

void constInit();



#endif /* CONSTANTS_H_ */