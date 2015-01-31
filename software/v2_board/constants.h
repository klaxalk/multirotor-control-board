#ifndef CONSTANTS_H_
#define CONSTANTS_H_


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
	unsigned char GROUND_DISTANCE_ESTIMATED;
	unsigned char GROUND_DISTANCE;
	unsigned char ELEVATOR_SPEED;
	unsigned char AILERON_SPEED;
	unsigned char ELEVATOR_SPEED_ESTIMATED;
	unsigned char AILERON_SPEED_ESTIMATED;
	unsigned char ELEVATOR_POS_ESTIMATED;
	unsigned char AILERON_POS_ESTIMATED;
	unsigned char THROTTLE_CONTROLLER_OUTPUT;
	unsigned char THROTTLE_SPEED;
	unsigned char AILERON_CONTROLLER_OUTPUT;
	unsigned char ELEVATOR_CONTROLLER_OUTPUT;
	unsigned char THROTTLE_SETPOINT;
	unsigned char ELEVATOR_POS_SETPOINT;
	unsigned char AILERON_POS_SETPOINT;
	unsigned char ELEVATOR_VEL_SETPOINT;
	unsigned char AILERON_VEL_SETPOINT;
	unsigned char ELEVATOR_ACC;
	unsigned char AILERON_ACC;
	unsigned char VALID_GUMSTIX;
	unsigned char OUTPUT_THROTTLE;
	unsigned char OUTPUT_ELEVATOR;
	unsigned char OUTPUT_AILERON;
	unsigned char OUTPUT_RUDDER;
	unsigned char BLOB_DISTANCE;
	unsigned char BLOB_HORIZONTAL;
	unsigned char BLOB_VERTICAL;
	unsigned char PITCH_ANGLE;
	unsigned char ROLL_ANGLE;
} TELEMETRIEST;

#define TELEMETRY_VARIABLES 37


typedef struct
{
	unsigned char ON;
	unsigned char OFF;
} ONOFFT;

typedef struct
{
	unsigned char THROTTLE_SP;
	unsigned char ELEVATOR_POSITION;
	unsigned char AILERON_POSITION;
	unsigned char ELEVATOR_VELOCITY;
	unsigned char AILERON_VELOCITY;
} SETPOINTST;

typedef struct
{
	unsigned char ABSOLUT;
	unsigned char RELATIV;
} POSITIONST;

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
	unsigned char TELEMETRY;
	unsigned char TELEMETRY_COORDINATOR;
	unsigned char LANDING;
	unsigned char SET_SETPOINTS;
	unsigned char CONTROLLERS;
	unsigned char TRAJECTORY_FOLLOW;
	unsigned char TRAJECTORY_POINTS;
	unsigned char GUMSTIX;
	unsigned char FOLLOWER_SET;
	unsigned char TIME;
}COMMANDST;

extern unsigned char GET_STATUS;

extern ADDRESST ADDRESS;
extern TELEMETRIEST TELEMETRIES;
extern ONOFFT ONOFF;
extern SETPOINTST SETPOINTS;
extern POSITIONST POSITIONS;
extern CONTROLLERST CONTROLLERS;
extern LANDINGT LANDING;
extern COMMANDST COMMANDS;

void constInit();



#endif /* CONSTANTS_H_ */