#ifndef PACKETS_H
#define PACKETS_H

typedef struct
{
	unsigned char COORDINATOR[8];
	unsigned char KC1[8];
	unsigned char K1[8];
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
	unsigned char AILERON_VEL_CONTROLLER_OUTPUT;
	unsigned char ELEVATOR_VEL_CONTROLLER_OUTPUT;
	unsigned char AILERON_POS_CONTROLLER_OUTPUT;
	unsigned char ELEVATOR_POS_CONTROLLER_OUTPUT;
} TELEMETRIEST;

//Telemetry request options
typedef struct
{
	unsigned char SENDING_OFF;
	unsigned char SENDING_ON;
	unsigned char SENDING_ONCE;
	unsigned char SENDING_STATUS;
} TELREQOPTT;

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
	unsigned char BOTH;	
} CONTROLLERST;

typedef struct
{
	unsigned char LANDING;		
	unsigned char SET_SETPOINTS;
	unsigned char CONTROLLERS;
	unsigned char TRAJECTORY_FOLLOW;
	unsigned char TRAJECTORY_POINTS;
	unsigned char GUMSTIX;
}COMMANDST;

extern unsigned char GET_STATUS;

extern ADDRESST ADDRESS;
extern TELEMETRIEST TELEMETRIES;
extern TELREQOPTT TELREQOPT;
extern ONOFFT ONOFF;
extern SETPOINTST SETPOINTS;
extern POSITIONST POSITIONS;
extern CONTROLLERST CONTROLLERS;
extern COMMANDST COMMANDS;

void constInit();
void packetHandler(unsigned char *inPacket);
void makeTRPacket(unsigned char *adr64,unsigned char *adr16,unsigned char options,unsigned char frameID,unsigned char *data, unsigned char dataLength);

#endif /*PACKETS_H*/
