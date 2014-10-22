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
	unsigned char GROUND_DISTANCE;
	unsigned char ELEVATOR_SPEED;
	unsigned char AILERON_SPEED;
	unsigned char ELEVATOR_SPEED_ESTIMATED;
	unsigned char AILERON_SPEED_ESTIMATED;
	unsigned char ELEVATOR_POS_ESTIMATED;
	unsigned char AILERON_POS_ESTIMATED;		
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
	unsigned char LAND_ON;
	unsigned char LAND_OFF;
} LANDINGT;

typedef struct
{
	unsigned char FOLLOW;
	unsigned char NOT_FOLLOW;
} TRAJECTORYT;

typedef struct
{
	unsigned char THROTTLE;
	unsigned char ELEVATOR;	
	unsigned char AILERON;	
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
} CONTROLLERST;

typedef struct
{
	unsigned char ARM;
	unsigned char DISARM;
} GESTUREST;

typedef struct
{
	unsigned char LANDING;		
	unsigned char SET_SETPOINTS;
	unsigned char CONTROLLERS;
	unsigned char GESTURES;
	unsigned char TRAJECTORY;
	unsigned char TRAJECTORY_POINTS;
}COMMANDST;

extern unsigned char GET_STATUS;

extern ADDRESST ADDRESS;
extern TELEMETRIEST TELEMETRIES;
extern TELREQOPTT TELREQOPT;
extern LANDINGT LANDING;
extern TRAJECTORYT TRAJECTORY;
extern SETPOINTST SETPOINTS;
extern POSITIONST POSITIONS;
extern CONTROLLERST CONTROLLERS;
extern GESTUREST GESTURES;
extern COMMANDST COMMANDS;

void constInit();

void packetHandler(unsigned char *inPacket);
//create Transmit Request Packet 0x10
void makeTRPacket(unsigned char *adr64,unsigned char *adr16,unsigned char options,unsigned char frameID,unsigned char *data, unsigned char dataLength);
//parse Modem Status 0x8A
void parMSPacket(unsigned char *inPacket,unsigned char *status);
//parse Transmit Status 0x8B
void parTSPacket(unsigned char *inPacket,unsigned char *frameID,unsigned char *address16,unsigned char *TrRetryCount,unsigned char *deliveryStatus,unsigned char *discoveryStatus);
//parse Receive Packet 0x90
void parReceivePacket(unsigned char *inPacket,unsigned char *address64,unsigned char *address16,unsigned char *receiveOptions,unsigned char *data);

#endif /*PACKETS_H*/
