#ifndef CONST_H
#define CONST_H

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
    unsigned char KC1;
    unsigned char K1;
    unsigned char K2;
    unsigned char K3;
    unsigned char UNKNOWN;
} KOPTERST;

#define KOPTERS_COUNT 4

/*typedef struct
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
    unsigned char THROTTLE_SETPOINT;
    unsigned char ELEVATOR_POS_SETPOINT;
    unsigned char AILERON_POS_SETPOINT;
    unsigned char ELEVATOR_VEL_SETPOINT;
    unsigned char AILERON_VEL_SETPOINT;
    unsigned char ELEVATOR_SPEED_ESTIMATED2;
    unsigned char AILERON_SPEED_ESTIMATED2;
    unsigned char ELEVATOR_ACC;
    unsigned char AILERON_ACC;
    unsigned char VALID_GUMSTIX;
    unsigned char ELEVATOR_DESIRED_SPEED_POS_CONT;
    unsigned char AILERON_DESIRED_SPEED_POS_CONT;
    unsigned char ELE_DES_SPEED_POS_CONT_LEADER;
    unsigned char AIL_DES_SPEED_POS_CONT_LEADER;
    unsigned char OUTPUT_THROTTLE;
    unsigned char OUTPUT_ELEVATOR;
    unsigned char OUTPUT_AILERON;
    unsigned char OUTPUT_RUDDER;
    unsigned char BLOB_DISTANCE;
    unsigned char BLOB_HORIZONTAL;
    unsigned char BLOB_VERTICAL;
    unsigned char PITCH_ANGLE;
    unsigned char ROLL_ANGLE;
} TELEMETRIEST;*/

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
    unsigned char ELEVATOR_SHIFT;
    unsigned char AILERON_SHIFT;
    unsigned char ELEVATOR_ACC_INPUT;
    unsigned char ELEVATOR_ACC_ERROR;
    unsigned char AILERON_ACC_INPUT;
    unsigned char AILERON_ACC_ERROR;
    unsigned char ALTITUDE_INTEGRATED_COMPONENT;

    unsigned char GROUND_DISTANCE_ESTIMATED;
    unsigned char GROUND_DISTANCE;
    unsigned char ELEVATOR_POS_ESTIMATED;
    unsigned char AILERON_POS_ESTIMATED;
    unsigned char THROTTLE_CONTROLLER_OUTPUT;
    unsigned char THROTTLE_SPEED;
    unsigned char AILERON_VEL_CONTROLLER_OUTPUT;
    unsigned char ELEVATOR_VEL_CONTROLLER_OUTPUT;
    unsigned char AILERON_POS_CONTROLLER_OUTPUT;
    unsigned char ELEVATOR_POS_CONTROLLER_OUTPUT;
    unsigned char THROTTLE_SETPOINT;
    unsigned char ELEVATOR_VEL_SETPOINT;
    unsigned char AILERON_VEL_SETPOINT;
    unsigned char ELEVATOR_SPEED_ESTIMATED2;
    unsigned char AILERON_SPEED_ESTIMATED2;
    unsigned char ELEVATOR_DESIRED_SPEED_POS_CONT;
    unsigned char AILERON_DESIRED_SPEED_POS_CONT;
    unsigned char ELE_DES_SPEED_POS_CONT_LEADER;
    unsigned char AIL_DES_SPEED_POS_CONT_LEADER;
    unsigned char BLOB_DISTANCE;
    unsigned char BLOB_HORIZONTAL;
    unsigned char BLOB_VERTICAL;
} TELEMETRIEST;

#define TELEMETRY_VARIABLES 34
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
    unsigned char POSITION_SLAVE_SET;
    unsigned char TIME;
    unsigned char POSITION_SET;
    unsigned char GUMSTIX;
    unsigned char TRAJECTORY_FOLLOW;
    unsigned char SET_SETPOINTS;
}COMMANDST;

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

extern unsigned char GET_STATUS;

extern ADDRESST ADDRESS;
extern TELEMETRIEST TELEMETRIES;
extern ONOFFT ONOFF;
extern CONTROLLERST CONTROLLERS;
extern LANDINGT LANDING;
extern COMMANDST COMMANDS;
extern SETPOINTST SETPOINTS;
extern POSITIONST POSITIONS;
extern KOPTERST KOPTERS;

void constInit();
#define REPORTS_COUNT 5

#endif /*CONST_H*/
