#include <stdio.h>
#include "system.h"
#include "usart_driver_RTOS.h"
#include <string.h>
#include "ioport.h"
#include "packets.h"
#include "openLog.h"

extern volatile float estimatedThrottlePos;							// TELEMETRIES.GROUND_DISTANCE_ESTIMATED
extern volatile float groundDistance;								// TELEMETRIES.GROUND_DISTANCE
extern volatile float elevatorSpeed;								// TELEMETRIES.ELEVATOR_SPEED
extern volatile float aileronSpeed;									// TELEMETRIES.AILERON_SPEED
extern volatile float estimatedElevatorVel;							// TELEMETRIES.ELEVATOR_SPEED_ESTIMATED
extern volatile float estimatedAileronVel;							// TELEMETRIES.AILERON_SPEED_ESTIMATED
extern volatile float estimatedElevatorPos;							// TELEMETRIES.ELEVATOR_POS_ESTIMATED
extern volatile float estimatedAileronPos;							// TELEMETRIES.AILERON_POS_ESTIMATED
extern volatile float controllerThrottleOutput;						// TELEMETRIES.THROTTLE_CONTROLLER_OUTPUT
extern volatile float estimatedThrottleVel;							// TELEMETRIES.THROTTLE_SPEED
extern volatile float velocityControllerAileronOutput;				// TELEMETRIES.AILERON_VEL_CONTROLLER_OUTPUT
extern volatile float velocityControllerElevatorOutput;				// TELEMETRIES.ELEVATOR_VEL_CONTROLLER_OUTPUT
extern volatile float positionControllerAileronOutput;				// TELEMETRIES.AILERON_POS_CONTROLLER_OUTPUT
extern volatile float positionControllerElevatorOutput;				// TELEMETRIES.ELEVATOR_POS_CONTROLLER_OUTPUT
extern volatile float throttleSetpoint;								// TELEMETRIES.THROTTLE_SETPOINT
extern volatile float elevatorPositionSetpoint;						// TELEMETRIES.ELEVATOR_POS_SETPOINT
extern volatile float aileronPositionSetpoint;						// TELEMETRIES.AILERON_POS_SETPOINT
extern volatile float elevatorVelocitySetpoint;						// TELEMETRIES.ELEVATOR_VEL_SETPOINT
extern volatile float aileronVelocitySetpoint;						// TELEMETRIES.AILERON_VEL_SETPOINT
extern volatile float estimatedElevatorVel2;						// TELEMETRIES.ELEVATOR_SPEED_ESTIMATED2
extern volatile float estimatedAileronVel2;							// TELEMETRIES.AILERON_SPEED_ESTIMATED2
extern volatile float estimatedElevatorAcc;							// TELEMETRIES.ELEVATOR_ACC
extern volatile float estimatedAileronAcc;							// TELEMETRIES.AILERON_ACC
extern volatile float validGumstix;									// TELEMETRIES.VALID_GUMSTIX
extern volatile float elevatorDesiredSpeedPosController;			// TELEMETRIES.ELEVATOR_DESIRED_SPEED_POS_CONT
extern volatile float aileronDesiredSpeedPosController;				// TELEMETRIES.AILERON_DESIRED_SPEED_POS_CONT
extern volatile float elevatorDesiredSpeedPosControllerLeader;		// TELEMETRIES.ELE_DES_SPEED_POS_CONT_LEADER
extern volatile float aileronDesiredSpeedPosControllerLeader;		// TELEMETRIES.AIL_DES_SPEED_POS_CONT_LEADER
extern int16_t outputThrottle;										// TELEMETRIES.OUTPUT_THROTTLE
extern int16_t outputElevator;										// TELEMETRIES.OUTPUT_ELEVATOR
extern int16_t outputAileron;										// TELEMETRIES.OUTPUT_AILERON
extern int16_t outputRudder;										// TELEMETRIES.OUTPUT_RUDDER




extern volatile float opticalFlowData;
extern volatile uint16_t outputChannels[6];


extern UsartBuffer * usart_buffer_log;
char fileName[12];
int i=0;

void openLogRequest(unsigned char *address64,unsigned char *address16,unsigned char *data,unsigned char frameID){
	unsigned char dataLength = *(data);
	*(data)='o';
	makeTRPacket(address64,address16,0x00,frameID,data,dataLength+1);
}

// The method for changing name through xbee
void openLogReceive(unsigned char *address64,unsigned char *address16,unsigned char *data){
	//1st byte - data length
	//2nd byte - 'o' (ignore)
	char fName[32];
	int i;																// number of log
	sprintf(fName,"%c",*(data+2));
	for (i=0;i<=(*(data)-3);i++)
	{sprintf(fName,"%s%c",fName,*(data+3+i));
		usartBufferPutByte(usart_buffer_4,*(data+3+i),0);
	}
	
	startLogging(fName);
}

void stopLogging(){
	usartBufferPutString(usart_buffer_log,"$$$",0);						//$$$ - characters for stop logging
	//
	vTaskDelay(200);
}

void startLogging(char * newFileName){
	stopLogging();
	i=0;																// definition of recording
	if(strlen(newFileName)>12) strcpy (fileName,"log.txt");				// name can be up to 12 characters
	else strcpy (fileName,newFileName);
	strcpy (fileName,strupr(fileName));									// name must be in uppercase
	char str[64];
	sprintf(str, "\rappend %s\rStart logging:\n",fileName);				// command for start logging
	usartBufferPutString(usart_buffer_log,str,0);						// send it to the log
}

void loggingData(){
	i++;
	char str[256];
	// string of variables
	sprintf(str, "%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d,%d,%d\n",
	i,estimatedThrottlePos,groundDistance,elevatorSpeed,aileronSpeed,estimatedElevatorVel,estimatedAileronVel,estimatedElevatorPos,estimatedAileronPos,
	controllerThrottleOutput,estimatedThrottleVel,velocityControllerAileronOutput,velocityControllerElevatorOutput,positionControllerAileronOutput,positionControllerElevatorOutput,
	throttleSetpoint,elevatorPositionSetpoint,aileronPositionSetpoint,elevatorVelocitySetpoint,aileronVelocitySetpoint,estimatedElevatorVel2,estimatedAileronVel2,estimatedElevatorAcc,
	estimatedAileronAcc,validGumstix,elevatorDesiredSpeedPosController,aileronDesiredSpeedPosController,elevatorDesiredSpeedPosControllerLeader,aileronDesiredSpeedPosControllerLeader,
	outputThrottle,outputElevator,outputAileron,outputRudder);
	usartBufferPutString(usart_buffer_log,str,0);
}

void setTelemetry(){
	stopLogging();
	
	//prikazy pro read
	//nedodelany, zalezi, co bude potreba
	
	startLogging(fileName);
}