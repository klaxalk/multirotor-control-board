#ifndef SEND_H
#define SEND_H
#include "const.h"
//(kopter KOPTERS.XXX)

//turning on and off sending telemetry type to coordinator (type TELEMETRIES.XXX)
void telemetryGet(unsigned char kopter,unsigned char type,unsigned char on);
//if telemetry type sending is on or off  (type TELEMETRIES.XXX)
void telemetryStatus(unsigned char kopter,unsigned char type);
//land with drone (on ONOFF.ON/OFF)
void land(unsigned char kopter,unsigned char on);
//drones landing status
void landStatus(unsigned char kopter);
//turning on and off trajectory following (on ONOFF.ON/OFF)
void trajectoryFollow(unsigned char kopter,unsigned char on);
//if trajectory following is on or off
void trajectoryFollowStatus(unsigned char kopter);
//add waypoint to trajectory
void trajectoryAddPoint(unsigned char kopter,unsigned char index,float time,float elevatorPos,float aileronPos,float throttlePos);
//ask drone for waypoints
void trajectoryPointStatus(unsigned char kopter);
//set drones desired setpoint  (type SETPOINTS.XXX incType POSITIONS.RELATIVE/ABSOLUTE)
void setSetpoint(unsigned char kopter,unsigned char type,unsigned char incType,float value);
//get desired setpoint value (type SETPOINTS.XXX)
void setpointValue(unsigned char kopter,unsigned char type);
//turn on selected controller (option CONTROLLERS.XXX)
void setController(unsigned char kopter,unsigned char option);
//get controller status
void controllerStatus(unsigned char kopter);
//turn on and off gumstix (on ONOFF.ON/OFF)
void setGumstix(unsigned char kopter,unsigned char on);
//if gumstix is on or off
void gumstixStatus(unsigned char kopter);


#endif /*SEND_H*/
