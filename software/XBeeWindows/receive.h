#ifndef RECEIVE_H
#define RECEIVE_H

#include "const.h"

float getTelemetry(unsigned char kopter, unsigned char type);
unsigned char getStatus(unsigned char kopter, unsigned char type);

void receivedTelemetry(unsigned char kopter,unsigned char type,float value);
void receivedTelToCoordReport(unsigned char kopter,unsigned char type,unsigned char status);
void receivedLandingReport(unsigned char kopter,unsigned char status);
void receivedTrajectoryFollowReport(unsigned char kopter,unsigned char status);
void receivedTrajectoryPointReport(unsigned char kopter,unsigned char index,float time,float elevatorPos,float aileronPos,float throttlePos);
void receivedDesiredSetpointReport(unsigned char kopter,unsigned char type,float value);
void receivedControllerReport(unsigned char kopter,unsigned char status);
void receivedGumstixReport(unsigned char kopter,unsigned char status);
void receivedXBeeMessage(unsigned char kopter,char * message);

#endif /*RECEIVE_H*/
