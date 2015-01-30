#ifndef COMMANDS_H
#define COMMANDS_H

float telemetryValue(unsigned char type);

void telemetrySend(unsigned char *address64,unsigned char *address16,unsigned char type,unsigned char frameID);
void telemetryRequest(unsigned char *address64,unsigned char *address16,unsigned char type, unsigned char frameID);
void telemetryReceive(unsigned char *address64,unsigned char *address16,unsigned char type,float value);

void telemetryToCoordinatorSend();
void telemetryToCoordinatorSet(unsigned char *address64,unsigned char *address16,unsigned char type,unsigned char on, unsigned char frameID);
void telemetryToCoordinator(unsigned char *address64,unsigned char *address16,unsigned char type,unsigned char on);
void telemetryToCoordinatorStatusRequest(unsigned char *address64,unsigned char *address16,unsigned char type,unsigned char frameID);
void telemetryToCoordinatorReport(unsigned char *address64,unsigned char *address16,unsigned char type,unsigned char frameID);
void telemetryToCoordinatorReportRecieved(unsigned char *address64,unsigned char *address16,unsigned char type,unsigned char status);

void dataTypeError(unsigned char *address64,unsigned char *address16,unsigned char *data);
void packetTypeError(unsigned char *inPacket);

void kopterLandRequest(unsigned char *address64,unsigned char *address16,unsigned char options,unsigned char frameID);
void kopterLand(unsigned char *address64,unsigned char *address16,unsigned char on);
void kopterLandStatusRequest(unsigned char *address64,unsigned char *address16,unsigned char frameID);
void kopterLandReport(unsigned char *address64,unsigned char *address16,unsigned char frameID);
void kopterLandReportRecieved(unsigned char *address64,unsigned char *address16,unsigned char status);

void kopterTrajectoryFollowRequest(unsigned char *address64,unsigned char *address16,unsigned char options,unsigned char frameID);
void kopterTrajectoryFollow(unsigned char *address64,unsigned char *address16,unsigned char on);
void kopterTrajectoryFollowStatusRequest(unsigned char *address64,unsigned char *address16,unsigned char frameID);
void kopterTrajectoryFollowReport(unsigned char *address64,unsigned char *address16,unsigned char frameID);
void kopterTrajectoryFollowReportRecieved(unsigned char *address64,unsigned char *address16,unsigned char status);

void kopterTrajectoryAddPointRequest(unsigned char *address64,unsigned char *address16,unsigned char index,float time,float elevatorPos,float aileronPos,float throttlePos,unsigned char frameID);
void kopterTrajectoryAddPoint(unsigned char *address64,unsigned char *address16,unsigned char index,float time,float elevatorPos,float aileronPos,float throttlePos);
void kopterTrajectoryPointStatusRequest(unsigned char *address64,unsigned char *address16,unsigned char frameID);
void kopterTrajectoryPointReport(unsigned char *address64,unsigned char *address16,unsigned char index,unsigned char frameID);
void kopterTrajectoryPointReportReceived(unsigned char *address64,unsigned char *address16,unsigned char index,float time,float elevatorPos,float aileronPos,float throttlePos);

void kopterSetpointsSetRequest(unsigned char *address64,unsigned char *address16,unsigned char type,unsigned char positionType,float value,unsigned char frameID);
void kopterSetpointsSet(unsigned char *address64,unsigned char *address16,unsigned char type,unsigned char positionType,float value);
void kopterSetpointStatusRequest(unsigned char *address64,unsigned char *address16,unsigned char type,unsigned char frameID);
void kopterSetpointsReport(unsigned char *address64,unsigned char *address16,unsigned char type,unsigned char frameID);
void kopterSetpointsReportReceived(unsigned char *address64,unsigned char *address16,unsigned char type,float value);

void kopterControllersRequest(unsigned char *address64,unsigned char *address16,unsigned char option,unsigned char frameID);
void kopterControllers(unsigned char *address64,unsigned char *address16,unsigned char option);
void kopterControllersStatusRequest(unsigned char *address64,unsigned char *address16,unsigned char frameID);
void kopterControllersReport(unsigned char *address64,unsigned char *address16,unsigned char frameID);
void kopterControllersReportReceived(unsigned char *address64,unsigned char *address16,unsigned char status);

void kopterFollowerSetRequest(unsigned char *address64,unsigned char *address16,unsigned char *followerAddr,unsigned char frameID);
void kopterFollowerSet(unsigned char *address64,unsigned char *address16,unsigned char *followerAddr);
void kopterFollowerSetStatusRequest(unsigned char *address64,unsigned char *address16,unsigned char frameID);
void kopterFollowerSetReport(unsigned char *address64,unsigned char *address16,unsigned char frameID);
void kopterFollowerSetReportRecieved(unsigned char *address64,unsigned char *address16,unsigned char *followerAddr);

void sendXBeeMessage(unsigned char *address64,unsigned char *address16,char *message,unsigned char frameID);
void receiveXBeeMessage(unsigned char *address64,unsigned char *address16,char *message);

void kopterLeadDataSend(unsigned char *address64,unsigned char *address16,volatile float altitude,volatile float elevatorVel,volatile float aileronVel,volatile float elevatorError,volatile float aileronError,unsigned char frameID);
void kopterLeadDataReceived(unsigned char *address64,unsigned char *address16,float altitude,float elevatorVel, float aileronVel, float elevatorError, float aileronError);


void kopterTimeRequest(unsigned char *address64,unsigned char *address16,int64_t time,unsigned char frameID);
void kopterTime(unsigned char *address64,unsigned char *address16,int64_t time);
void kopterTimeStatusRequest(unsigned char *address64,unsigned char *address16,unsigned char frameID);
void kopterTimeReport(unsigned char *address64,unsigned char *address16,unsigned char frameID);
void kopterTimeReportReceived(unsigned char *address64,unsigned char *address16,int64_t time);

#endif /*COMMANDS_H*/
