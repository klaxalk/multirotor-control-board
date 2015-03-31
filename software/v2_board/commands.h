 #ifndef COMMANDS_H
#define COMMANDS_H

float telemetryValue(unsigned char type);
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

void kopterTrajectorySetRequest(unsigned char *address64,unsigned char *address16,unsigned char size,uint32_t* time,float* elevatorPos,float* aileronPos,float* throttlePos,unsigned char frameID);
void kopterTrajectorySet(unsigned char *address64,unsigned char *address16,unsigned char index,uint32_t time,float elevatorPos,float aileronPos,float throttlePos);
void kopterTrajectorySetStatusRequest(unsigned char *address64,unsigned char *address16,unsigned char frameID);
void kopterTrajectorySetReport(unsigned char *address64,unsigned char *address16,unsigned char index,unsigned char frameID);
void kopterTrajectorySetReportReceived(unsigned char *address64,unsigned char *address16,unsigned char index,uint32_t time,float elevatorPos,float aileronPos,float throttlePos);

void kopterControllersRequest(unsigned char *address64,unsigned char *address16,unsigned char option,unsigned char frameID);
void kopterControllers(unsigned char *address64,unsigned char *address16,unsigned char option);
void kopterControllersStatusRequest(unsigned char *address64,unsigned char *address16,unsigned char frameID);
void kopterControllersReport(unsigned char *address64,unsigned char *address16,unsigned char frameID);
void kopterControllersReportReceived(unsigned char *address64,unsigned char *address16,unsigned char status);

void kopterPositionSlaveSetRequest(unsigned char *address64,unsigned char *address16,unsigned char *slaveAddr,unsigned char frameID);
void kopterPositionSlaveSet(unsigned char *address64,unsigned char *address16,unsigned char *slaveAddr);
void kopterPositionSlaveSetStatusRequest(unsigned char *address64,unsigned char *address16,unsigned char frameID);
void kopterPositionSlaveSetReport(unsigned char *address64,unsigned char *address16,unsigned char frameID);
void kopterPositionSlaveSetReportRecieved(unsigned char *address64,unsigned char *address16,unsigned char *slaveAddr);

void sendXBeeMessage(unsigned char *address64,unsigned char *address16,char *message,unsigned char frameID);
void receiveXBeeMessage(unsigned char *address64,unsigned char *address16,char *message);

void kopterTimeRequest(unsigned char *address64,unsigned char *address16,uint32_t time,unsigned char frameID);
void kopterTime(unsigned char *address64,unsigned char *address16,uint32_t time);
void kopterTimeStatusRequest(unsigned char *address64,unsigned char *address16,unsigned char frameID);
void kopterTimeReport(unsigned char *address64,unsigned char *address16,unsigned char frameID);
void kopterTimeReportReceived(unsigned char *address64,unsigned char *address16,uint32_t time);

void kopterPositionSetRequest(unsigned char *address64,unsigned char *address16,float elevator,float aileron,unsigned char frameID);
void kopterPositionSet(unsigned char *address64,unsigned char *address16,float elevator,float aileron);

#endif /*COMMANDS_H*/
