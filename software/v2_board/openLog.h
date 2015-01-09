/*
 * openLog.h
 *
 * Created: 30. 10. 2014 16:38:31
 *  Author: Martin
 */ 


#ifndef OPENLOG_H_
#define OPENLOG_H_


void openLogRequest(unsigned char *address64,unsigned char *address16,unsigned char *data,unsigned char frameID);
void openLogReceive(unsigned char *address64,unsigned char *address16,unsigned char *data);
void startLogging(char * fileName);
void stopLogging();
void loggingData();
void setTelemetry();


#endif /* OPENLOG_H_ */