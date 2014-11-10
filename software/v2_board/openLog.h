/*
 * openLog.h
 *
 * Created: 30. 10. 2014 16:38:31
 *  Author: Martin
 */ 


#ifndef OPENLOG_H_
#define OPENLOG_H_


//void availableLogging(uint8_t ID);
void startLogging(char fileName[]);
void stopLogging();
void loggingData();
void setTelemetry();


#endif /* OPENLOG_H_ */