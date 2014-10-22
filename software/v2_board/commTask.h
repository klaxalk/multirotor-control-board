/*
 * communicationTask.h
 *
 * Created: 11.9.2014 11:17:48
 *  Author: Tomas Baca
 */

#ifndef COMMTASK_H_
#define COMMTASK_H_

// the communication task
void commTask(void *p);
void sendXBeePacket(unsigned char *packet);
void sendPacketUART(unsigned char *packet);

#endif /* COMMTASK_H_ */
