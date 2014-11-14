#ifndef OPENLOG_H
#define OPENLOG_H

void openLogRequest(unsigned char *address64,unsigned char *address16,unsigned char *data,unsigned char frameID);
void openLogReceive(unsigned char *address64,unsigned char *address16,unsigned char *data);
#endif /* OPENLOG_H */
