#include <stdio.h>
#include "openLog.h"
#include "packets.h"

void openLogRequest(unsigned char *address64,unsigned char *address16,unsigned char *data,unsigned char frameID){
	unsigned char dataLength = *(data);
	*(data)='o';
	makeTRPacket(address64,address16,0x00,frameID,data,dataLength+1);
}

void openLogReceive(unsigned char *address64,unsigned char *address16,unsigned char *data){
	//1st byte - data length
	//2nd byte - 'o' (ignore)
}
