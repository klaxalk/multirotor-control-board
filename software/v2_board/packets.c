#include <stdio.h>
#include "packets.h"



char outPacket[60];
char inPacket[60];
int i=0;

void strInsert(char *str,int start, char *ins, int insLength){
    for (i=0 ; i<insLength ; i++){
            *(str+start+i)=*(ins+i);
    }
}

//recognize packet
void recPacket(){
    switch (*(inPacket+3)) {
    //Modem Status
    case 0x8A:

        break;
    //Transmit Status
    case 0x8B:

        break;
    //Receive Packet
    case 0x90:

        break;
    default:
            printf("Response to Frame Type: %X is not implemented yet.",*(inPacket+3));
        break;
    }

}


//create Transmit Request Packet 0x10
void makeTRPacket(char *adr64, char *adr16, char options, char frameID, char *data, int dataLength){
    //initial byte
    *outPacket=0x7E;
    //length
    *(outPacket+1)=0x00;
    *(outPacket+2)=14+dataLength;
    //frame type
    *(outPacket+3)=0x10;
    //frame ID
    *(outPacket+4)=frameID;
    //addresses
    strInsert(outPacket,5,adr64,8);
    strInsert(outPacket,13,adr16,2);
    //Broadcast Radius
    *(outPacket+15)=0x00;
    /*Options
    0x01 - Disable retries and route repair
    0x20 - Enable APS encryption (if EE=1)
    0x40 - Use the extended transmission timeout*/
    *(outPacket+16)=options;
    //data
    strInsert(outPacket,17,data,dataLength);
    //checksum
    *(outPacket+17+dataLength)=0xFF;
    for (i=3; i<17+dataLength; i++){
        *(outPacket+17+dataLength)=*(outPacket+17+dataLength)-*(outPacket+i);
    }
    strInsert(outPacket,18+dataLength,"",1);
}

//parse Modem Status 0x8A
void parMSPacket(char *status){
    *status=*(inPacket+4);
    /*0 = Hardware reset
    1 = Watchdog timer reset
    2 =Joined network (routers and end devices)
    3 =Disassociated
    6 =Coordinator started
    7 = Network security key was updated
    0x0D = Voltage supply limit exceeded (PRO S2B only)
    0x11 = Modem configuration changed while join in progress
    0x80+ = stack error*/
}

//parse Transmit Status 0x8B
void parTSPacket(char *frameID, char *address16,char *TrRetryCount, char *deliveryStatus, char *discoveryStatus){
    *frameID=*(inPacket+4);
    *address16=*(inPacket+5);
    *(address16+1)=*(inPacket+6);
    *TrRetryCount=*(inPacket+7);
    *deliveryStatus=*(inPacket+8);
    /*0x00 = Success
    0x01 = MAC ACK Failure
    0x02 = CCA Failure
    0x15 = Invalid destination endpoint
    0x21 = Network ACK Failure
    0x22 = Not Joined to Network
    0x23 = Self-addressed
    0x24 = Address Not Found
    0x25 = Route Not Found
    0x26 = Broadcast source failed to hear a neighbor relay the message
    0x2B = Invalid binding table index
    0x2C = Resource error lack of free buffers, timers, etc.
    0x2D = Attempted broadcast with APS transmission
    0x2E = Attempted unicast with APS transmission, but EE=0
    0x32 = Resource error lack of free buffers, timers, etc.
    0x74 = Data payload too large*/
    *discoveryStatus=*(inPacket+9);
    /*0x00 = No Discovery Overhead
    0x01 = Address Discovery
    0x02 = Route Discovery
    0x03 = Address and Route
    0x40 = Extended Timeout Discovery*/
}

//parse Receive Packet 0x90
void parReceivePacket(char *address64, char *address16, char *receiveOptions, char *data, int *dataLength){
    for (i=0 ; i<8 ; i++){
        *(address64+i)=*(inPacket+4+i);
    }
    for (i=0 ; i<2 ; i++){
        *(address16+i)=*(inPacket+12+i);
    }
    *receiveOptions=*(inPacket+14);
    *dataLength=*(inPacket+2)-12;
    for (i=0 ; i<*dataLength ; i++){
        *(data+i)=*(inPacket+15+i);
    }

}

/*
int main(){
	printf("Zadej retezec:");
	scanf("%s",str+2);
	printf("%s",str);
}
*/
