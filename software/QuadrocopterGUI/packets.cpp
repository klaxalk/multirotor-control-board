#include <stdio.h>
#include "packets.h"
#include "defines.h"
#include "commands.h"
#include "XBEEComm.h"
#include "serial.h"
#include "openLog.h"
#include "const.h"

//addresses to other XBees

//values for telemetry types
unsigned char packet[255]={0};

void packetHandler(unsigned char *inPacket){
    unsigned char address16[2];
    unsigned char address64[8];
    unsigned char recieveOptions[5];
    unsigned char dataIN[255];
    int i;

	float *f1;
	float *f2;
	float *f3;
	float *f4;
	char ch1[4];
	char ch2[4];
	char ch3[4];
	char ch4[4];
	unsigned char usc;

    switch ((int)*(inPacket+3)) {
    //Modem Status
    case 0x8A:
        break;
    //Transmit Status
    case 0x8B:
        parTSPacket(inPacket,recieveOptions,address16,recieveOptions+1,recieveOptions+2,recieveOptions+3);
        break;
    //Route record indicator
    case 0xA1:
        break;
    //Receive Packet
    case 0x90:
            parReceivePacket(inPacket,address64,address16,recieveOptions,dataIN);
            //*data is data length
            switch ((int)*(dataIN+1)){
                //command
                case 'c':
                    break;
                //telemetry
                case 't':
                    for (i=0;i<(*dataIN-1)/5;i++){
                           usc=*(dataIN+2+i*5);
						ch1[0]=*(dataIN+3+i*5);
						ch1[1]=*(dataIN+4+i*5);
						ch1[2]=*(dataIN+5+i*5);
						ch1[3]=*(dataIN+6+i*5);
						f1=(float *)ch1;
						telemetryReceive(address64,address16,usc,*f1);
                    }
                    break;
                //report
                case 'r':
                    //TELEMETRY TO COORDINATOR STATUS
                    if(*(dataIN+2)==COMMANDS.TELEMETRY_COORDINATOR){
                        telemetryToCoordinatorReportRecieved(address64,address16,*(dataIN+3),*(dataIN+4));
                    } else
                     //LANDING STATUS
                    if(*(dataIN+2)==COMMANDS.LANDING){
                        kopterLandReportRecieved(address64,address16,*(dataIN+3));
                    }else
                    //TRAJECTORY FOLLOW STATUS
                    if(*(dataIN+2)==COMMANDS.TRAJECTORY_FOLLOW){
					    kopterTrajectoryReportRecieved(address64,address16,*(dataIN+3));
				    } else
					 //TRAJECTORY POINTS
				    if(*(dataIN+2)==COMMANDS.TRAJECTORY_POINTS){
                         ch1[0]=*(dataIN+4); ch1[1]=*(dataIN+5); ch1[2]=*(dataIN+6); ch1[3]=*(dataIN+7); f1=(float *)ch1;
						 ch2[0]=*(dataIN+8); ch2[1]=*(dataIN+9); ch2[2]=*(dataIN+10); ch2[3]=*(dataIN+11); f2=(float *)ch2;
						 ch3[0]=*(dataIN+12); ch3[1]=*(dataIN+13); ch3[2]=*(dataIN+14); ch3[3]=*(dataIN+15); f3=(float *)ch3;
						 ch4[0]=*(dataIN+16); ch4[1]=*(dataIN+17); ch4[2]=*(dataIN+18); ch4[3]=*(dataIN+19); f4=(float *)ch4;
						 kopterTrajectoryPointReportReceived(address64,address16,*(dataIN+3),*f1,*f2,*f3,*f4);
				    } else
                    //SETPOINTS STATUS
                    if(*(dataIN+2)==COMMANDS.SET_SETPOINTS){
						ch1[0]=*(dataIN+4);
						ch1[1]=*(dataIN+5);
						ch1[2]=*(dataIN+6);
						ch1[3]=*(dataIN+7);
						f1=(float *)ch1;
						kopterSetpointsReportReceived(address64,address16,*(dataIN+3),*f1);
                    }else
					 //CONTROLLERS STATUS
					 if(*(dataIN+2)==COMMANDS.CONTROLLERS){
						kopterControllersReportReceived(address64,address16,*(dataIN+3));
					 }else
					 //GUMSTIX STATUS
					 if(*(dataIN+2)==COMMANDS.GUMSTIX){
						 kopterGumstixReportRecieved(address64,address16,*(dataIN+3));
					 }
                    break;
                //warning
                case 'w':
                    break;
                //error
                case 'e':
                    break;
                default:
                        dataTypeError(address64,address16,dataIN);
                    break;
            }
        break;
    default:
           packetTypeError(inPacket);
        break;
    }
}


void strInsert(unsigned char *str,unsigned char start,unsigned char *ins, unsigned char insLength){
    int i;
    for (i=0 ; i<insLength ; i++){
            *(str+start+i)=*(ins+i);
    }
}

//create Transmit Request Packet 0x10
void makeTRPacket(unsigned char *adr64,unsigned char *adr16,unsigned char options,unsigned char frameID,unsigned char *data, unsigned char dataLength){
        unsigned char outPacket[60];
        int i=0;
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
        *(outPacket+18+dataLength)=0x00;
        sendPacket(outPacket);
}

//parse Modem Status 0x8A
void parMSPacket(unsigned char *inPacket,unsigned char *status){
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
void parTSPacket(unsigned char *inPacket,unsigned char *frameID,unsigned char *address16,unsigned char *TrRetryCount,unsigned char *deliveryStatus,unsigned char *discoveryStatus){
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
void parReceivePacket(unsigned char *inPacket,unsigned char *address64,unsigned char *address16,unsigned char *receiveOptions,unsigned char *dataIN){
    int i=0;
    for (i=0 ; i<8 ; i++){
        *(address64+i)=*(inPacket+4+i);
    }
    for (i=0 ; i<2 ; i++){
        *(address16+i)=*(inPacket+12+i);
    }
    *receiveOptions=*(inPacket+14);
    /*0x01 - Packet Acknowledged
    0x02 - Packet was a broadcast packet
    0x20 - Packet encrypted with APS encryption
    0x40 - Packet was sent from an end device (if known)*/

    //the *data is dataLength
    *dataIN=*(inPacket+2)-12;
    for (i=0 ; i<*dataIN ; i++){
        *(dataIN+i+1)=*(inPacket+15+i);
    }

}


