#include <stdio.h>
#include "packets.h"
#include "defines.h"
#include "commands.h"
#include "communication.h"
#include "controllers.h"
#include "usart_driver_RTOS.h"
#include "system.h"

//Telemetry sending
volatile unsigned char telemetryToCoordinatorArr[TELEMETRY_VARIABLES]={0};



void makeTRPacket(unsigned char *adr64,unsigned char *adr16,unsigned char options,unsigned char frameID,unsigned char *data, unsigned char dataLength);
void parMSPacket(unsigned char *inPacket,unsigned char *status);
void parTSPacket(unsigned char *inPacket,unsigned char *frameID,unsigned char *address16,unsigned char *TrRetryCount,unsigned char *deliveryStatus,unsigned char *discoveryStatus);
void parReceivePacket(unsigned char *inPacket,unsigned char *address64,unsigned char *address16,unsigned char *receiveOptions,unsigned char *data);

void adr64Setter(unsigned char * adr,unsigned char b1,unsigned char b2,unsigned char b3,unsigned char b4,unsigned char b5,unsigned char b6,unsigned char b7,unsigned char b8){
    *adr=b1; *(adr+1)=b2; *(adr+2)=b3; *(adr+3)=b4; *(adr+4)=b5; *(adr+5)=b6; *(adr+6)=b7; *(adr+7)=b8;
}
void adr64Setter2(unsigned char * adr,unsigned char * adr2){
	int i;
	for(i=0;i<8;i++){
		*(adr+i)=*(adr2+i);
	}
}


void packetHandler(unsigned char *inPacket){
    unsigned char address16[2];
    unsigned char address64[8];
    unsigned char recieveOptions[5];
    unsigned char dataIN[25];

	
	float *f1;
	float *f2;
	float *f3;
	float *f4;
	uint8_t i;
	uint32_t *ui32;
	char ch1[4];
	char ch2[4];
	char ch3[4];
	char ch4[4];	
	unsigned char usc;
    switch ((int)*(inPacket+3)) {
    //Receive Packet
    case 0x90:
            parReceivePacket(inPacket,address64,address16,recieveOptions,dataIN);			
            //*data is data length				
            switch ((int)*(dataIN+1)){
                //command
                case 'c':
                        //TELEMETRY REQUESTS						
                        if(*(dataIN+2)==COMMANDS.TELEMETRY){                            					            
                            telemetrySend(address64,address16,*(dataIN+3),0x00);                            
                        }else
						//TELEMETRY TO COORDINATOR
						 if(*(dataIN+2)==COMMANDS.TELEMETRY_COORDINATOR){  
							if(*(dataIN+3)==GET_STATUS){
								telemetryToCoordinatorReport(address64,address16,*(dataIN+4),0x00);
							}else{
								telemetryToCoordinator(address64,address16,*(dataIN+3),*(dataIN+4));
							}
						}else
						//LANDING REQUEST
						if(*(dataIN+2)==COMMANDS.LANDING){
							if(		 *(dataIN+3)==ONOFF.ON){
								kopterLand(address64,address16,1);								
							}else if(*(dataIN+3)==ONOFF.OFF){
								kopterLand(address64,address16,0);								
							}else if(*(dataIN+3)==GET_STATUS){
								kopterLandReport(address64,address16,0x00);								
							}
						} else
						//TRAJECTORY REQUEST
						if(*(dataIN+2)==COMMANDS.TRAJECTORY_POINTS){
							if(*(dataIN+3)==GET_STATUS){																
								for(i=0;i<=trajMaxIndex;i++){
										kopterTrajectoryPointReport(address64,address16,i,0x00);
								}								
							}else{								
								ch1[0]=*(dataIN+4); ch1[1]=*(dataIN+5); ch1[2]=*(dataIN+6); ch1[3]=*(dataIN+7); ui32=(uint32_t *)ch1;
								ch2[0]=*(dataIN+8); ch2[1]=*(dataIN+9); ch2[2]=*(dataIN+10); ch2[3]=*(dataIN+11); f2=(float *)ch2;
								ch3[0]=*(dataIN+12); ch3[1]=*(dataIN+13); ch3[2]=*(dataIN+14); ch3[3]=*(dataIN+15); f3=(float *)ch3;
								ch4[0]=*(dataIN+16); ch4[1]=*(dataIN+17); ch4[2]=*(dataIN+18); ch4[3]=*(dataIN+19); f4=(float *)ch4;
								kopterTrajectoryAddPoint(address64,address16,*(dataIN+3),*ui32,*f2,*f3,*f4);																															
							}
						} else																		
						//SETPOINTS REQUEST
						if(*(dataIN+2)==COMMANDS.SET_SETPOINTS){
							if(*(dataIN+3)==GET_STATUS){
								kopterSetpointsReport(address64,address16,*(dataIN+4),0x00);
							}else{
								ch1[0]=*(dataIN+5);
								ch1[1]=*(dataIN+6);
								ch1[2]=*(dataIN+7);
								ch1[3]=*(dataIN+8);
								f1=(float *)ch1;
								kopterSetpointsSet(address64,address64,*(dataIN+3),*(dataIN+4),*f1);
							}
						}else
						//CONTROLLERS
						if(*(dataIN+2)==COMMANDS.CONTROLLERS){
							if(*(dataIN+3)==GET_STATUS){
								kopterControllersReport(address64,address16,0x00);
							}else{
								kopterControllers(address64,address16,*(dataIN+3));
							}															
						}else
						//FOLLOWER SET
						if(*(dataIN+2)==COMMANDS.FOLLOWER_SET){
							if(*(dataIN+3)==GET_STATUS){
								kopterFollowerSetReport(address64,address16,0x00);
							}else{
								kopterFollowerSet(address64,address16,(dataIN+3));
							}
						}else
						//TIME
						if(*(dataIN+2)==COMMANDS.TIME){
							if(*(dataIN+3)==GET_STATUS){
								kopterTimeReport(address64,address16,0x00);
							}else{
								ch1[0]=*(dataIN+3);
								ch1[1]=*(dataIN+4);
								ch1[2]=*(dataIN+5);
								ch1[3]=*(dataIN+6);
								ui32=(uint32_t *)ch1;
								kopterTime(address64,address16,*ui32);
							}
						}else
						//LOCK ON BLOB
						if(*(dataIN+2)==COMMANDS.LOCK_ON_BLOB){
							if(*(dataIN+3)==GET_STATUS){
								kopterLockOnBlobReport(address64,address16,0x00);
							}else{
								ch1[0]=*(dataIN+3);
								ch1[1]=*(dataIN+4);
								ch1[2]=*(dataIN+5);
								ch1[3]=*(dataIN+6);
								f1=(float *)ch1;
								kopterLockOnBlob(address64,address16,*f1);
							}
						}				
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
				         } else
						 //TRAJECTORY POINTS
				         if(*(dataIN+2)==COMMANDS.TRAJECTORY_POINTS){							 
							 ch1[0]=*(dataIN+4); ch1[1]=*(dataIN+5); ch1[2]=*(dataIN+6); ch1[3]=*(dataIN+7); ui32=(uint32_t *)ch1;
							 ch2[0]=*(dataIN+8); ch2[1]=*(dataIN+9); ch2[2]=*(dataIN+10); ch2[3]=*(dataIN+11); f2=(float *)ch2;
							 ch3[0]=*(dataIN+12); ch3[1]=*(dataIN+13); ch3[2]=*(dataIN+14); ch3[3]=*(dataIN+15); f3=(float *)ch3;
							 ch4[0]=*(dataIN+16); ch4[1]=*(dataIN+17); ch4[2]=*(dataIN+18); ch4[3]=*(dataIN+19); f4=(float *)ch4;
							 kopterTrajectoryPointReportReceived(address64,address16,*(dataIN+3),*ui32,*f2,*f3,*f4);
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
						 //TIME STATUS
						 if(*(dataIN+2)==COMMANDS.TIME){
							ch1[0]=*(dataIN+3);
							ch1[1]=*(dataIN+4);
							ch1[2]=*(dataIN+5);
							ch1[3]=*(dataIN+6);
							ui32=(uint32_t *)ch1;							 
							kopterTimeReportReceived(address64,address16,*ui32);
						}else
						//LOCK ON BLOB
						if(*(dataIN+2)==COMMANDS.LOCK_ON_BLOB){						
							ch1[0]=*(dataIN+3);
							ch1[1]=*(dataIN+4);
							ch1[2]=*(dataIN+5);
							ch1[3]=*(dataIN+6);
							f1=(float *)ch1;
							kopterLockOnBlobReportRecieved(address64,address16,*f1);					
						}				 
                    break;
                //warning
                case 'w':
                    break;
                //message
                case 'm':                   
                    break;	
				//leading	
				case 'l':
						leadFreshTimer=0;
						ch1[0]=*(dataIN+2); ch1[1]=*(dataIN+3); ch1[2]=*(dataIN+4); ch1[3]=*(dataIN+5); f1=(float *)ch1;
						ch2[0]=*(dataIN+6); ch2[1]=*(dataIN+7); ch2[2]=*(dataIN+8); ch2[3]=*(dataIN+9); f2=(float *)ch2;
						ch3[0]=*(dataIN+10); ch3[1]=*(dataIN+11); ch3[2]=*(dataIN+12); ch3[3]=*(dataIN+13); f3=(float *)ch3;
						kopterLeadDataReceived(address64,address16,*f1,*f2,*f3);
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
    uint8_t i;
    for (i=0 ; i<insLength ; i++){
            *(str+start+i)=*(ins+i);
    }
}

//create Transmit Request Packet 0x10
void makeTRPacket(unsigned char *adr64,unsigned char *adr16,unsigned char options,unsigned char frameID,unsigned char *data, unsigned char dataLength){
        unsigned char outPacket[256];
        uint8_t i=0;
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
        sendXBeePacket(outPacket);
}

//parse Receive Packet 0x90
void parReceivePacket(unsigned char *inPacket,unsigned char *address64,unsigned char *address16,unsigned char *receiveOptions,unsigned char *data){
    uint8_t i=0;
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
    *data=*(inPacket+2)-12;
    for (i=0 ; i<*data ; i++){
        *(data+i+1)=*(inPacket+15+i);
    }
}


