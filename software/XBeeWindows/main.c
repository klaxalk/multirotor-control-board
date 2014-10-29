#include <stdio.h>
#include <windows.h>
#include "XBeeComm.h"
#include "packets.h"
#include "commands.h"
#include "serialLink.h"

HANDLE matlabH;

void main()
{
    int decison;
    float f;

    constInit();

    openXBeeComm(4);
    #ifdef MATLAB
    matlabH=openSerialLine(8);
    #endif // MATLAB

    do{
       printf("\n0 - EXIT\n1 - TELEMETRY\n2 - LANDING\n3 - SETPOINTS\n4 - CONTROLLERS\n5 - TRAJECTORY\n\n");
       printf("DECISION:");
       scanf("%d",&decison);
       switch(decison){
        case 0:
            printf("BYE\n");
            break;
        case 1:
            printf("\nTELEMTRY: 1 - GROUND DISTANCE   2 - ELEVATOR SPEED   3 - AILERON SPEED   4 - ELEVATOR POS   5 - AILERON POS   6 - THROTTLE CONT OUT\n");
            printf("DECISION:");
            scanf("%d",&decison);
                    switch(decison){
                        case 1:
                            telemetryRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,TELEMETRIES.GROUND_DISTANCE,TELREQOPT.SENDING_ONCE,0x01);
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            break;
                        case 2:
                            telemetryRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,TELEMETRIES.ELEVATOR_SPEED,TELREQOPT.SENDING_ONCE,0x01);
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            break;
                        case 3:
                            telemetryRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,TELEMETRIES.AILERON_SPEED,TELREQOPT.SENDING_ONCE,0x01);
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            break;
                        case 4:
                            telemetryRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,TELEMETRIES.ELEVATOR_POS_ESTIMATED,TELREQOPT.SENDING_ONCE,0x01);
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            break;
                        case 5:
                            telemetryRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,TELEMETRIES.AILERON_POS_ESTIMATED,TELREQOPT.SENDING_ONCE,0x01);
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            break;
                        case 6:
                            telemetryRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,TELEMETRIES.THROTTLE_CONTROLLER_OUTPUT,TELREQOPT.SENDING_ONCE,0x01);
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            break;
                        default:
                            printf("WRONG NUMBER\n");
                        break;
                    }
            break;
        case 2:
            printf("\nLANDING: 1 - ON LANDING   2 - OFF LANDING   3 - LANDING STATUS\n");
            printf("DECISION:");
            scanf("%d",&decison);
                    switch(decison){
                        case 1:
                            kopterLandRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,LANDING.LAND_ON,0x02);
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            break;
                        case 2:
                            kopterLandRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,LANDING.LAND_OFF,0x02);
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            break;
                        case 3:
                            kopterLandStatusRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,0x02);
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            break;
                        default:
                            printf("WRONG NUMBER\n");
                        break;
                    }
            break;
        case 3:
            printf("\nSETPOINTS (POS RELATIVE): 1 - SET    THROTTLE    2 - SET    ELEVATOR   3 - SET    AILERON\n                          4 - STATUS THROTTLE    5 - STATUS ELEVATOR   6 - STATUS AILERON\n");
            printf("DECISION:");
            scanf("%d",&decison);
            if (decison<4 && decison>0){
                printf("Set value in meters:");
                scanf("%f",&f);
            }
                    switch(decison){
                        case 1:
                            kopterSetpointsSetRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,SETPOINTS.THROTTLE,POSITIONS.ABSOLUT,f,0x02);
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            break;
                        case 2:
                            kopterSetpointsSetRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,SETPOINTS.ELEVATOR,POSITIONS.RELATIV,f,0x02);
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            break;
                        case 3:
                            kopterSetpointsSetRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,SETPOINTS.AILERON,POSITIONS.RELATIV,f,0x02);
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            break;
                        case 4:
                            kopterSetpointStatusRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,SETPOINTS.THROTTLE,0x02);
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            break;
                        case 5:
                            kopterSetpointStatusRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,SETPOINTS.ELEVATOR,0x02);
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            break;
                        case 6:
                            kopterSetpointStatusRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,SETPOINTS.AILERON,0x02);
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            break;
                        default:
                            printf("WRONG NUMBER\n");
                        break;
                    }
            break;
        case 4:
            printf("1 - OFF   2 - VELOCITY   3 - POSITION\n");
            printf("DECISION:");
            scanf("%d",&decison);
            switch(decison){
                    case 1:
                        kopterControllersRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,CONTROLLERS.OFF,0x13);
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                        break;
                    case 2:
                        kopterControllersRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,CONTROLLERS.VELOCITY,0x13);
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                        break;
                    case 3:
                        kopterControllersRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,CONTROLLERS.POSITION,0x13);
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                        break;
                    default:
                        printf("WRONG NUMBER\n");
                        break;
             }
             break;
        case 5:
            printf("1 - FOLLOW   2 - NOT FOLLOW   3 - UPLOAD   4 - SHOW\n");
            printf("DECISION:");
            scanf("%d",&decison);
            switch(decison){
                    case 1:
                        kopterTrajectoryRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,TRAJECTORY.FOLLOW,0x10);
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                        break;
                    case 2:
                        kopterTrajectoryRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,TRAJECTORY.NOT_FOLLOW,0x10);
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                        break;
                    case 3:
                        kopterTrajectoryAddPointRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,1,2,0.5,0,1,0x16);
                        kopterTrajectoryAddPointRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,2,5,-1,0,2,0x16);
                        kopterTrajectoryAddPointRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,3,7,-1,0,0.5,0x16);
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                        break;
                    case 4: kopterTrajectoryPointStatusRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,0x02);
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                        break;
                    default:
                        printf("WRONG NUMBER\n");
                        break;
             }
             break;
        default:
                printf("WRONG NUMBER\n");
            break;
       }

    }while(decison!=0);

    #ifdef MATLAB
    closeSerialLine(matlabH);
    #endif // MATLAB
    closeXBeeComm();
}
