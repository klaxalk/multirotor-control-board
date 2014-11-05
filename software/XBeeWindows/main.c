#include <stdio.h>
#include <windows.h>
#include "XBeeComm.h"
#include "packets.h"
#include "commands.h"
#include "serialLink.h"

HANDLE matlabH;

int main()
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
            printf("\nTELEMTRY: 1 - GROUND DISTANCE   2 - ELEVATOR SPEED   3 - AILERON SPEED   4 - ELEVATOR POS   5 - AILERON POS   6 - THROTTLE SETPOINT   7 - ELEVATOR VEL SETPOINT   8 - AILERON VEL SETPOINT\n");
            printf("DECISION:");
            scanf("%d",&decison);
                    switch(decison){
                        case 1:
                            telemetryRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,TELEMETRIES.GROUND_DISTANCE,0x01);
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            break;
                        case 2:
                            telemetryRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,TELEMETRIES.ELEVATOR_SPEED,0x01);
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            break;
                        case 3:
                            telemetryRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,TELEMETRIES.AILERON_SPEED,0x01);
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            break;
                        case 4:
                            telemetryRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,TELEMETRIES.ELEVATOR_POS_ESTIMATED,0x01);
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            break;
                        case 5:
                            telemetryRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,TELEMETRIES.AILERON_POS_ESTIMATED,0x01);
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            break;
                        case 6:
                            telemetryRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,TELEMETRIES.THROTTLE_SETPOINT,0x01);
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            break;
                        case 7:
                            telemetryRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,TELEMETRIES.ELEVATOR_VEL_SETPOINT,0x01);
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            break;
                        case 8:
                            telemetryRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,TELEMETRIES.AILERON_VEL_SETPOINT,0x01);
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
                            kopterLandRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,ONOFF.ON,0x02);
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            break;
                        case 2:
                            kopterLandRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,ONOFF.OFF,0x02);
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
            printf("\nSETPOINTS (POS RELATIVE): 1 - SET THROTTLE    2 - SET ELEVATOR POS   3 - SET AILERON POS\n4 - SET ELEVATOR VEL   5 - SET AILERON VEL \n");
            printf("6 - STATUS THROTTLE 7 - STATUS ELEVATOR POS   8 - STATUS AILERON POS\n9 - STATUS ELEVATOR VEL   10 - STATUS AILERON VEL\n");
            printf("DECISION:");
            scanf("%d",&decison);
            if (decison<6 && decison>0){
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
                            kopterSetpointsSetRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,SETPOINTS.ELEVATOR_POSITION,POSITIONS.RELATIV,f,0x02);
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            break;
                        case 3:
                            kopterSetpointsSetRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,SETPOINTS.AILERON_POSITION,POSITIONS.RELATIV,f,0x02);
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            break;
                        case 4:
                            kopterSetpointsSetRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,SETPOINTS.ELEVATOR_VELOCITY,POSITIONS.ABSOLUT,f,0x02);
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            break;
                        case 5:
                            kopterSetpointsSetRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,SETPOINTS.AILERON_VELOCITY,POSITIONS.ABSOLUT,f,0x02);
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            break;

                        case 6:
                            kopterSetpointStatusRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,SETPOINTS.THROTTLE,0x02);
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            break;
                        case 7:
                            kopterSetpointStatusRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,SETPOINTS.ELEVATOR_POSITION,0x02);
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            break;
                        case 8:
                            kopterSetpointStatusRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,SETPOINTS.AILERON_POSITION,0x02);
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            break;
                        case 9:
                            kopterSetpointStatusRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,SETPOINTS.ELEVATOR_VELOCITY,0x02);
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            break;
                        case 10:
                            kopterSetpointStatusRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,SETPOINTS.AILERON_VELOCITY,0x02);
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
            printf("1 - OFF   2 - VELOCITY   3 - POSITION   4 - BOTH   5 - STATUS\n");
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
                    case 4:
                        kopterControllersRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,CONTROLLERS.BOTH,0x13);
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                        break;
                    case 5:
                        kopterControllersStatusRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,0x13);
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
                        kopterTrajectoryRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,ONOFF.ON,0x10);
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                        break;
                    case 2:
                        kopterTrajectoryRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,ONOFF.OFF,0x10);
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                            packetHandler(readPacket());
                        break;
                    case 3:
                        kopterTrajectoryAddPointRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,0,2,-1.5,0,0.75,0x00);
                        kopterTrajectoryAddPointRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,1,2,   0,0, 0.5,0x00);
                        kopterTrajectoryAddPointRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,2,2,  -1,0,   1,0x00);
                        kopterTrajectoryAddPointRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,3,2,   0,0, 0.5,0x00);
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
    return 0;
}
