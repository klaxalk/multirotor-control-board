#include "failuredetection.h"
#include <iostream>
#include <fstream>
#include <string>
#include "receive.h"
#include "send.h"
#include "mpc/elevator/elevatorMpc.h"
#include "mpc/aileron/aileronMpc.h"
#include "Eigen/Dense"
#include "controllers.h"
#include "modeldata.h"

/**
 * chyby měření:
 * gumstix= [gm,px4flow]=mesurment_covariance=diag[0.0058,0.08-pozice a vyska-0.0153^2,0,0,0]
 * diagonala covariacni matice -tedy prvni cislo je rozptyl mereni gumstyxe druhy je px4flow a
 * kdyz chci odchylku musim to odmocnit(odmocnit pro standartni odchylku)
 * nasimuluju- normalni rozdeleni
 * precist si o sumu z normalniho rozdeleni
 * v Ccku normalni distribucni rozdeleni : suma 1/2* (aproximace pro random normalni rozdeleni)
 * Px4flow
 * vzorek z normalniho rozdeleni prictu k te hodnote
 *
 * smazat regulatory a doplnit akcni zasah z modelu
 */

using Eigen::MatrixXf;
using namespace std;
void failuredetection:: run(){
    m_abort=false;
    for(int i=0;i<300;i++)
    {
        differenceOfDataElevator[i]=0;
    }
    for(int i=0;i<300;i++)
    {
        differenceOfDataAileron[i]=0;
    }
    for(int i=0;i<300;i++)
    {
        differenceOfDataHeight[i]=0;
    }
    pointerToPosition=0;
    pointerToHeight=0;
    SumDifferenceElevator=0;
    SumDifferenceAileron=0;
    SumDifferenceHeight=0;
    PositionEnabled=true;
    HeightEnabled=true;
    // inicializace matic modelu a regulatoru
    // pozicni system
//    mpcHandler_t * elevatorMpcHandler = initializeElevatorMPC();
//    mpcHandler_t * aileronMpcHandler = initializeAileronMPC();
    float dt =0.0114;
    MatrixA=MatrixXf(5,5);
    MatrixA <<  1,dt,0,0,0,
                0,1,dt,0,0,
                0,0,0,1,1,
                0,0,0,0.9796,0,
                0,0,0,0,1;

    MatrixB=MatrixXf(5,1);
    MatrixB <<  0,0,0,0.0000513,0;

    MatrixXElevator=MatrixXf(5,1);
    MatrixXElevator <<  0,0,0,0,0;

    MatrixXAileron=MatrixXf(5,1);
    MatrixXAileron <<  0,0,0,0,0;

    MatrixUElevator=MatrixXf(1,1);
    MatrixUElevator <<-10;

    MatrixUAileron=MatrixXf(1,1);
    MatrixUAileron <<-10;

    MatrixXPrevElevator=MatrixXf(5,1);
    MatrixXPrevElevator <<1,0,0,0,0;

    MatrixXPrevAileron=MatrixXf(5,1);
    MatrixXPrevAileron <<0,1,0,0,0;

    // vyskovy system

    float P1 = 0.9658;
    float P2 = 0.0023;
    float g=9.8;

    MatrixAHeight=MatrixXf(4,4);
    MatrixAHeight <<    1,dt,0,0,
            0,1,dt,0,
            0,0,0,1,
            0,0,0,P1;

    MatrixBHeight=MatrixXf(4,2);
    MatrixBHeight <<  0,0,
            0,0,
            0,-g,
            P2,0;

    MatrixXHeight=MatrixXf(4,1);
    MatrixXHeight <<  0,0,0,0;

    MatrixUHeight=MatrixXf(2,1);
    MatrixUHeight <<0,1;

    MatrixXPrevHeight=MatrixXf(4,1);
    MatrixXPrevHeight <<0,0,0,0;

    while(!m_abort)
    {
  /*      dataValues[0] = getTelemetry(kopter,TELEMETRIES.GROUND_DISTANCE_ESTIMATED);
        dataValues[1] = getTelemetry(kopter,TELEMETRIES.GROUND_DISTANCE);
        dataValues[2] = getTelemetry(kopter,TELEMETRIES.ELEVATOR_SPEED);
        dataValues[3] = getTelemetry(kopter,TELEMETRIES.AILERON_SPEED);
        dataValues[4] = getTelemetry(kopter,TELEMETRIES.ELEVATOR_SPEED_ESTIMATED);
        dataValues[5] = getTelemetry(kopter,TELEMETRIES.AILERON_SPEED_ESTIMATED);
        dataValues[6] = getTelemetry(kopter,TELEMETRIES.ELEVATOR_POS_ESTIMATED);
        dataValues[7] = getTelemetry(kopter,TELEMETRIES.AILERON_POS_ESTIMATED);
        dataValues[8] = getTelemetry(kopter,TELEMETRIES.THROTTLE_CONTROLLER_OUTPUT);
        dataValues[9] = getTelemetry(kopter,TELEMETRIES.THROTTLE_SPEED);
        dataValues[10] = getTelemetry(kopter,TELEMETRIES.AILERON_VEL_CONTROLLER_OUTPUT);
        dataValues[11] = getTelemetry(kopter,TELEMETRIES.ELEVATOR_VEL_CONTROLLER_OUTPUT);
        dataValues[12] = getTelemetry(kopter,TELEMETRIES.AILERON_POS_CONTROLLER_OUTPUT);
        dataValues[13] = getTelemetry(kopter,TELEMETRIES.ELEVATOR_POS_CONTROLLER_OUTPUT);
        dataValues[14] = getTelemetry(kopter,TELEMETRIES.THROTTLE_SETPOINT);
        dataValues[15] = getTelemetry(kopter,TELEMETRIES.ELEVATOR_POS_SETPOINT);
        dataValues[16] = getTelemetry(kopter,TELEMETRIES.AILERON_POS_SETPOINT);
        dataValues[17] = getTelemetry(kopter,TELEMETRIES.ELEVATOR_VEL_SETPOINT);
        dataValues[18] = getTelemetry(kopter,TELEMETRIES.AILERON_VEL_SETPOINT);
        dataValues[19] = getTelemetry(kopter,TELEMETRIES.ELEVATOR_SPEED_ESTIMATED2);
        dataValues[20] = getTelemetry(kopter,TELEMETRIES.AILERON_SPEED_ESTIMATED2);
        dataValues[21] = getTelemetry(kopter,TELEMETRIES.ELEVATOR_ACC);
        dataValues[22] = getTelemetry(kopter,TELEMETRIES.AILERON_ACC);
        dataValues[23] = getTelemetry(kopter,TELEMETRIES.VALID_GUMSTIX);
        dataValues[24] = getTelemetry(kopter,TELEMETRIES.ELEVATOR_DESIRED_SPEED_POS_CONT);
        dataValues[25] = getTelemetry(kopter,TELEMETRIES.AILERON_DESIRED_SPEED_POS_CONT);
        dataValues[26] = getTelemetry(kopter,TELEMETRIES.ELE_DES_SPEED_POS_CONT_LEADER);
        dataValues[27] = getTelemetry(kopter,TELEMETRIES.AIL_DES_SPEED_POS_CONT_LEADER);
        dataValues[28] = getTelemetry(kopter,TELEMETRIES.OUTPUT_THROTTLE);
        dataValues[29] = getTelemetry(kopter,TELEMETRIES.OUTPUT_ELEVATOR);
        dataValues[30] = getTelemetry(kopter,TELEMETRIES.OUTPUT_AILERON);
        dataValues[31] = getTelemetry(kopter,TELEMETRIES.OUTPUT_RUDDER);
        dataValues[32] = getTelemetry(kopter,TELEMETRIES.BLOB_DISTANCE);
        dataValues[33] = getTelemetry(kopter,TELEMETRIES.BLOB_HORIZONTAL);
        dataValues[34] = getTelemetry(kopter,TELEMETRIES.BLOB_VERTICAL);
        dataValues[35] = getTelemetry(kopter,TELEMETRIES.PITCH_ANGLE);
        dataValues[36] = getTelemetry(kopter,TELEMETRIES.ROLL_ANGLE);

        sprintf(buffer,"testVlakna_%c",kopter);

        ofstream outputFile(buffer,ios::out | ios::app);
        outputFile << samples;
        outputFile << ",";
        for(int i=0;i<37;i++){
            outputFile << dataValues[i];
            outputFile << ",";
        }
        outputFile << "\n";
        outputFile.close();
        samples++;*/

        // naplneni matic X podle dat z quad !!!!!!!!!!!!!!!!!!!
        // pro pozici zasumime rychlost a pro vysku zasumime mereni vysky !!!
        // nezapomenout na vstup-u, nestaci jen krmit data z X

        // pozicni
       // if(PositionEnabled){
        MatrixUElevator(0,0)=getData(15);
        MatrixXElevator=MatrixA*MatrixXPrevElevator+MatrixB*MatrixUElevator;
        MatrixXPrevElevator=MatrixXElevator;
        for(int i=0;i<5;i++)
        {
        setData(MatrixXElevator(i,0),(i+20));
        }


            MatrixUAileron(0,0)=getData(16);
            MatrixXAileron=MatrixA*MatrixXPrevAileron+MatrixB*MatrixUAileron;
            MatrixXPrevAileron=MatrixXAileron;
            for(int i=0;i<5;i++)
            {
            setData(MatrixXAileron(i,0),(i+25));
            }

            differenceOfDataElevator[pointerToPosition]=fabs(getData(1)-MatrixXElevator(1,0));
            SumDifferenceElevator=0;
            for(int i=0;i<300;i++)
            {
                SumDifferenceElevator+=differenceOfDataElevator[i];
            }
            differenceOfDataAileron[pointerToPosition]=fabs(getData(6)-MatrixXAileron(1,0));
            SumDifferenceAileron=0;
            for(int i=0;i<300;i++)
            {
                SumDifferenceAileron+=differenceOfDataAileron[i];
            }
            pointerToPosition++;
            setData(SumDifferenceElevator,35);
            setData(SumDifferenceAileron,36);


     //  }


        // vyskovy
     //   if(HeightEnabled)
     //  {
            MatrixUHeight(0,0)=getData(17);
            MatrixXHeight=MatrixAHeight*MatrixXPrevHeight+MatrixBHeight*MatrixUHeight;
            MatrixXPrevHeight=MatrixXHeight;
            for(int i=0;i<4;i++)
            {
            setData(MatrixXHeight(i,0),(i+30));
            }
            differenceOfDataHeight[pointerToHeight]=fabs(getData(10)-MatrixXHeight(0,0));
                    pointerToHeight++;
                    SumDifferenceHeight=0;
            for(int i=0;i<300;i++)
            {
                SumDifferenceHeight+=differenceOfDataHeight[i];
            }
            setData(SumDifferenceHeight,37);
       // }

        // sleep
            if(pointerToHeight==300){
                pointerToHeight=0;
                MatrixXPrevHeight(0,0)=getData(10);
                MatrixXPrevHeight(1,0)=getData(11);
                MatrixXPrevHeight(2,0)=getData(12);
                MatrixXPrevHeight(3,0)=getData(13);
            }
            if(pointerToPosition==300){
                pointerToPosition=0;
                MatrixXPrevElevator(0,0)=getData(0);
                MatrixXPrevElevator(1,0)=getData(1);
                MatrixXPrevElevator(2,0)=getData(2);
                MatrixXPrevElevator(3,0)=getData(3);
                MatrixXPrevElevator(4,0)=getData(4);

                MatrixXPrevAileron(0,0)=getData(5);
                MatrixXPrevAileron(1,0)=getData(6);
                MatrixXPrevAileron(2,0)=getData(7);
                MatrixXPrevAileron(3,0)=getData(8);
                MatrixXPrevAileron(4,0)=getData(9);

            }
        this->msleep(14);
    }
}

void failuredetection:: setKopter(unsigned char kpt){
    kopter=kpt;
}

int failuredetection:: getErrors(int index){
  return fieldOfError[index];
}

void failuredetection::setPositionEnable(bool value)
{
    if(PositionEnabled==false&&value==true)
        {
    PositionEnabled=true;
    // synchronizace dat - elevator,aileron TODO

    }else if(PositionEnabled==true&&value==false)PositionEnabled=false;
}

void failuredetection::setHeightEnable(bool value)
{
    //synchronizace dat - height    TODO
    if(HeightEnabled==false&&value==true)
        {
        HeightEnabled=true;
        //synchronizace dat - height    TODO

    }else if(HeightEnabled==true&&value==false)HeightEnabled=false;
}

