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
    PositionEnabled=false;
    HeightEnabled=false;
    // inicializace matic modelu a regulatoru
    // pozicni system
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
            0,0,
            P2,0;

    MatrixXHeight=MatrixXf(4,1);
    MatrixXHeight <<  0,0,0,0;

    MatrixUHeight=MatrixXf(2,1);
    MatrixUHeight <<0,1;

    MatrixXPrevHeight=MatrixXf(4,1);
    MatrixXPrevHeight <<0,0,0,0;
    sprintf(buffer,"FailLog_Oprava3.txt%c",kopter);
    while(!m_abort)
    {

        sprintf(buffer,"testVlakna_%c",kopter);

        ofstream outputFile("AZ3.txt",ios::out | ios::app);
        outputFile << getTelemetry(kopter,TELEMETRIES.ELEVATOR_CONTROLLER_OUTPUT);
        outputFile << "\n";
        outputFile.close();

        // naplneni matic X podle dat z quad !!!!!!!!!!!!!!!!!!!
        // pro pozici zasumime rychlost a pro vysku zasumime mereni vysky !!!
        // nezapomenout na vstup-u, nestaci jen krmit data z X

        // pozicni
        float FieldForTest[300];
        float TMP;
        float median;
        float pkpk,max,min,min2,max2,pkpk2;
        if(PositionEnabled)
        {
            MatrixUElevator(0,0)=getTelemetry(kopter,TELEMETRIES.ELEVATOR_CONTROLLER_OUTPUT);
            MatrixXPrevElevator(4,0)=getTelemetry(kopter,TELEMETRIES.ELEVATOR_ACC_ERROR);
            MatrixXElevator=MatrixA*MatrixXPrevElevator+MatrixB*MatrixUElevator;
            MatrixXPrevElevator=MatrixXElevator;
            for(int i=0;i<5;i++)
            {
                setData(MatrixXElevator(i,0),(i+20));
            }


            MatrixUAileron(0,0)=getTelemetry(kopter,TELEMETRIES.AILERON_CONTROLLER_OUTPUT);
            MatrixXPrevAileron(4,0)=getTelemetry(kopter,TELEMETRIES.AILERON_ACC_ERROR);
            MatrixXAileron=MatrixA*MatrixXPrevAileron+MatrixB*MatrixUAileron;
            MatrixXPrevAileron=MatrixXAileron;
            for(int i=0;i<5;i++)
            {
                setData(MatrixXAileron(i,0),(i+25));
            }

            differenceOfDataElevator[pointerToPosition]=fabs(getTelemetry(kopter,TELEMETRIES.ELEVATOR_POSITION)-MatrixXElevator(0,0));
            differenceOfDataAileron[pointerToPosition]=getTelemetry(kopter,TELEMETRIES.ELEVATOR_ACC_ERROR);
            /*SumDifferenceElevator=0;
            for(int i=0;i<300;i++)
            {
                SumDifferenceElevator+=differenceOfDataElevator[i];
                FieldForTest[i]=differenceOfDataElevator[i];
            }


            /*for (int i = 1; i < 300; i++) {
                for (int bubble = 300 - 1; bubble >= i; bubble--) {
                    if (FieldForTest[bubble - 1] > FieldForTest[bubble]) {
                        TMP = FieldForTest[bubble - 1];
                        FieldForTest[bubble - 1] = FieldForTest[bubble];
                        FieldForTest[bubble] = TMP;
                    }
                }
            }
            median=((FieldForTest[149]+FieldForTest[150])/2);*/
            max2=min2=max=min=0;
            for(int i=0;i<300; i++){

                if (max < differenceOfDataElevator[i]){
                    max = differenceOfDataElevator[i];}
                if (min > differenceOfDataElevator[i]){
                    min = differenceOfDataElevator[i];}
                if (max2 < differenceOfDataAileron[i]){
                    max2 = differenceOfDataAileron[i];}
                if (min2 > differenceOfDataAileron[i]){
                    min2 = differenceOfDataAileron[i];}
            }
            pkpk=max-min;
            pkpk2=max2-min2;
            //setData(SumDifferenceElevator,40);
            //setData(median,41);
            setData(pkpk,42);
            if(pkpk>0.35)
            {
                fieldOfError[0]=1;}
            else if(pkpk2>0.25) {fieldOfError[0]=1;
            }else fieldOfError[0]=0;

            differenceOfDataAileron[pointerToPosition]=fabs(getTelemetry(kopter,TELEMETRIES.AILERON_POSITION)-MatrixXAileron(0,0));
            /*SumDifferenceAileron=0;
            for(int i=0;i<300;i++)
            {
                SumDifferenceAileron+=differenceOfDataAileron[i];
            }*/
            pointerToPosition++;

            pkpk=max=min=0;
            for(int i=0;i<300; i++){
                if (max < differenceOfDataAileron[i]){
                    max = differenceOfDataAileron[i];}
                if (min > differenceOfDataAileron[i]){
                    min = differenceOfDataAileron[i];}
            }
            pkpk=max-min;
            setData(pkpk,43);
            if(pkpk>0.35)
            {
                 //fieldOfError[1]=1;
            }
            else fieldOfError[1]=0;

        }

        // vyskovy
        if(HeightEnabled)
        {
            //for(int i=0;i<8;i++)printf("%i ",*(kopter+i);
            MatrixUHeight(0,0)=(getTelemetry(kopter,TELEMETRIES.ALTITUDE_CONTROLLER_OUTPUT)-getTelemetry(kopter,TELEMETRIES.ALTITUDE_INTEGRATED_COMPONENT));

            //printf("%f ",getTelemetry(kopter,TELEMETRIES.ALTITUDE_CONTROLLER_OUTPUT));
            setData((getTelemetry(kopter,TELEMETRIES.ALTITUDE_CONTROLLER_OUTPUT)-getTelemetry(kopter,TELEMETRIES.ALTITUDE_INTEGRATED_COMPONENT)),49);
            MatrixXHeight=MatrixAHeight*MatrixXPrevHeight+MatrixBHeight*MatrixUHeight;
            MatrixXPrevHeight=MatrixXHeight;
            for(int i=0;i<4;i++)
            {
                setData(MatrixXHeight(i,0),(i+30));
            }

            differenceOfDataHeight[pointerToHeight]=fabs(getTelemetry(kopter,TELEMETRIES.ALTITUDE)-MatrixXHeight(0,0));
            pointerToHeight++;
            /*        SumDifferenceHeight=0;
            for(int i=0;i<300;i++)
            {
                SumDifferenceHeight+=differenceOfDataHeight[i];
                FieldForTest[i]=differenceOfDataHeight[i];
            }
            setData(SumDifferenceHeight,37);


            for (int i = 1; i < 300; i++) {
                for (int bubble = 300 - 1; bubble >= i; bubble--) {
                    if (FieldForTest[bubble - 1] > FieldForTest[bubble]) {
                        TMP = FieldForTest[bubble - 1];
                        FieldForTest[bubble - 1] = FieldForTest[bubble];
                        FieldForTest[bubble] = TMP;
                    }
                }
            }
            median=((FieldForTest[149]+FieldForTest[150])/2);*/
            max=min=0;
            for(int i=0;i<300; i++){

                if (max < differenceOfDataHeight[i]){
                    max = differenceOfDataHeight[i];}
                if (min > differenceOfDataHeight[i]){
                    min = differenceOfDataHeight[i];}
            }
            pkpk=max-min;
            //setData(SumDifferenceHeight,43);
            //setData(median,44);
            setData(pkpk,44);

            if(pkpk>1 && kopter==KOPTERS.K1)
            {
              //    fieldOfError[2]=1;
            }
            else if(pkpk>1.2 && kopter==KOPTERS.K3)
            {
               //   fieldOfError[2]=1;
            }
            else fieldOfError[2]=0;

            ofstream outputFile("video2.txt",ios::out | ios::app);
            outputFile << samples;
            outputFile << ",";
            for(int i=0;i<50;i++){
                outputFile << getData(i);
                outputFile << ",";
            }
            //outputFile << "\n";
            //outputFile.close();
            samples++;

            outputFile << ",";
            outputFile << getTelemetry(kopter,TELEMETRIES.ELEVATOR_POSITION);
            outputFile << ",";
            outputFile << getTelemetry(kopter,TELEMETRIES.ELEVATOR_SPEED);
            outputFile << ",";
            outputFile <<getTelemetry(kopter,TELEMETRIES.AILERON_POSITION);
            outputFile << ",";
            outputFile <<getTelemetry(kopter,TELEMETRIES.AILERON_SPEED);
            outputFile << ",";
            outputFile <<getTelemetry(kopter,TELEMETRIES.ALTITUDE);
            outputFile << ",";
            outputFile << getTelemetry(kopter,TELEMETRIES.ALTITUDE_SPEED);
            outputFile << ",";
            outputFile << "\n";
            outputFile.close();
        }
        // sleep
        if(pointerToHeight==150){
            pointerToHeight=0;
            MatrixXPrevHeight(0,0)=getTelemetry(kopter,TELEMETRIES.ALTITUDE);//getData(10);//TELEMETRIES.ALTITUDE);
            MatrixXPrevHeight(1,0)=getTelemetry(kopter,TELEMETRIES.ALTITUDE_SPEED);//getData(11);//TELEMETRIES.ALTITUDE_SPEED); rychlost
            MatrixXPrevHeight(2,0)=0;//getData(12);// skutecna zrychleni-gravitace+zrychleni-0
            MatrixXPrevHeight(3,0)=0;//9.8;//getData(13);// -zrychleni z AZ-0
            /*
                MatrixXPrevHeight(0,0)=getData(10);//TELEMETRIES.ALTITUDE);
                MatrixXPrevHeight(1,0)=getData(11);//TELEMETRIES.ALTITUDE_SPEED); rychlost
                MatrixXPrevHeight(2,0)=getData(12);// skutecna zrychleni-gravitace+zrychleni-0
                MatrixXPrevHeight(3,0)=getData(13);// -zrychleni z AZ-0*/
        }
        if(pointerToPosition==150){
            pointerToPosition=0;
            MatrixXPrevElevator(0,0)=getTelemetry(kopter,TELEMETRIES.ELEVATOR_POSITION);//getData(0);//ELEVATOR_POSITION
            MatrixXPrevElevator(1,0)=getTelemetry(kopter,TELEMETRIES.ELEVATOR_SPEED_ESTIMATED);//getData(1);//ELEVATOR_SPEED
            MatrixXPrevElevator(2,0)=getTelemetry(kopter,TELEMETRIES.ELEVATOR_ACC);//getData(2);//ELEVATOR_ACC
            MatrixXPrevElevator(3,0)=getTelemetry(kopter,TELEMETRIES.ELEVATOR_ACC_INPUT);//getData(3);// ELEVATOR_ACC_INPUT
            MatrixXPrevElevator(4,0)=getTelemetry(kopter,TELEMETRIES.ELEVATOR_ACC_ERROR);//getData(4);//ELEVATOR_ACC_ERROR

            MatrixXPrevAileron(0,0)=getTelemetry(kopter,TELEMETRIES.AILERON_POSITION);//getData(5);//AILERON_POSITION
            MatrixXPrevAileron(1,0)=getTelemetry(kopter,TELEMETRIES.AILERON_SPEED_ESTIMATED);//getData(6);//AILERON_SPEED
            MatrixXPrevAileron(2,0)=getTelemetry(kopter,TELEMETRIES.AILERON_ACC);//getData(7);//AILERON_ACC-skutecne zrychleni
            MatrixXPrevAileron(3,0)=getTelemetry(kopter,TELEMETRIES.AILERON_ACC_INPUT);//getData(8);//AILERON_ACC_INPUT
            MatrixXPrevAileron(4,0)=getTelemetry(kopter,TELEMETRIES.AILERON_ACC_ERROR);//getData(9);//AILERON_ACC_ERROR
            /*
                MatrixXPrevElevator(0,0)=getData(0);//ELEVATOR_POSITION
                MatrixXPrevElevator(1,0)=getData(1);//ELEVATOR_SPEED
                MatrixXPrevElevator(2,0)=getData(2);//ELEVATOR_ACC
                MatrixXPrevElevator(3,0)=getData(3);// ELEVATOR_ACC_INPUT
                MatrixXPrevElevator(4,0)=getData(4);//ELEVATOR_ACC_ERROR

                MatrixXPrevAileron(0,0)=getData(5);//AILERON_POSITION
                MatrixXPrevAileron(1,0)=getData(6);//AILERON_SPEED
                MatrixXPrevAileron(2,0)=getData(7);//AILERON_ACC-skutecne zrychleni
                MatrixXPrevAileron(3,0)=getData(8);//AILERON_ACC_INPUT
                MatrixXPrevAileron(4,0)=getData(9);//AILERON_ACC_ERROR*/

        }
        this->msleep(10);
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
        pointerToPosition=0;
        MatrixXPrevElevator(0,0)=getTelemetry(kopter,TELEMETRIES.ELEVATOR_POSITION);//getData(0);//ELEVATOR_POSITION
        MatrixXPrevElevator(1,0)=getTelemetry(kopter,TELEMETRIES.ELEVATOR_SPEED);//getData(1);//ELEVATOR_SPEED
        MatrixXPrevElevator(2,0)=getTelemetry(kopter,TELEMETRIES.ELEVATOR_ACC);//getData(2);//ELEVATOR_ACC
        MatrixXPrevElevator(3,0)=getTelemetry(kopter,TELEMETRIES.ELEVATOR_ACC_INPUT);//getData(3);// ELEVATOR_ACC_INPUT
        MatrixXPrevElevator(4,0)=getTelemetry(kopter,TELEMETRIES.ELEVATOR_ACC_ERROR);//getData(4);//ELEVATOR_ACC_ERROR

        MatrixXPrevAileron(0,0)=getTelemetry(kopter,TELEMETRIES.AILERON_POSITION);//getData(5);//AILERON_POSITION
        MatrixXPrevAileron(1,0)=getTelemetry(kopter,TELEMETRIES.AILERON_SPEED);//getData(6);//AILERON_SPEED
        MatrixXPrevAileron(2,0)=getTelemetry(kopter,TELEMETRIES.AILERON_ACC);//getData(7);//AILERON_ACC-skutecne zrychleni
        MatrixXPrevAileron(3,0)=getTelemetry(kopter,TELEMETRIES.AILERON_ACC_INPUT);//getData(8);//AILERON_ACC_INPUT
        MatrixXPrevAileron(4,0)=getTelemetry(kopter,TELEMETRIES.AILERON_ACC_ERROR);//getData(9);//AILERON_ACC_ERROR


    }else if(PositionEnabled==true&&value==false)PositionEnabled=false;
}

void failuredetection::setHeightEnable(bool value)
{
    if(HeightEnabled==false&&value==true)
    {
        HeightEnabled=true;

        MatrixXPrevHeight(0,0)=getTelemetry(kopter,TELEMETRIES.ALTITUDE);//getData(10);//TELEMETRIES.ALTITUDE);
        MatrixXPrevHeight(1,0)=getTelemetry(kopter,TELEMETRIES.ALTITUDE_SPEED);//getData(11);//TELEMETRIES.ALTITUDE_SPEED); rychlost
        MatrixXPrevHeight(2,0)=0;//getData(12);// skutecna zrychleni-gravitace+zrychleni-0
        MatrixXPrevHeight(3,0)=0;//9.8;//getData(13);// -zrychleni z AZ-0

    }else if(HeightEnabled==true&&value==false)HeightEnabled=false;
}

