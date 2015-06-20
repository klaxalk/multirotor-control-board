#include "model.h"
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

using Eigen::MatrixXf;
using namespace std;

void Model:: run(){
    m_abort=false;

    mpcHandler_t * elevatorMpcHandler = initializeElevatorMPC();
    mpcHandler_t * aileronMpcHandler = initializeAileronMPC();
    //inicializace modelu systemu
    // pozicni system(Aileron,Elevator)
    float dt =0.0114;
    MatrixXf MatrixA(5,5);
    MatrixA <<  1,dt,0,0,0,
            0,1,dt,0,0,
            0,0,0,1,1,
            0,0,0,0.9796,0,
            0,0,0,0,1;

    MatrixXf MatrixB(5,1);
    MatrixB <<  0,0,0,0.0000513,0;

    MatrixXf MatrixXElevator(5,1);
    MatrixXElevator <<  0,0,0,0,0;

    MatrixXf MatrixXAileron(5,1);
    MatrixXAileron <<  0,0,0,0,0;

    MatrixXf MatrixUElevator(1,1);
    MatrixUElevator <<0;

    MatrixXf MatrixUAileron(1,1);
    MatrixUAileron <<0;

    MatrixXf MatrixXPrevElevator(5,1);
    MatrixXPrevElevator <<0,0,0,0,0;

    MatrixXf MatrixXPrevAileron(5,1);
    MatrixXPrevAileron <<0,0,0,0,0;

    // vyskovy system

    float P1 = 0.9658;
    float P2 = 0.0023;
    float g=9.8;

    MatrixXf MatrixAHeight(4,4);
    MatrixAHeight <<    1,dt,0,0,
            0,1,dt,0,
            0,0,0,1,
            0,0,0,P1;

    MatrixXf MatrixBHeight(4,2);
    MatrixBHeight <<  0,0,
            0,0,
            0,-g,
            P2,0;

    MatrixXf MatrixXHeight(4,1);
    MatrixXHeight <<  0,0,0,0;

    MatrixXf MatrixUHeight(2,1);
    MatrixUHeight <<0,1;

    MatrixXf MatrixXPrevHeight(4,1);
    MatrixXPrevHeight <<0,0,0,0;

    // konec inicializace modelu
    while(!m_abort)
    {
        /*
             * Matrix-pozice
             * 0-pozice
             * 1-rychlost
             * 2-skutecne zrychleni
             * 3-normalni zrychleni
             * 4-chyba zrychleni
             *
             * Matrix-vyska
             * 0-vyska
             * 1-rychlost
             * 2-skutecna zrychleni
             * 3-normalni zrychleni (od motorů)
             */
        // kod modelu- vypocet dat z helikoptery
        // nutne doplnit model mereni z px4flow

        //Model pro chovani systemu
        //for(int i=0;i<400;i++){
        vector_float_set_to(elevatorMpcHandler->position_reference,10);
        filterReferenceTrajectory(elevatorMpcHandler);
        //MatrixXPrevElevator(4,0)=0.3;
        //MatrixXPrevElevator(0,0)+=0.02;
        vector_float_set(elevatorMpcHandler->initial_cond, 1, MatrixXPrevElevator(0,0));
        vector_float_set(elevatorMpcHandler->initial_cond, 2, MatrixXPrevElevator(1,0));
        vector_float_set(elevatorMpcHandler->initial_cond, 3, MatrixXPrevElevator(2,0));
        vector_float_set(elevatorMpcHandler->initial_cond, 4, MatrixXPrevElevator(3,0));
        vector_float_set(elevatorMpcHandler->initial_cond, 5, MatrixXPrevElevator(4,0));
        MatrixUElevator(0,0)=calculateMPC(elevatorMpcHandler);
        MatrixXElevator=MatrixA*MatrixXPrevElevator+MatrixB*MatrixUElevator;
        MatrixXPrevElevator=MatrixXElevator;
        //šum
        //saturace mereni rychlost max 0.5
        //vejska 0.3-4m omezit podlahu-nejde pod 0
        suma=0;
        for(int i=0;i<12;i++)
        {
            sumaTMP=(rand()% ( 100 - 0 + 1 ));
            suma=suma+(((sumaTMP)-50)/50)*0.08;
        }
        suma=suma/2;

        for(int i=0;i<5;i++)
        {
            data[i]=MatrixXElevator(i,0);
        }
        data[1]=suma+data[1];
        for(int i=0;i<5;i++)
        {
            setData(data[i],i);
        }

        vector_float_set_to(aileronMpcHandler->position_reference,0);
        filterReferenceTrajectory(aileronMpcHandler);
        vector_float_set(aileronMpcHandler->initial_cond, 1, MatrixXPrevAileron(0,0));
        vector_float_set(aileronMpcHandler->initial_cond, 2, MatrixXPrevAileron(1,0));
        vector_float_set(aileronMpcHandler->initial_cond, 3, MatrixXPrevAileron(2,0));
        vector_float_set(aileronMpcHandler->initial_cond, 4, MatrixXPrevAileron(3,0));
        vector_float_set(aileronMpcHandler->initial_cond, 5, MatrixXPrevAileron(4,0));
        MatrixUAileron(0,0)=calculateMPC(aileronMpcHandler);
        MatrixXAileron=MatrixA*MatrixXPrevAileron+MatrixB*MatrixUAileron;
        MatrixXPrevAileron=MatrixXAileron;
        suma=0;
        for(int i=0;i<12;i++)
        {
            sumaTMP=(rand()% ( 100 - 0 + 1 ));
            suma=suma+(((sumaTMP)-50)/50)*0.08;
        }
        suma=suma/2;
        for(int i=0;i<5;i++)
        {
            data[i+5]=MatrixXAileron(i,0);
        }
        data[6]=data[6]+suma;
        for(int i=0;i<5;i++)
        {
            setData(data[i+5],(i+5));
        }
        //}
        // vyskovy
        // for(int i=0;i<1000;i++){
        //MatrixXPrevHeight(0,0)+=0.02;
        MatrixUHeight(0,0)=altitudeController(1,MatrixXPrevHeight(1,0),MatrixXPrevHeight(0,0));
        MatrixXHeight=MatrixAHeight*MatrixXPrevHeight+MatrixBHeight*MatrixUHeight;
        MatrixXPrevHeight=MatrixXHeight;
        suma=0;
        for(int i=0;i<12;i++)
        {
            sumaTMP=(rand()% ( 100 - 0 + 1 ));
            suma=suma+(((sumaTMP)-50)/50)*0.00023409;
        }
        suma=suma/2;
        for(int i=0;i<4;i++)
        {
            data[i+10]=MatrixXHeight(i,0);
        }
        data[10]=suma+data[10];
        for(int i=0;i<4;i++)
        {
            setData(data[i+10],(i+10));
        }

        // akcni zasahy
        /* data[15]=MatrixUElevator(0,0);
                data[16]=MatrixUAileron(0,0);
                data[17]=MatrixUHeight(0,0);*/
        setData(MatrixUElevator(0,0),15);
        setData(MatrixUAileron(0,0),16);
        setData(MatrixUHeight(0,0),17);


        //}
        //sleep
        this->msleep(14);
    }
}
float Model:: getData(int index){
    return data[index];
}
void Model:: setPoints(float setPointAileron,float setPointElevator,float setPointHeight)
{
    // nastaveni setpointu
}

