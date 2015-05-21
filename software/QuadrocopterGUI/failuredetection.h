#ifndef FAILUREDETECTION_H
#define FAILUREDETECTION_H
#include <QThread>
#include "Eigen/Dense"


using Eigen::MatrixXf;
class failuredetection : public QThread
{
public:
    //pozicni system - definice matic
    MatrixXf MatrixA;
    MatrixXf MatrixB;
    MatrixXf MatrixXElevator;
    MatrixXf MatrixXAileron;
    MatrixXf MatrixUElevator;
    MatrixXf MatrixUAileron;
    MatrixXf MatrixXPrevElevator;
    MatrixXf MatrixXPrevAileron;

    // vyskovy system - definice matic

    MatrixXf MatrixAHeight;
    MatrixXf MatrixBHeight;
    MatrixXf MatrixXHeight;
    MatrixXf MatrixUHeight;
    MatrixXf MatrixXPrevHeight;

    bool m_abort;
    unsigned char kopter;
    char buffer[80];
    int dataValues[37];
    int samples=0;
    void setKopter(unsigned char kopter);
    int getErrors(int index);
    int fieldOfError[10]={};
    float data[16];
    float differenceOfDataElevator[300];
    float SumDifferenceElevator;
    float differenceOfDataAileron[300];
    float SumDifferenceAileron;
    float differenceOfDataHeight[300];
    float SumDifferenceHeight;
    int pointerToPosition;
    int pointerToHeight;
    bool PositionEnabled;
    bool HeightEnabled;
    void setPositionEnable(bool value);
    void setHeightEnable(bool value);
protected:
    void run();

};

#endif // FAILUREDETECTION_H
