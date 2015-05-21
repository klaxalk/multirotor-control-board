#ifndef MODEL_H
#define MODEL_H

#include <QThread>


class Model : public QThread {
public:
    bool m_abort;
    float data[18];
    float suma;
    float sumaTMP;
    float getData(int index);
    void setPoints(float setPointAileron,float setPointElevator,float setPointHeight);
protected:
    void run();

};

#endif // MODEL_H
