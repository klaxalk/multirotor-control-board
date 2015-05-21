#ifndef CHECKTHREAD_H
#define CHECKTHREAD_H

#include <QThread>
#include "serial.h"


class checkThread : public QThread {
public:
    bool m_abort;
protected:
    void run();

};


#endif // CHECKTHREAD_H
