#include "checkthread.h"

void checkThread:: run(){
        m_abort=false;
        while(!m_abort)
        {
            checkSerial();
        }
}


