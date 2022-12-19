#ifndef CMPS11_H
#define CMPS11_H

#include "mbed.h"

class cmps11 
{
    
public:
    cmps11(PinName tx , PinName rx );
    char startCalibration() ;
    char stopCalibration();
    int readCompassAngle16Bit() ;
    void set0degree();
    int readAngle16Bit() ;
    
private:
    Serial _cmps11 ;
    char readstate , buffer[2] , modelast , modenow ;
    int readnow , readlast , sudut , bufferx;
    
};

#endif
