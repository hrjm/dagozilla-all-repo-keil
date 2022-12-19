#ifndef MOTOR_DAGOZ_H
#define MOTOR_DAGOZ_H

#include "mbed.h"

class MotorDagoz
{
public :
    MotorDagoz(PinName direction, PinName pwmpin);
    void setpwm(float inputpwm);
        
private :
    PwmOut _pwmpin;
    DigitalOut _direction;
};
#endif
