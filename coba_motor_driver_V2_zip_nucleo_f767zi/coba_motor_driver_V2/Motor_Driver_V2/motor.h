#ifndef MOTOR_H
#define MOTOR_H

#include "mbed.h"

class motor
{
public :
    motor(PinName direction, PinName pwmpin);
    void setpwm(float inputpwm);
        
private :
    PwmOut _pwmpin;
    DigitalOut _direction;
};
#endif
